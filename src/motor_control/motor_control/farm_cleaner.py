#!/usr/bin/env python3
"""
farm_cleaner.py

Boss Orchestrator Node for Autonomous Farm Cleaning Robot.
Manages the complete lifecycle:
  IDLE -> MAPPING -> MAP_SAVED -> SWEEPING -> DONE

Features:
  1. Process group management for launch/stop of SLAM & Nav2 stacks (clean TF handoff).
  2. Map saver integration and stale SLAM file cleanup.
  3. Dock localization initialization (0,0,0 initial pose).
  4. Snake waypoint generation filtered by occupancy map (.pgm + .yaml) with robot clearance.
  5. Map PGM -> PNG byte converter for app rendering.
"""

import os
import sys
import io
import math
import time
import signal
import subprocess
import threading
import shlex
from enum import Enum
import yaml
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def robot_home() -> str:
    return os.path.abspath(os.environ.get("ROBOT_HOME", os.path.expanduser("~")))


class S(Enum):
    IDLE = 0
    MAPPING = 1
    MAP_SAVED = 2
    SWEEPING = 3
    DONE = 4


def launch(cmd) -> subprocess.Popen:
    """Launch process in its own session group for clean kill later."""
    if isinstance(cmd, (list, tuple)):
        cmd_str = " ".join(shlex.quote(str(arg)) for arg in cmd)
    else:
        cmd_str = str(cmd)
    print(f"[BossNode] Launching process group: {cmd_str}")
    install_setup = os.path.join(robot_home(), "ros2_ws/install/setup.bash")
    bash_cmd = [
        "bash", "-c",
        f'source /opt/ros/humble/setup.bash; [ -f {shlex.quote(install_setup)} ] && source {shlex.quote(install_setup)}; export ROS_LOG_DIR=/tmp; exec {cmd_str}'
    ]
    return subprocess.Popen(bash_cmd, start_new_session=True)


def stop(p: subprocess.Popen, timeout: float = 10.0):
    """Terminate process group cleanly."""
    if p is None:
        return
    try:
        pgid = os.getpgid(p.pid)
        print(f"[BossNode] Stopping process group (PGID {pgid})...")
        os.killpg(pgid, signal.SIGINT)

        start = time.time()
        while time.time() - start < timeout:
            if p.poll() is not None:
                print(f"[BossNode] Process group {pgid} terminated cleanly.")
                return
            time.sleep(0.5)

        print(f"[BossNode] Process group {pgid} did not exit on SIGINT. Sending SIGKILL...")
        os.killpg(pgid, signal.SIGKILL)
        p.wait()
    except Exception as e:
        print(f"[BossNode] Error stopping process: {e}")


def kill_all_zombies():
    """Kill any zombie nodes that might conflict over serial ports or TFs using exact process names."""
    exact_targets = [
        "async_slam_toolbox_node",
        "amcl",
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
        "rplidar_node",
        "motor_node",
        "encoder_odom_node",
        "robot_state_publisher",
        "rviz2",
        "component_container_isolated"
    ]
    for target in exact_targets:
        subprocess.run(["pkill", "-9", "-x", target], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    # Target only motor_control launch wrappers, never matching bare 'nav2'
    subprocess.run(["pkill", "-9", "-f", "ros2 launch motor_control"], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(1.0)


def yaw_to_quaternion(yaw: float) -> dict:
    """Convert yaw angle (radians) to quaternion dict."""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': float(round(math.sin(yaw / 2.0), 4)),
        'w': float(round(math.cos(yaw / 2.0), 4))
    }


def get_entire_room_bounds(map_yaml_file: str, robot_radius: float = 0.48) -> list:
    """
    Auto-detect the entire room's connected safe bounding box from map.
    Returns [xmin, xmax, ymin, ymax] or None on failure.
    """
    if not os.path.exists(map_yaml_file):
        return None
    try:
        from PIL import Image
        import numpy as np
        from scipy.ndimage import distance_transform_edt, label
        with open(map_yaml_file, 'r') as f:
            meta = yaml.safe_load(f)
        img_rel = meta['image']
        img_path = img_rel if img_rel.startswith('/') else os.path.join(os.path.dirname(map_yaml_file), img_rel)
        if not os.path.exists(img_path):
            return None
        img = Image.open(img_path)
        grid = np.array(img)
        h, w = grid.shape
        res = float(meta['resolution'])
        ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

        occ = (grid < 100)
        dist_m = distance_transform_edt(~occ) * res
        safe = (dist_m >= robot_radius) & (grid >= 250)

        labeled, num_features = label(safe)
        if num_features == 0:
            return None
        component_sizes = [np.sum(labeled == i) for i in range(1, num_features + 1)]
        largest_comp_idx = np.argmax(component_sizes) + 1
        main_comp = (labeled == largest_comp_idx)
        rows, cols = np.where(main_comp)
        xmin = float(round(ox + cols.min() * res, 2))
        xmax = float(round(ox + cols.max() * res, 2))
        ymin = float(round(oy + (h - 1 - rows.max()) * res, 2))
        ymax = float(round(oy + (h - 1 - rows.min()) * res, 2))
        return [xmin, xmax, ymin, ymax]
    except Exception:
        return None


def make_snake(map_yaml_file: str, bounds: list = None, row_gap: float = 0.4, step_x: float = 0.4, robot_radius: float = 0.48) -> list:
    """
    Generate snake (lawnmower) waypoints filtered by map occupancy and robot radius.

    :param map_yaml_file: Full path to map yaml file
    :param bounds: Optional [xmin, xmax, ymin, ymax] bounding box in meters
    :param row_gap: Distance between snake rows (meters)
    :param step_x: Step size along rows (meters)
    :param robot_radius: Safety clearance radius around robot (meters, default 0.48 for Nav2)
    :return: List of filtered safe waypoint dicts
    """
    if not os.path.exists(map_yaml_file):
        print(f"[SnakeFilter] Error: Map YAML file not found: {map_yaml_file}")
        return []

    with open(map_yaml_file, 'r') as f:
        meta = yaml.safe_load(f)

    img_rel = meta['image']
    if not img_rel.startswith('/'):
        img_path = os.path.join(os.path.dirname(map_yaml_file), img_rel)
    else:
        img_path = img_rel

    if not os.path.exists(img_path):
        print(f"[SnakeFilter] Error: Map image file not found: {img_path}")
        return []

    img = Image.open(img_path)
    grid = np.array(img)
    height, width = grid.shape
    res = float(meta['resolution'])
    origin_x = float(meta['origin'][0])
    origin_y = float(meta['origin'][1])

    map_xmin = origin_x
    map_xmax = origin_x + width * res
    map_ymin = origin_y
    map_ymax = origin_y + height * res

    radius_px = int(math.ceil(robot_radius / res))

    # Free cells threshold: auto-detect connected room bounds if bounds is None or empty
    if bounds is None or len(bounds) != 4:
        room_bounds = get_entire_room_bounds(map_yaml_file, robot_radius)
        if room_bounds is None:
            print(f"[SnakeFilter] Error: Could not detect room bounds from map {map_yaml_file}")
            return []
        xmin, xmax, ymin, ymax = room_bounds
        print(f"[SnakeFilter] No custom bounds provided. Auto-selected Entire Room bounds: X:[{xmin:.2f}, {xmax:.2f}], Y:[{ymin:.2f}, {ymax:.2f}]")
    else:
        b = [float(x) for x in bounds]
        xmin = min(b[0], b[1])
        xmax = max(b[0], b[1])
        ymin = min(b[2], b[3])
        ymax = max(b[2], b[3])

    # Validate against map limits to prevent "out of map range" errors
    if xmin < map_xmin or xmax > map_xmax or ymin < map_ymin or ymax > map_ymax:
        print(f"[SnakeFilter] Warning: Requested bounds X:[{xmin:.2f}, {xmax:.2f}], Y:[{ymin:.2f}, {ymax:.2f}] "
              f"extend beyond map boundaries X:[{map_xmin:.2f}, {map_xmax:.2f}], Y:[{map_ymin:.2f}, {map_ymax:.2f}]. Clamping to map range.")
        xmin = max(xmin, map_xmin + robot_radius)
        xmax = min(xmax, map_xmax - robot_radius)
        ymin = max(ymin, map_ymin + robot_radius)
        ymax = min(ymax, map_ymax - robot_radius)

    print(f"[SnakeFilter] Area bounds: X:[{xmin:.2f}, {xmax:.2f}], Y:[{ymin:.2f}, {ymax:.2f}]")

    dx = xmax - xmin
    dy = ymax - ymin

    if dx < 0.1 or dy < 0.1:
        print(f"[SnakeFilter] Error: Sweep area is completely out of map range or too small! (dx={dx:.2f}m, dy={dy:.2f}m)")
        return []

    # Auto-align sweep axis along the longer dimension (reduces turns drastically)
    sweep_axis = 'y' if dy > dx else 'x'
    sweep_span = dy if sweep_axis == 'x' else dx
    est_sweeps = max(1, int(math.ceil(sweep_span / max(0.05, row_gap)))) + 1
    print(f"[SnakeFilter] Auto-selected sweep axis: along {sweep_axis.upper()} (dx={dx:.2f}m, dy={dy:.2f}m, estimated {est_sweeps} sweeps)")

    def _linspace_unique(start: float, end: float, step: float) -> list:
        dist = abs(end - start)
        if dist < 1e-4:
            return [float(round(start, 3))]
        n_intervals = max(1, int(math.ceil(dist / max(0.05, step))))
        actual_step = (end - start) / n_intervals
        return [float(round(start + i * actual_step, 3)) for i in range(n_intervals + 1)]

    raw_rows = []
    if sweep_axis == 'x':
        y_coords = _linspace_unique(ymin, ymax, row_gap)
        for r_idx, y_val in enumerate(y_coords):
            x_line = _linspace_unique(xmin, xmax, step_x)
            if r_idx % 2 != 0:
                x_line.reverse()
            raw_rows.append([(x, y_val) for x in x_line])
    else:
        x_coords = _linspace_unique(xmin, xmax, row_gap)
        for r_idx, x_val in enumerate(x_coords):
            y_line = _linspace_unique(ymin, ymax, step_x)
            if r_idx % 2 != 0:
                y_line.reverse()
            raw_rows.append([(x_val, y) for y in y_line])

    # Obstacle grid safety filter with full robot radius collision check
    def _is_safe(px: float, py: float) -> bool:
        c_col = int(math.floor((px - origin_x) / res))
        c_row = int(math.floor(height - 1 - (py - origin_y) / res))
        if c_col < 0 or c_col >= width or c_row < 0 or c_row >= height:
            return False
        for dr in range(-radius_px, radius_px + 1):
            for dc in range(-radius_px, radius_px + 1):
                if dr * dr + dc * dc > radius_px * radius_px:
                    continue
                r_idx = c_row + dr
                c_idx = c_col + dc
                if r_idx < 0 or r_idx >= height or c_idx < 0 or c_idx >= width:
                    return False
                if grid[r_idx, c_idx] < 250:  # Must be strictly free (grid >= 250); unknown (~205) is not free
                    return False
        return True

    filtered_rows = []
    for row in raw_rows:
        safe_row = [pt for pt in row if _is_safe(pt[0], pt[1])]
        if safe_row:
            filtered_rows.append(safe_row)

    if not filtered_rows:
        print("[SnakeFilter] Error: No safe waypoints found in defined area! (All points hit obstacles or walls)")
        return []

    chosen_rows = filtered_rows

    # Build sequence with smooth U-turn transitions between rows
    pts = []
    for r_idx, row in enumerate(chosen_rows):
        if r_idx > 0 and len(pts) > 0 and len(row) > 0:
            prev_pt = pts[-1]
            next_pt = row[0]
            mid_x = float(round((prev_pt[0] + next_pt[0]) / 2.0, 3))
            mid_y = float(round((prev_pt[1] + next_pt[1]) / 2.0, 3))
            # Only insert intermediate midpoint if far enough and safe
            if math.hypot(next_pt[0] - prev_pt[0], next_pt[1] - prev_pt[1]) > 0.05 and _is_safe(mid_x, mid_y):
                pts.append((mid_x, mid_y))

        for pt in row:
            if pts and math.hypot(pt[0] - pts[-1][0], pt[1] - pts[-1][1]) < 0.02:
                continue
            pts.append(pt)

    # Compute continuous segment headings pointing along actual motion
    waypoints = []
    for i in range(len(pts)):
        x, y = pts[i]
        if i < len(pts) - 1:
            seg_dx = pts[i+1][0] - x
            seg_dy = pts[i+1][1] - y
            if abs(seg_dx) > 1e-4 or abs(seg_dy) > 1e-4:
                seg_yaw = math.atan2(seg_dy, seg_dx)
            else:
                seg_yaw = waypoints[-1]['yaw'] if waypoints else 0.0
        else:
            seg_yaw = waypoints[-1]['yaw'] if waypoints else 0.0

        seg_yaw = float(round(seg_yaw, 3))
        waypoints.append({
            'x': x,
            'y': y,
            'yaw': seg_yaw,
            'orientation': yaw_to_quaternion(seg_yaw)
        })

    num_sweeps = len(chosen_rows)
    if num_sweeps <= 2:
        print(f"\033[1;33m[SnakeFilter WARNING]\033[0m Only {num_sweeps} sweeps generated! Span across rows is only {sweep_span:.2f} m with row gap {row_gap:.2f} m.")
        print(f"  -> Tip: To cover more area, increase the bounding box span (e.g. Min Y to Max Y).")
    else:
        print(f"[SnakeFilter] Successfully generated {num_sweeps} sweeps ({len(waypoints)} waypoints) across {sweep_span:.2f} m span.")

    return waypoints


def map_pgm_to_png_bytes(map_yaml_file: str) -> bytes:
    """Convert map PGM/YAML to PNG binary bytes for web app rendering."""
    if not os.path.exists(map_yaml_file):
        return b''

    try:
        with open(map_yaml_file, 'r') as f:
            meta = yaml.safe_load(f)

        img_rel = meta['image']
        if not img_rel.startswith('/'):
            img_path = os.path.join(os.path.dirname(map_yaml_file), img_rel)
        else:
            img_path = img_rel

        if not os.path.exists(img_path):
            return b''

        img = Image.open(img_path).convert('L')
        buf = io.BytesIO()
        img.save(buf, format='PNG')
        return buf.getvalue()
    except Exception as e:
        print(f"[BossNode] Map PNG conversion error: {e}")
        return b''


class Cleaner:
    """
    Boss Orchestrator Class managing state transitions and ROS 2 process bringup/shutdown.
    """

    def __init__(self, map_path: str = None):
        self.state = S.IDLE
        if map_path is None:
            map_path = os.path.join(robot_home(), "my_map")
        self.map_prefix = map_path
        self.map_yaml = f"{map_path}.yaml"

        self.slam = None
        self.nav = None

        self.current_waypoint = 0
        self.total_waypoints = 0
        self.progress_pct = 0.0
        self.navigator = None
        self.lock = threading.Lock()

    def get_status_dict(self) -> dict:
        with self.lock:
            return {
                'state': self.state.name,
                'state_code': self.state.value,
                'current_waypoint': self.current_waypoint,
                'total_waypoints': self.total_waypoints,
                'progress_pct': round(self.progress_pct, 1)
            }

    def start_mapping(self) -> bool:
        with self.lock:
            print("[Cleaner] Starting MAPPING state...")
            self.stop_all_locked()

            self.slam = launch("ros2 launch motor_control slam_bringup.launch.py")
            self.state = S.MAPPING
            return True

    def save_map(self, map_name: str = None) -> bool:
        with self.lock:
            if map_name:
                self.map_prefix = map_name if map_name.startswith('/') else os.path.join(robot_home(), map_name)
                self.map_yaml = f"{self.map_prefix}.yaml"

            print(f"[Cleaner] Saving map to {self.map_prefix} and transitioning to MAP_SAVED...")
            install_setup = os.path.join(robot_home(), "ros2_ws/install/setup.bash")
            bash_script = (
                f'source /opt/ros/humble/setup.bash; '
                f'[ -f {shlex.quote(install_setup)} ] && source {shlex.quote(install_setup)}; '
                f'export ROS_LOG_DIR=/tmp; '
                f'exec ros2 run nav2_map_server map_saver_cli -f {shlex.quote(self.map_prefix)} --ros-args --remap map:=/map'
            )
            ret = -1
            try:
                res = subprocess.run(["bash", "-c", bash_script], timeout=30)
                ret = res.returncode
            except Exception as e:
                print(f"[Cleaner] Exception running map_saver_cli: {e}")

            if self.slam:
                stop(self.slam)
                self.slam = None

            kill_all_zombies()
            # Dropped dangerous rm -f *.posegraph *.data /tmp/*.posegraph /tmp/*.data cleanup

            if (ret == 0 and os.path.exists(self.map_yaml)) or (os.path.exists(self.map_yaml) and os.path.getmtime(self.map_yaml) > (time.time() - 35)):
                print(f"[Cleaner] Map saved successfully: {self.map_yaml}")
                self.state = S.MAP_SAVED
                return True
            else:
                print(f"[Cleaner] Map save failed! Returncode: {ret}")
                return False

    def start_sweep(self, bounds: list = None, row_gap: float = 0.4, step_x: float = 0.4, robot_radius: float = 0.48) -> bool:
        with self.lock:
            print("[Cleaner] Starting SWEEPING state...")
            self.stop_all_locked()

            if not os.path.exists(self.map_yaml):
                print(f"[Cleaner] Error: Map {self.map_yaml} does not exist!")
                return False

            cmd = f"ros2 launch motor_control nav2_bringup.launch.py map:={self.map_yaml}"
            self.nav = launch(cmd)
            self.state = S.SWEEPING

        threading.Thread(
            target=self._run_sweep,
            args=(bounds, row_gap, step_x, robot_radius),
            daemon=True
        ).start()
        return True

    def _run_sweep(self, bounds: list, row_gap: float, step_x: float, robot_radius: float):
        try:
            waypoints = make_snake(self.map_yaml, bounds, row_gap, step_x, robot_radius)
            with self.lock:
                self.total_waypoints = len(waypoints)
                self.current_waypoint = 0
                self.progress_pct = 0.0

            if not waypoints:
                print("[Cleaner] No valid waypoints to execute!")
                with self.lock:
                    self.state = S.DONE
                return

            print("[Cleaner] Waiting 6 seconds for Nav2 lifecycle nodes...")
            time.sleep(6.0)

            # Initialize BasicNavigator
            if not rclpy.ok():
                rclpy.init()

            self.navigator = BasicNavigator()

            # Load saved dock pose from locations.yaml (or default 0,0,0)
            dock_x, dock_y, dock_yaw = 0.0, 0.0, 0.0
            loc_candidates = [
                os.path.join(robot_home(), "ros2_ws/src/motor_control/config/locations.yaml"),
                os.path.join(robot_home(), "ros2_ws/install/motor_control/share/motor_control/config/locations.yaml")
            ]
            for lc in loc_candidates:
                if os.path.exists(lc):
                    try:
                        with open(lc, 'r') as f:
                            loc_data = yaml.safe_load(f)
                            if loc_data and 'dock' in loc_data:
                                dock_x = float(loc_data['dock'].get('x', 0.0))
                                dock_y = float(loc_data['dock'].get('y', 0.0))
                                dock_yaw = float(loc_data['dock'].get('yaw', 0.0))
                                break
                    except Exception:
                        pass

            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = dock_x
            initial_pose.pose.position.y = dock_y
            q_dock = yaw_to_quaternion(dock_yaw)
            initial_pose.pose.orientation.x = q_dock['x']
            initial_pose.pose.orientation.y = q_dock['y']
            initial_pose.pose.orientation.z = q_dock['z']
            initial_pose.pose.orientation.w = q_dock['w']

            print(f"[Cleaner] Setting initial pose to dock (X={dock_x:.2f}, Y={dock_y:.2f}, Yaw={dock_yaw:.2f})...")
            self.navigator.setInitialPose(initial_pose)

            print("[Cleaner] Waiting for Nav2 active...")
            self.navigator.waitUntilNav2Active()

            pose_list = []
            for wp in waypoints:
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.header.stamp = self.navigator.get_clock().now().to_msg()
                p.pose.position.x = wp['x']
                p.pose.position.y = wp['y']
                q = wp['orientation']
                p.pose.orientation.x = q['x']
                p.pose.orientation.y = q['y']
                p.pose.orientation.z = q['z']
                p.pose.orientation.w = q['w']
                pose_list.append(p)

            corrector = None
            try:
                from motor_control.pose_corrector import AutonomousPoseCorrector
                corrector = AutonomousPoseCorrector(node_name='cleaner_pose_corrector')
                print("[Cleaner] Autonomous 2D Pose Corrector active.")
                corrector.sharpen_pose(timeout_sec=0.1)
            except Exception as e:
                print(f"[Cleaner] Pose corrector warning: {e}")

            print(f"[Cleaner] Navigating through {len(pose_list)} snake waypoints...")
            self.navigator.followWaypoints(pose_list)

            last_wp_idx = -1
            while not self.navigator.isTaskComplete():
                with self.lock:
                    if self.state != S.SWEEPING:
                        print("[Cleaner] Sweep task cancelled.")
                        self.navigator.cancelTask()
                        break

                feedback = self.navigator.getFeedback()
                if feedback:
                    cw = getattr(feedback, 'current_waypoint', None)
                    if cw is not None:
                        with self.lock:
                            self.current_waypoint = cw + 1
                            if self.total_waypoints > 0:
                                self.progress_pct = (self.current_waypoint / self.total_waypoints) * 100.0

                        if corrector and cw != last_wp_idx:
                            last_wp_idx = cw
                            corrector.sharpen_pose(timeout_sec=0.1)

                time.sleep(0.5)

            res = self.navigator.getResult()
            if res == TaskResult.SUCCEEDED:
                print("[Cleaner] Sweep finished successfully! All waypoints reached.")
                # Autonomously navigate to Pit location
                loc_candidates = [
                    os.path.join(robot_home(), "ros2_ws/src/motor_control/config/locations.yaml"),
                    os.path.join(robot_home(), "ros2_ws/install/motor_control/share/motor_control/config/locations.yaml")
                ]
                pit_pose = None
                for lc in loc_candidates:
                    if os.path.exists(lc):
                        try:
                            with open(lc, 'r') as f:
                                d = yaml.safe_load(f)
                                if d and 'pit' in d:
                                    p_x = float(d['pit'].get('x', 0.0))
                                    p_y = float(d['pit'].get('y', 0.0))
                                    p_yaw = float(d['pit'].get('yaw', 0.0))
                                    p_msg = PoseStamped()
                                    p_msg.header.frame_id = 'map'
                                    p_msg.header.stamp = self.navigator.get_clock().now().to_msg()
                                    p_msg.pose.position.x = p_x
                                    p_msg.pose.position.y = p_y
                                    q = yaw_to_quaternion(p_yaw)
                                    p_msg.pose.orientation.x = q['x']
                                    p_msg.pose.orientation.y = q['y']
                                    p_msg.pose.orientation.z = q['z']
                                    p_msg.pose.orientation.w = q['w']
                                    pit_pose = p_msg
                                    break
                        except Exception:
                            pass
                if pit_pose:
                    print(f"[Cleaner] Autonomously navigating to Pit location (X={p_x:.2f}, Y={p_y:.2f})...")
                    self.navigator.goToPose(pit_pose)
                    while not self.navigator.isTaskComplete():
                        with self.lock:
                            if self.state != S.SWEEPING:
                                self.navigator.cancelTask()
                                break
                        time.sleep(0.5)
                    if self.navigator.getResult() == TaskResult.SUCCEEDED:
                        print("[Cleaner] Successfully reached Pit location after sweeping entire room!")
            else:
                print(f"[Cleaner] Sweep finished with result: {res}")

        except Exception as e:
            print(f"[Cleaner] Error in sweep execution: {e}")
        finally:
            with self.lock:
                self.state = S.DONE
                self.progress_pct = 100.0 if self.total_waypoints > 0 else 0.0

    def e_stop(self):
        with self.lock:
            print("[Cleaner] E-STOP TRIGGERED! Halting navigation task without killing processes.")
            if self.navigator:
                try:
                    self.navigator.cancelTask()
                except Exception:
                    pass
            self.state = S.IDLE

    def stop_all_locked(self):
        if self.slam:
            stop(self.slam)
            self.slam = None
        if self.nav:
            stop(self.nav)
            self.nav = None
        kill_all_zombies()


def main():
    print("==================================================")
    print("      FARM CLEANER BOSS ORCHESTRATOR NODE         ")
    print("==================================================")
    cleaner = Cleaner()
    print(f"Cleaner initialized. Initial State: {cleaner.state.name}")


if __name__ == '__main__':
    main()
