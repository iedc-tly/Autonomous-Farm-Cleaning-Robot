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
from enum import Enum
import yaml
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class S(Enum):
    IDLE = 0
    MAPPING = 1
    MAP_SAVED = 2
    SWEEPING = 3
    DONE = 4


def launch(cmd: str) -> subprocess.Popen:
    """Launch process in its own session group for clean kill later."""
    print(f"[BossNode] Launching process group: {cmd}")
    return subprocess.Popen(cmd, shell=True, start_new_session=True)


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
    """Kill any zombie nodes that might conflict over serial ports or TFs."""
    targets = [
        "async_slam_toolbox_node",
        "slam_toolbox",
        "nav2",
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
        "rviz2"
    ]
    for target in targets:
        subprocess.run(f"pkill -9 -f {target}", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(1.0)


def yaw_to_quaternion(yaw: float) -> dict:
    """Convert yaw angle (radians) to quaternion dict."""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': float(round(math.sin(yaw / 2.0), 4)),
        'w': float(round(math.cos(yaw / 2.0), 4))
    }


def make_snake(map_yaml_file: str, bounds: list = None, row_gap: float = 0.4, step_x: float = 0.4, robot_radius: float = 0.3) -> list:
    """
    Generate snake (lawnmower) waypoints filtered by map occupancy and robot radius.

    :param map_yaml_file: Full path to map yaml file
    :param bounds: Optional [xmin, xmax, ymin, ymax] bounding box in meters
    :param row_gap: Distance between snake rows (meters)
    :param step_x: Step size along rows (meters)
    :param robot_radius: Safety clearance radius around robot (meters)
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

    radius_px = int(math.ceil(robot_radius / res))

    # Free cells threshold: in ROS maps, >= 250 represents free space
    if bounds is None or len(bounds) != 4:
        free_rows, free_cols = np.where(grid >= 250)
        if len(free_rows) == 0:
            print("[SnakeFilter] Warning: No free pixels found in map!")
            return []
        xmin = origin_x + free_cols.min() * res + robot_radius
        xmax = origin_x + free_cols.max() * res - robot_radius
        ymin = origin_y + (height - 1 - free_rows.max()) * res + robot_radius
        ymax = origin_y + (height - 1 - free_rows.min()) * res - robot_radius
    else:
        xmin, xmax, ymin, ymax = [float(b) for b in bounds]

    print(f"[SnakeFilter] Area bounds: X:[{xmin:.2f}, {xmax:.2f}], Y:[{ymin:.2f}, {ymax:.2f}]")

    if xmin >= xmax or ymin >= ymax:
        print("[SnakeFilter] Error: Invalid bounds coordinates!")
        return []

    num_rows = max(1, int(math.ceil((ymax - ymin) / row_gap)) + 1)
    waypoints = []

    for r in range(num_rows):
        cur_y = min(ymin + r * row_gap, ymax)
        num_cols = max(1, int(math.ceil((xmax - xmin) / step_x)) + 1)
        x_coords = [min(xmin + c * step_x, xmax) for c in range(num_cols)]

        if r % 2 == 0:
            yaw = 0.0
        else:
            x_coords.reverse()
            yaw = math.pi

        for x in x_coords:
            c_col = int(math.floor((x - origin_x) / res))
            c_row = int(math.floor(height - 1 - (cur_y - origin_y) / res))

            is_safe = True
            for dr in range(-radius_px, radius_px + 1):
                for dc in range(-radius_px, radius_px + 1):
                    if dr * dr + dc * dc > radius_px * radius_px:
                        continue
                    r_idx = c_row + dr
                    c_idx = c_col + dc
                    if r_idx < 0 or r_idx >= height or c_idx < 0 or c_idx >= width:
                        is_safe = False
                        break
                    if grid[r_idx, c_idx] < 250:  # Occupied or unknown cell
                        is_safe = False
                        break
                if not is_safe:
                    break

            if is_safe:
                q = yaw_to_quaternion(yaw)
                waypoints.append({
                    'x': float(round(x, 3)),
                    'y': float(round(cur_y, 3)),
                    'yaw': float(round(yaw, 3)),
                    'orientation': q
                })

    print(f"[SnakeFilter] Filtered snake pattern: {len(waypoints)} valid waypoints created.")
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

    def __init__(self, map_path: str = "/home/iedctly/my_map"):
        self.state = S.IDLE
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

    def save_map(self) -> bool:
        with self.lock:
            print("[Cleaner] Saving map and transitioning to MAP_SAVED...")
            cmd = f"ros2 run nav2_map_server map_saver_cli -f {self.map_prefix}"
            res = subprocess.run(cmd, shell=True)

            if self.slam:
                stop(self.slam)
                self.slam = None

            kill_all_zombies()
            subprocess.run("rm -f *.posegraph *.data /tmp/*.posegraph /tmp/*.data", shell=True)

            if res.returncode == 0 and os.path.exists(self.map_yaml):
                print(f"[Cleaner] Map saved successfully: {self.map_yaml}")
                self.state = S.MAP_SAVED
                return True
            else:
                print(f"[Cleaner] Map save failed!")
                return False

    def start_sweep(self, bounds: list = None, row_gap: float = 0.4, step_x: float = 0.4, robot_radius: float = 0.3) -> bool:
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

            # Set initial pose = dock (0, 0, 0)
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.orientation.w = 1.0

            print("[Cleaner] Setting initial pose to dock (0, 0, 0)...")
            self.navigator.setInitialPose(initial_pose)

            print("[Cleaner] Waiting for Nav2 active...")
            self.navigator.waitUntilNav2Active(timeout_sec=20.0)

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

            print(f"[Cleaner] Navigating through {len(pose_list)} snake waypoints...")
            self.navigator.followWaypoints(pose_list)

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

                time.sleep(0.5)

            res = self.navigator.getResult()
            if res == TaskResult.SUCCEEDED:
                print("[Cleaner] Sweep finished successfully!")
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
            print("[Cleaner] E-STOP TRIGGERED!")
            if self.navigator:
                try:
                    self.navigator.cancelTask()
                except Exception:
                    pass
            self.stop_all_locked()
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
