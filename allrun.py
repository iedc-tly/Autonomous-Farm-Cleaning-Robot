#!/usr/bin/env python3
"""
allrun.py - Autonomous Farm Cleaning Robot Master All-In-One Mission Pipeline
=============================================================================
Sequential Pipeline:
  Step 1: Mapping choice:
          [1] Map new area with SLAM + Teleop -> Save map with [Ctrl+M] or [M]
          [2] Skip mapping -> Automatically load latest saved map
  Step 2: Launch Nav2 Navigation Stack & Localization
  Step 3: Interactive Point & Bounds Capture (WASD Teleop + Hotkeys [1-4]):
          - [1] Dock
          - [2] Pit
          - [3] Cleaning Square Bottom-Left (BL)
          - [4] Cleaning Square Top-Right (TR)
          - [Enter] Confirm & Auto-generate Snake Pattern Waypoints
  Step 4: Launch App Bridge (:8000) & ROSBridge WebSocket (:9090)
  Step 5: Standby: Execute Snake Pattern on CLI command ([G]/[S]) or App command
          with SINGLE-SIDE MOTOR TURNS (smooth ground pivot, no one-point turn)
"""

import os
import sys
import glob
import math
import time
import signal
import select
import termios
import tty
import yaml
import subprocess
import threading
import argparse
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Workspace paths
WS_DIR = "/home/iedctly/ros2_ws"
MAP_PREFIX = "/home/iedctly/my_map"
MAP_FILE = f"{MAP_PREFIX}.yaml"
MAP_PGM = f"{MAP_PREFIX}.pgm"
LOCATIONS_FILE = os.path.join(WS_DIR, "src/motor_control/config/locations.yaml")
SNAKE_WAYPOINTS_FILE = os.path.join(WS_DIR, "snake_waypoints.yaml")
MAPPING_RVIZ = os.path.join(WS_DIR, "src/motor_control/config/mapping.rviz")
BRIDGE_SCRIPT = os.path.join(WS_DIR, "src/rover_app_bridge/rover_app_bridge/rover_bridge_node.py")

# Imports from motor_control
try:
    from motor_control.snake_pattern_navigator import generate_snake_pattern
except ImportError:
    generate_snake_pattern = None

try:
    from motor_control.farm_cleaner import make_snake
except ImportError:
    make_snake = None

try:
    from motor_control.pose_corrector import AutonomousPoseCorrector
except ImportError:
    AutonomousPoseCorrector = None

ACTIVE_PROCESSES = []


def launch_proc(cmd: str, name: str = "", log_file: str = None) -> subprocess.Popen:
    """Launch a command in its own process group for clean signal handling."""
    if log_file:
        out = open(log_file, "w")
    else:
        out = subprocess.DEVNULL
    p = subprocess.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setsid,
        stdout=out,
        stderr=subprocess.STDOUT
    )
    ACTIVE_PROCESSES.append((p, name))
    return p


def kill_proc(p: subprocess.Popen, timeout: float = 3.0):
    """Terminate process group cleanly with SIGINT, falling back to SIGKILL."""
    if p is None:
        return
    try:
        pgid = os.getpgid(p.pid)
        os.killpg(pgid, signal.SIGINT)
        start_t = time.time()
        while time.time() - start_t < timeout:
            if p.poll() is not None:
                return
            time.sleep(0.2)
        os.killpg(pgid, signal.SIGKILL)
        p.wait()
    except Exception:
        pass


def kill_all_zombies(clean_shm: bool = False):
    """Clean up leftover ROS 2, slam, rplidar, and nav2 processes."""
    kill_names = [
        "slam_toolbox", "sync_slam_toolbox_node", "async_slam_toolbox_node",
        "rplidar_node", "rplidar_composition", "motor_node", "encoder_odom_node",
        "mpu9150_node", "bt_navigator", "controller_server", "planner_server",
        "recoveries_server", "behavior_server", "waypoint_follower",
        "robot_state_publisher", "rosbridge_websocket"
    ]
    for name in kill_names:
        subprocess.run(f"pkill -9 -f {name} >/dev/null 2>&1", shell=True)

    if clean_shm:
        subprocess.run("rm -rf /dev/shm/fastrtps_* >/dev/null 2>&1", shell=True)


def ensure_pigpiod():
    """Verify that pigpio daemon is active for hardware motor PWM."""
    res = subprocess.run("pgrep -x pigpiod >/dev/null 2>&1", shell=True)
    if res.returncode != 0:
        print("\033[1;36m[Hardware]\033[0m Starting pigpio hardware daemon...")
        subprocess.run("sudo systemctl start pigpiod 2>/dev/null || sudo pigpiod 2>/dev/null", shell=True)
        time.sleep(0.5)


def get_key(orig_settings, timeout: float = 0.05) -> str:
    """Read a single keypress non-blockingly."""
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                rest = sys.stdin.read(2)
                return ch + rest
            return ch
        return ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)


def find_latest_saved_map():
    """Search for valid ROS map YAML files and return latest."""
    search_dirs = ["/home/iedctly", WS_DIR]
    candidates = []
    seen = set()
    for d in search_dirs:
        for ypath in glob.glob(os.path.join(d, "*.yaml")):
            ypath_real = os.path.realpath(ypath)
            if ypath_real in seen:
                continue
            seen.add(ypath_real)
            try:
                with open(ypath, 'r') as f:
                    data = yaml.safe_load(f)
                    if isinstance(data, dict) and 'image' in data and 'resolution' in data:
                        img_path = data['image']
                        if not os.path.isabs(img_path):
                            img_path = os.path.join(os.path.dirname(ypath), img_path)
                        if os.path.exists(img_path):
                            mtime = os.path.getmtime(ypath)
                            candidates.append((mtime, ypath))
            except Exception:
                pass

    if candidates:
        candidates.sort(key=lambda x: x[0], reverse=True)
        latest_time, latest_path = candidates[0]
        dt_str = datetime.fromtimestamp(latest_time).strftime('%Y-%m-%d %H:%M:%S')
        return latest_path, dt_str

    if os.path.exists(MAP_FILE):
        mtime = os.path.getmtime(MAP_FILE)
        dt_str = datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')
        return MAP_FILE, dt_str

    return None, None


def yaw_to_quaternion(yaw: float) -> dict:
    return {
        'x': 0.0,
        'y': 0.0,
        'z': float(round(math.sin(yaw / 2.0), 4)),
        'w': float(round(math.cos(yaw / 2.0), 4))
    }


def yaw_from_quaternion(q) -> float:
    if hasattr(q, 'z') and hasattr(q, 'w'):
        qz, qw = q.z, q.w
        qx = getattr(q, 'x', 0.0)
        qy = getattr(q, 'y', 0.0)
    else:
        qx, qy, qz, qw = q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0), q.get('w', 1.0)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PipelineSupervisorNode(Node):
    """Supervisor node for teleop, TF pose capture, and path visualization."""

    def __init__(self):
        super().__init__('allrun_supervisor_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/snake_path', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.turn_mode_pub = self.create_publisher(Bool, '/single_side_turn', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Teleop speeds
        self.linear_speed = 0.05
        self.angular_speed = 0.15
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.teleop_active = False
        self.last_teleop_time = 0.0

        # Turn mode: False = Point Turn (zero-radius spin), True = Single-Side Pivot
        self.single_side_turn = False

        # RViz pose capture
        self.rviz_goal_pose = None
        self.last_rviz_click_time = 0.0
        self.rviz_sub = self.create_subscription(PoseStamped, '/goal_pose', self._on_rviz_goal, 10)

        # 20 Hz teleop publish timer
        self.teleop_timer = None

    def set_turn_mode(self, single_side: bool):
        """Set robot turn mode: True=Single-Side Pivot, False=Point Turn (zero-radius spin)."""
        self.single_side_turn = bool(single_side)
        msg = Bool()
        msg.data = self.single_side_turn
        for _ in range(3):
            self.turn_mode_pub.publish(msg)
            time.sleep(0.01)
        mode_str = "\033[1;32m[SINGLE-SIDE PIVOT ⮵ (ONE-SIDE MOTORS)]\033[0m" if self.single_side_turn else "\033[1;33m[TANK TURN ⟲ (DUAL-SIDE ZERO RADIUS)]\033[0m"
        print(f"\n[TURN MODE] Switched to: {mode_str}")

    def toggle_turn_mode(self):
        """Toggle between single-side pivot and point turn."""
        self.set_turn_mode(not self.single_side_turn)

    def adjust_linear_speed(self, delta: float):
        """Adjust linear speed live and sync with Nav2 / motor_node."""
        self.linear_speed = round(max(0.01, min(0.80, self.linear_speed + delta)), 2)
        print(f"\n\033[1;36m[SPEED ADJUSTED]\033[0m Linear: {self.linear_speed:.2f} m/s | Turn: {self.angular_speed:.2f} rad/s")
        self.sync_nav2_speed()

    def adjust_angular_speed(self, delta: float):
        """Adjust angular turn rate live and sync with Nav2 / motor_node."""
        self.angular_speed = round(max(0.02, min(1.50, self.angular_speed + delta)), 2)
        print(f"\n\033[1;36m[SPEED ADJUSTED]\033[0m Linear: {self.linear_speed:.2f} m/s | Turn: {self.angular_speed:.2f} rad/s")
        self.sync_nav2_speed()

    def sync_nav2_speed(self):
        """Asynchronously push speed parameters to Nav2 and motor_node."""
        def _worker(lin, ang):
            min_approach = round(min(0.02, lin * 0.5), 3)
            subprocess.run(f"ros2 param set /motor_node max_linear {max(0.40, lin + 0.05)} >/dev/null 2>&1", shell=True)
            subprocess.run(f"ros2 param set /motor_node max_angular {max(0.60, ang + 0.10)} >/dev/null 2>&1", shell=True)
            subprocess.run(f"ros2 param set /controller_server FollowPath.desired_linear_vel {lin} >/dev/null 2>&1", shell=True)
            subprocess.run(f"ros2 param set /controller_server FollowPath.rotate_to_heading_angular_vel {ang} >/dev/null 2>&1", shell=True)
            subprocess.run(f"ros2 param set /controller_server FollowPath.min_approach_linear_velocity {min_approach} >/dev/null 2>&1", shell=True)
            subprocess.run(f"ros2 param set /controller_server FollowPath.regulated_linear_scaling_min_speed {min_approach} >/dev/null 2>&1", shell=True)
            subprocess.run(f"ros2 param set /velocity_smoother max_velocity '[{lin}, 0.0, {ang}]' >/dev/null 2>&1", shell=True)
            subprocess.run(f"ros2 param set /velocity_smoother min_velocity '[{-lin}, 0.0, {-ang}]' >/dev/null 2>&1", shell=True)
        threading.Thread(target=_worker, args=(self.linear_speed, self.angular_speed), daemon=True).start()

    def start_teleop_publisher(self):
        if self.teleop_timer is None:
            self.teleop_timer = self.create_timer(0.05, self._publish_teleop_twist)

    def stop_teleop_publisher(self):
        if self.teleop_timer is not None:
            self.teleop_timer.cancel()
            self.teleop_timer = None
        self.stop_robot()

    def set_motion(self, lin_factor: float, ang_factor: float):
        self.target_linear = lin_factor * self.linear_speed
        self.target_angular = ang_factor * self.angular_speed
        self.teleop_active = True
        self.last_teleop_time = time.time()

    def stop_robot(self):
        if self.teleop_active:
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.teleop_active = False
            # Send single zero twist to halt, then yield /cmd_vel cleanly to Nav2
            msg = Twist()
            self.cmd_pub.publish(msg)

    def _publish_teleop_twist(self):
        # CRITICAL FIX: Only publish when teleoperation is actively commanded by user!
        # Do NOT publish zeros continuously. Continuous zeros collide with Nav2's controller_server
        # and cause severe jitter/stuttering when navigating to 2D Goal Pose or waypoints.
        if self.teleop_active:
            if time.time() - self.last_teleop_time > 0.4:
                # Key released (timeout) -> stop and release /cmd_vel cleanly
                self.stop_robot()
            else:
                msg = Twist()
                msg.linear.x = float(self.target_linear)
                msg.angular.z = float(self.target_angular)
                self.cmd_pub.publish(msg)

    def _on_rviz_goal(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self.rviz_goal_pose = {
            'x': float(round(x, 3)),
            'y': float(round(y, 3)),
            'yaw': float(round(yaw, 3))
        }
        self.last_rviz_click_time = time.time()
        print(f"\n\033[1;36m[RVIZ 2D GOAL POSE CLICKED]\033[0m X={x:.3f}, Y={y:.3f}, Yaw={math.degrees(yaw):.1f}°")

    def get_robot_pose_tf(self, timeout_sec: float = 0.5):
        """Get live robot pose from map -> base_link TF."""
        start_t = time.time()
        while time.time() - start_t < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                t = trans.transform.translation
                q = trans.transform.rotation
                yaw = yaw_from_quaternion(q)
                return {
                    'x': float(round(t.x, 3)),
                    'y': float(round(t.y, 3)),
                    'yaw': float(round(yaw, 3))
                }
            except Exception:
                pass
        return None

    def publish_initial_pose(self, pose: dict):
        """Seed AMCL with initial pose estimate."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(pose.get('x', 0.0))
        msg.pose.pose.position.y = float(pose.get('y', 0.0))
        msg.pose.pose.position.z = 0.0
        q = yaw_to_quaternion(float(pose.get('yaw', 0.0)))
        msg.pose.pose.orientation.x = q['x']
        msg.pose.pose.orientation.y = q['y']
        msg.pose.pose.orientation.z = q['z']
        msg.pose.pose.orientation.w = q['w']
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06
        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.05)


def check_lifecycle_nodes(node: Node, timeout_sec: float = 25.0) -> bool:
    """Verify that key Nav2 lifecycle nodes have reached the active state."""
    check_nodes = ['amcl', 'controller_server', 'planner_server', 'bt_navigator']
    start_time = time.time()
    for n_name in check_nodes:
        client = node.create_client(GetState, f'{n_name}/get_state')
        ready = False
        while (time.time() - start_time) < timeout_sec:
            if client.wait_for_service(timeout_sec=0.5):
                ready = True
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        if not ready:
            return False
    return True


def save_locations(dock: dict, pit: dict, sweep: dict, filepath: str = LOCATIONS_FILE):
    """Save dock, pit, and sweep bounds to locations.yaml."""
    data = {
        'dock': {
            'x': float(round(dock['x'], 3)),
            'y': float(round(dock['y'], 3)),
            'yaw': float(round(dock['yaw'], 3))
        },
        'pit': {
            'x': float(round(pit['x'], 3)),
            'y': float(round(pit['y'], 3)),
            'yaw': float(round(pit['yaw'], 3))
        },
        'sweep': {
            'xmin': float(round(sweep['xmin'], 3)),
            'xmax': float(round(sweep['xmax'], 3)),
            'ymin': float(round(sweep['ymin'], 3)),
            'ymax': float(round(sweep['ymax'], 3)),
            'row_gap': float(round(sweep.get('row_gap', 0.4), 3)),
            'robot_radius': float(round(sweep.get('robot_radius', 0.3), 3))
        }
    }
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    with open(filepath, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"\n\033[1;32m[SAVED]\033[0m Locations saved to {filepath}")
    return data


def generate_and_save_snake(sweep_bounds: dict, map_file: str = MAP_FILE, path_publisher=None):
    """
    Generate snake pattern with auto-axis alignment, smooth U-turn transitions,
    and save to snake_waypoints.yaml.
    """
    xmin = min(sweep_bounds['xmin'], sweep_bounds['xmax'])
    xmax = max(sweep_bounds['xmin'], sweep_bounds['xmax'])
    ymin = min(sweep_bounds['ymin'], sweep_bounds['ymax'])
    ymax = max(sweep_bounds['ymin'], sweep_bounds['ymax'])
    row_gap = sweep_bounds.get('row_gap', 0.4)
    robot_radius = sweep_bounds.get('robot_radius', 0.3)

    waypoints = []
    # 1. Try obstacle-filtered generator if map is present
    if make_snake and os.path.exists(map_file):
        try:
            print(f"\033[1;36m[SNAKE GENERATOR]\033[0m Generating obstacle-filtered snake pattern ({map_file})...")
            waypoints = make_snake(map_file, [xmin, xmax, ymin, ymax], row_gap=row_gap, step_x=0.4, robot_radius=robot_radius)
        except Exception as e:
            print(f"\033[1;33m[WARNING]\033[0m Filter warning: {e}. Falling back to geometric generator.")

    # 2. Geometric generator fallback
    if not waypoints:
        if generate_snake_pattern:
            waypoints = generate_snake_pattern(xmin, xmax, ymin, ymax, step_x=0.4, step_y=row_gap, axis='auto')
        else:
            dx = xmax - xmin
            dy = ymax - ymin
            axis = 'y' if dy > dx else 'x'
            pts = []
            if axis == 'y':
                num_cols = max(1, int(math.ceil(dx / row_gap)))
                step_c = dx / num_cols
                x_vals = [xmin + c * step_c for c in range(num_cols + 1)]
                for r_idx, x in enumerate(x_vals):
                    num_rows = max(1, int(math.ceil(dy / 0.4)))
                    step_r = dy / num_rows
                    y_vals = [ymin + r * step_r for r in range(num_rows + 1)]
                    if r_idx % 2 != 0:
                        y_vals.reverse()
                    for y in y_vals:
                        pts.append((x, y))
            else:
                num_rows = max(1, int(math.ceil(dy / row_gap)))
                step_r = dy / num_rows
                y_vals = [ymin + r * step_r for r in range(num_rows + 1)]
                for r_idx, y in enumerate(y_vals):
                    num_cols = max(1, int(math.ceil(dx / 0.4)))
                    step_c = dx / num_cols
                    x_vals = [xmin + c * step_c for c in range(num_cols + 1)]
                    if r_idx % 2 != 0:
                        x_vals.reverse()
                    for x in x_vals:
                        pts.append((x, y))

            for i, (x, y) in enumerate(pts):
                if i < len(pts) - 1:
                    yaw = math.atan2(pts[i+1][1] - y, pts[i+1][0] - x)
                else:
                    yaw = waypoints[-1]['yaw'] if waypoints else 0.0
                waypoints.append({
                    'x': float(round(x, 3)),
                    'y': float(round(y, 3)),
                    'yaw': float(round(yaw, 3)),
                    'orientation': yaw_to_quaternion(yaw)
                })

    with open(SNAKE_WAYPOINTS_FILE, 'w') as f:
        yaml.dump({'num_waypoints': len(waypoints), 'waypoints': waypoints}, f, default_flow_style=False)

    print(f"\033[1;32m[SUCCESS]\033[0m Generated {len(waypoints)} snake pattern waypoints -> {SNAKE_WAYPOINTS_FILE}")

    # Publish to RViz path visualizer
    if path_publisher:
        try:
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            for wp in waypoints:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = float(wp['x'])
                ps.pose.position.y = float(wp['y'])
                q = wp['orientation']
                ps.pose.orientation.x = float(q['x'])
                ps.pose.orientation.y = float(q['y'])
                ps.pose.orientation.z = float(q['z'])
                ps.pose.orientation.w = float(q['w'])
                path_msg.poses.append(ps)
            path_publisher.publish(path_msg)
            print("\033[1;36m[RVIZ VISUALIZATION]\033[0m Published snake pattern to /snake_path")
        except Exception:
            pass

    return waypoints


def run_snake_navigation(node: PipelineSupervisorNode, snake_waypoints: list, dock_pose: dict, orig_settings=None):
    """Execute autonomous snake pattern navigation via Nav2 BasicNavigator with single-side motor turning."""
    if not snake_waypoints:
        print("\033[1;31m[ERROR]\033[0m No snake waypoints to execute!")
        return

    node.stop_teleop_publisher()
    # Default to tank turn (dual-side motors in opposite directions) for lawnmower sweep!
    node.set_turn_mode(False)
    node.sync_nav2_speed()

    print("\n\033[1;34m" + "=" * 70 + "\033[0m")
    print(f"\033[1;32m   EXECUTING AUTONOMOUS SNAKE PATTERN ({len(snake_waypoints)} waypoints)   \033[0m")
    print(f"\033[1;33m   [TANK TURN MOTOR TURNING]\033[0m Turning with dual-side motors in opposite directions")
    print(f"   [LIVE CONTROLS]  [+ / -] : Adjust Speed live | [T] : Toggle Turn Mode | [Space/X] : Halt")
    print("\033[1;34m" + "=" * 70 + "\033[0m")

    navigator = BasicNavigator()

    corrector = None
    if AutonomousPoseCorrector:
        try:
            corrector = AutonomousPoseCorrector(node_name='allrun_pose_corrector')
            corrector.sharpen_pose(timeout_sec=0.1)
        except Exception:
            pass

    pose_list = []
    for wp in snake_waypoints:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = navigator.get_clock().now().to_msg()
        p.pose.position.x = float(wp['x'])
        p.pose.position.y = float(wp['y'])
        q = wp.get('orientation', yaw_to_quaternion(wp.get('yaw', 0.0)))
        p.pose.orientation.x = float(q['x'])
        p.pose.orientation.y = float(q['y'])
        p.pose.orientation.z = float(q['z'])
        p.pose.orientation.w = float(q['w'])
        pose_list.append(p)

    navigator.followWaypoints(pose_list)
    print("Nav2 waypoint navigation active. Press [Space] or [Ctrl+C] to abort.\n")

    last_idx = -1
    loop_cnt = 0
    try:
        while not navigator.isTaskComplete():
            loop_cnt += 1
            if orig_settings:
                key = get_key(orig_settings, timeout=0.02)
                if key in ('+', '='):
                    node.adjust_linear_speed(0.05)
                elif key in ('-', '_'):
                    node.adjust_linear_speed(-0.05)
                elif key in (']', '}'):
                    node.adjust_angular_speed(0.05)
                elif key in ('[', '{'):
                    node.adjust_angular_speed(-0.05)
                elif key in ('t', 'T'):
                    node.toggle_turn_mode()
                elif key in (' ', 'x', 'X', '\x03'):
                    print("\n[STOPPED] Snake pattern cancelled by user. Halting robot...")
                    try:
                        navigator.cancelTask()
                    except Exception:
                        pass
                    node.stop_robot()
                    break

            feedback = navigator.getFeedback()
            if feedback and loop_cnt % 5 == 0:
                cur_wp = getattr(feedback, 'current_waypoint', 0)
                rem_dist = getattr(feedback, 'distance_remaining', 0.0)
                turn_mode_lbl = "SINGLE ⮵" if node.single_side_turn else "TANK ⟲"
                print(f" -> WP {cur_wp + 1}/{len(snake_waypoints)} | Rem: {rem_dist:.2f}m | Lin: {node.linear_speed:.2f}m/s | [{turn_mode_lbl}] ([+-]Spd, [T]Turn, [Space]Halt)")

                if corrector and cur_wp != last_idx:
                    last_idx = cur_wp
                    corrector.sharpen_pose(timeout_sec=0.05)

            time.sleep(0.1)

        res = navigator.getResult()
        if res == TaskResult.SUCCEEDED:
            print("\n\033[1;32m" + "=" * 70 + "\033[0m")
            print("\033[1;32m   [MISSION COMPLETE] Successfully completed all snake pattern waypoints!   \033[0m")
            print("\033[1;32m" + "=" * 70 + "\033[0m")
        elif res == TaskResult.CANCELED:
            print("\n\033[1;33m[NOTICE]\033[0m Snake pattern navigation was canceled.")
        else:
            print(f"\n\033[1;31m[FAILED]\033[0m Navigation finished with result: {res}")
    except KeyboardInterrupt:
        print("\n[CANCELLED] User cancelled navigation. Halting robot...")
        try:
            navigator.cancelTask()
        except Exception:
            pass
        node.stop_robot()
    finally:
        node.start_teleop_publisher()


def navigate_to_location(node: PipelineSupervisorNode, target_pose: dict, name: str, orig_settings=None):
    """Navigate to Dock or Pit via Nav2."""
    if not target_pose:
        print(f"\n\033[1;31m[ERROR]\033[0m No {name} coordinates available.")
        return

    node.stop_teleop_publisher()
    node.sync_nav2_speed()
    print(f"\n\033[1;34m[NAVIGATING TO {name}]\033[0m Target: X={target_pose.get('x', 0):.2f}, Y={target_pose.get('y', 0):.2f}, Yaw={math.degrees(target_pose.get('yaw', 0.0)):.1f}°")
    print(f"   [LIVE CONTROLS]  [+ / -] : Adjust Speed live | [T] : Toggle Turn Mode | [Space/X] : Halt")
    navigator = BasicNavigator()

    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = float(target_pose.get('x', 0.0))
    p.pose.position.y = float(target_pose.get('y', 0.0))
    q = yaw_to_quaternion(float(target_pose.get('yaw', 0.0)))
    p.pose.orientation.x = float(q['x'])
    p.pose.orientation.y = float(q['y'])
    p.pose.orientation.z = float(q['z'])
    p.pose.orientation.w = float(q['w'])

    navigator.goToPose(p)

    try:
        while not navigator.isTaskComplete():
            if orig_settings:
                key = get_key(orig_settings, timeout=0.02)
                if key in ('+', '='):
                    node.adjust_linear_speed(0.05)
                elif key in ('-', '_'):
                    node.adjust_linear_speed(-0.05)
                elif key in (']', '}'):
                    node.adjust_angular_speed(0.05)
                elif key in ('[', '{'):
                    node.adjust_angular_speed(-0.05)
                elif key in ('t', 'T'):
                    node.toggle_turn_mode()
                elif key in (' ', 'x', 'X', '\x03'):
                    print(f"\n[STOPPED] Navigation to {name} canceled by user.")
                    try:
                        navigator.cancelTask()
                    except Exception:
                        pass
                    node.stop_robot()
                    break

            feedback = navigator.getFeedback()
            if feedback:
                rem_dist = getattr(feedback, 'distance_remaining', 0.0)
                turn_mode_lbl = "SINGLE ⮵" if node.single_side_turn else "TANK ⟲"
                sys.stdout.write(f"\r -> To {name} | Rem: {rem_dist:.2f}m | Lin: {node.linear_speed:.2f}m/s | [{turn_mode_lbl}] ([+-]Spd, [T]Turn, [Space]Halt) ")
                sys.stdout.flush()
            time.sleep(0.15)

        res = navigator.getResult()
        if res == TaskResult.SUCCEEDED:
            print(f"\n\033[1;32m[ARRIVED]\033[0m Successfully reached {name}!")
        elif res == TaskResult.CANCELED:
            print(f"\n\033[1;33m[CANCELED]\033[0m Navigation to {name} was canceled.")
        else:
            print(f"\n\033[1;31m[FAILED]\033[0m Navigation to {name} finished with result: {res}")
    except KeyboardInterrupt:
        try:
            navigator.cancelTask()
        except Exception:
            pass
        node.stop_robot()
    finally:
        node.start_teleop_publisher()


def run_pipeline(node: PipelineSupervisorNode, orig_settings, force_map: bool = None):
    """All-in-one sequential runner."""
    ensure_pigpiod()
    kill_all_zombies(clean_shm=False)

    slam_proc = None
    rviz_proc = None
    nav2_proc = None
    bridge_proc = None
    rosbridge_proc = None

    latest_map, map_mtime = find_latest_saved_map()
    active_map_file = MAP_FILE

    # ==========================================================================
    # STEP 1: MAPPING SELECTION (MAP OR SKIP MAP)
    # ==========================================================================
    print("\n\033[1;34m" + "=" * 72 + "\033[0m")
    print("\033[1;32m      AUTONOMOUS FARM CLEANING ROBOT - ALLRUN MASTER PIPELINE       \033[0m")
    print("\033[1;34m" + "=" * 72 + "\033[0m")

    if force_map is not None:
        do_mapping = force_map
    elif latest_map and os.path.exists(latest_map):
        print(f"\033[1;36m[SAVED MAP FOUND]\033[0m {latest_map}  (Modified: {map_mtime})")
        print("\nChoose an option:")
        print("  \033[1;32m[1]\033[0m Use Saved Map -> Skip SLAM (Proceed directly to Navigation & Snake)")
        print("  \033[1;33m[2]\033[0m Map New Area with SLAM & Teleop (Save map with Ctrl+M or Enter)")
        print("\033[1;34m" + "-" * 72 + "\033[0m")
        choice = input("Select [1-2] (default 1): ").strip() or "1"
        do_mapping = (choice == "2")
        if not do_mapping:
            active_map_file = latest_map
    else:
        print("\033[1;33m[NOTICE]\033[0m No saved map found. Starting with SLAM Mapping first.")
        do_mapping = True

    try:
        # ======================================================================
        # STEP 1.1: SLAM MAPPING + TELEOP (IF SELECTED)
        # ======================================================================
        if do_mapping:
            print("\n\033[1;34m" + "=" * 72 + "\033[0m")
            print("\033[1;32m   STEP 1: SLAM MAPPING & TELEOP BRINGUP   \033[0m")
            print("\033[1;34m" + "=" * 72 + "\033[0m")
            print("Launching hardware drivers, RPLiDAR C1, and SLAM Toolbox...")

            slam_proc = launch_proc("ros2 launch motor_control slam_bringup.launch.py", "SLAM_Bringup", "/tmp/slam_bringup.log")
            print("Waiting 4 seconds for sensors and SLAM Toolbox to initialize...")
            time.sleep(4.0)

            if "DISPLAY" in os.environ:
                rviz_proc = launch_proc(f"rviz2 -d {MAPPING_RVIZ}", "RViz2", "/tmp/rviz2.log")
                print("RViz2 visualizer launched.")

            print("\n\033[1;43;30m >>> ATTENTION: CLICK THIS TERMINAL WINDOW TO FOCUS BEFORE USING WASD! <<< \033[0m")
            print("\033[1;33m (If RViz2 or another window is active, keyboard input will not reach the terminal) \033[0m\n")
            print("\033[1;33m" + "-" * 72 + "\033[0m")
            print("  \033[1;37mTELEOP MAPPING CONTROLS:\033[0m")
            print("    [ W / Up ]    : Drive Forward       [ S / Down ]  : Drive Backward")
            print("    [ A / Left ]  : Turn Left           [ D / Right ] : Turn Right")
            print("    [ Q / E ]     : Curved Fwd-Left / Fwd-Right")
            print("    [ Space / X ] : STOP Robot")
            print("    [ T ]         : TOGGLE TURN MODE (Point Turn ⟲ <-> Single-Side Pivot ⮵)")
            print("    [ + / = ]     : Increase Linear Speed (+0.05 m/s)  [ - / _ ] : Decrease Speed (-0.05 m/s)")
            print("    [ [ / ] ]     : Adjust Turn Rate (+/- 0.05 rad/s)")
            print("  \033[1;32m>>> PRESS [ Ctrl+M ], [ M ], OR [ Enter ] TO SAVE MAP & PROCEED <<<\033[0m")
            print("    Press [ Ctrl+C ] to abort.")
            print("\033[1;33m" + "-" * 72 + "\033[0m\n")

            node.set_turn_mode(False) # Point turn by default for SLAM mapping
            node.start_teleop_publisher()
            motion_desc = "STOPPED ■"

            while rclpy.ok():
                key = get_key(orig_settings, timeout=0.05)
                rclpy.spin_once(node, timeout_sec=0.01)

                if key in ('w', 'W', '\x1b[A'):
                    node.set_motion(1.0, 0.0)
                    motion_desc = "FORWARD ▲"
                elif key in ('s', 'S', '\x1b[B'):
                    node.set_motion(-1.0, 0.0)
                    motion_desc = "BACKWARD ▼"
                elif key in ('a', 'A', '\x1b[D'):
                    node.set_motion(0.0, 1.0)
                    motion_desc = "PIVOT LEFT ◄"
                elif key in ('d', 'D', '\x1b[C'):
                    node.set_motion(0.0, -1.0)
                    motion_desc = "PIVOT RIGHT ►"
                elif key in ('q', 'Q'):
                    node.set_motion(1.0, 0.7)
                    motion_desc = "CURVE FWD-L ◄▲"
                elif key in ('e', 'E'):
                    node.set_motion(1.0, -0.7)
                    motion_desc = "CURVE FWD-R ▲►"
                elif key in (' ', 'x', 'X'):
                    node.stop_robot()
                    motion_desc = "STOPPED ■"
                elif key in ('t', 'T'):
                    node.toggle_turn_mode()
                elif key in ('+', '='):
                    node.adjust_linear_speed(0.05)
                elif key in ('-', '_'):
                    node.adjust_linear_speed(-0.05)
                elif key in (']', '}'):
                    node.adjust_angular_speed(0.05)
                elif key in ('[', '{'):
                    node.adjust_angular_speed(-0.05)
                elif key in ('\x0d', '\r', '\n', 'm', 'M'):  # Ctrl+M sends \r in raw mode
                    node.stop_robot()
                    print("\n\n\033[1;32m[TRIGGERED]\033[0m Save key pressed! Halting robot and saving map...")
                    break
                elif key == '\x03':  # Ctrl+C
                    node.stop_robot()
                    print("\n[ABORTED] User pressed Ctrl+C.")
                    return

                turn_lbl = "SINGLE ⮵" if node.single_side_turn else "TANK ⟲"
                status = f"\r[MAPPING: {motion_desc:<12}] [SPD: {node.linear_speed:.2f}m/s|{node.angular_speed:.2f}r/s] [{turn_lbl}] [T]Turn [+-]Spd [Ctrl+M/Enter]Save "
                sys.stdout.write(status)
                sys.stdout.flush()

            # Save map via nav2_map_server CLI
            print(f"\n\033[1;36m[SAVING MAP]\033[0m Running nav2_map_server map_saver_cli -> {MAP_PREFIX} ...")
            cmd_save = f"ros2 run nav2_map_server map_saver_cli -f {MAP_PREFIX} --ros-args --remap map:=/map"
            subprocess.run(cmd_save, shell=True)

            if not (os.path.exists(MAP_FILE) and os.path.exists(MAP_PGM)):
                print("\033[1;31m[ERROR]\033[0m Map saver failed to produce map files. Cannot continue.")
                return

            print(f"\033[1;32m[SUCCESS]\033[0m Map saved at: {MAP_FILE}")
            node.stop_robot()

            # Cleanly stop SLAM bringup
            print("\n\033[1;36m[TRANSITION]\033[0m Stopping SLAM for clean TF handoff to Nav2...")
            if slam_proc:
                kill_proc(slam_proc)
                slam_proc = None
            kill_all_zombies(clean_shm=False)
            time.sleep(2.0)
            active_map_file = MAP_FILE
        else:
            print(f"\n\033[1;32m[USING MAP]\033[0m Loaded map: {active_map_file}")

        # ======================================================================
        # STEP 2: LAUNCH NAV2 NAVIGATION SYSTEM
        # ======================================================================
        print("\n\033[1;34m" + "=" * 72 + "\033[0m")
        print("\033[1;32m   STEP 2: NAV2 NAVIGATION BRINGUP & LOCALIZATION   \033[0m")
        print("\033[1;34m" + "=" * 72 + "\033[0m")
        print(f"Launching Nav2 bringup stack with map: {active_map_file} ...")

        nav2_proc = launch_proc(
            f"ros2 launch motor_control nav2_bringup.launch.py map:={active_map_file}",
            "Nav2_Bringup",
            "/tmp/nav2_bringup.log"
        )

        print("Waiting for Nav2 lifecycle nodes (AMCL, Planner, Controller, BT Navigator)...")
        if not check_lifecycle_nodes(node, timeout_sec=25.0):
            print("\033[1;33m[WARNING]\033[0m Nav2 nodes taking longer than expected. Continuing...")
        else:
            print("\033[1;32m[ACTIVE]\033[0m Nav2 lifecycle nodes are fully ACTIVE!")

        time.sleep(2.0)

        # Launch RViz2 for Navigation if DISPLAY exists
        if "DISPLAY" in os.environ and rviz_proc is None:
            rviz_proc = launch_proc(f"rviz2 -d {MAPPING_RVIZ}", "RViz2", "/tmp/rviz2.log")

        # Load existing locations
        dock_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        pit_pose = {'x': 2.0, 'y': 2.0, 'yaw': 0.0}
        sweep_bounds = {'xmin': -1.0, 'xmax': 2.0, 'ymin': -1.0, 'ymax': 2.0, 'row_gap': 0.4, 'robot_radius': 0.3}

        has_existing_locs = False
        if os.path.exists(LOCATIONS_FILE):
            try:
                with open(LOCATIONS_FILE, 'r') as f:
                    loc_data = yaml.safe_load(f) or {}
                    if 'dock' in loc_data: dock_pose = loc_data['dock']
                    if 'pit' in loc_data: pit_pose = loc_data['pit']
                    if 'sweep' in loc_data: sweep_bounds = loc_data['sweep']
                    has_existing_locs = True
            except Exception:
                pass

        # Publish initial dock pose to AMCL and verify
        print("\033[1;36m[AUTONOMOUS LOCALIZATION]\033[0m Initializing robot 2D pose from saved Dock position...")
        node.publish_initial_pose(dock_pose)
        if AutonomousPoseCorrector:
            try:
                corrector = AutonomousPoseCorrector(parent_node=node, node_name='allrun_init_corrector')
                corrector.correct_pose(force=True, allow_twist=False)
            except Exception:
                pass

        # ======================================================================
        # STEP 3: TELEOP & POINT CAPTURE (DOCK, PIT, SWEEP SQUARE)
        # ======================================================================
        print("\n\033[1;34m" + "=" * 72 + "\033[0m")
        print("\033[1;32m   STEP 3: SAVE POINTS - DOCK, PIT & SWEEP SQUARE BOUNDS   \033[0m")
        print("\033[1;34m" + "=" * 72 + "\033[0m")
        if has_existing_locs:
            print("\033[1;36m[CURRENT SAVED LOCATIONS]\033[0m")
            print(f"  • Dock:  X={dock_pose.get('x', 0):.2f}, Y={dock_pose.get('y', 0):.2f}")
            print(f"  • Pit:   X={pit_pose.get('x', 0):.2f}, Y={pit_pose.get('y', 0):.2f}")
            print(f"  • Sweep: X[{sweep_bounds.get('xmin', 0):.2f}..{sweep_bounds.get('xmax', 0):.2f}], Y[{sweep_bounds.get('ymin', 0):.2f}..{sweep_bounds.get('ymax', 0):.2f}]")
            print("  -> Press \033[1;32m[ Enter ]\033[0m immediately to KEEP these saved spots, or drive with WASD to re-capture.\n")

        print("Controls to set points (drive to spot or click 2D Goal Pose in RViz):")
        print("  \033[1;33m[ 1 ]\033[0m         : Save DOCK position")
        print("  \033[1;33m[ 2 ]\033[0m         : Save PIT position")
        print("  \033[1;33m[ 3 ]\033[0m         : Save SWEEP BOTTOM-LEFT corner (xmin, ymin)")
        print("  \033[1;33m[ 4 ]\033[0m         : Save SWEEP TOP-RIGHT corner (xmax, ymax)")
        print("  \033[1;33m[ M ]\033[0m         : Manually enter coordinates (xmin, xmax, ymin, ymax)")
        print("  \033[1;33m[ T ]\033[0m         : TOGGLE TURN MODE (Point Turn ⟲ <-> Single-Side Pivot ⮵)")
        print("  \033[1;33m[ + / = ]\033[0m     : Increase Linear Speed (+0.05 m/s)  [ - / _ ] : Decrease (-0.05 m/s)")
        print("  \033[1;33m[ [ / ] ]\033[0m     : Adjust Turn Rate (+/- 0.05 rad/s)")
        print("  \033[1;32m[ Enter ]\033[0m     : CONFIRM POINTS & AUTO-GENERATE SNAKE PATTERN")
        print("  \033[1;37m[ Space / X ]\033[0m : Stop Robot")
        print("\033[1;34m" + "-" * 72 + "\033[0m")

        dock_set, pit_set, bl_set, tr_set = False, False, False, False
        node.set_turn_mode(False) # Default to Point Turn for locating/orienting at Pit and Dock!
        node.sync_nav2_speed()
        node.start_teleop_publisher()
        motion_desc = "STOPPED ■"

        while rclpy.ok():
            key = get_key(orig_settings, timeout=0.05)
            rclpy.spin_once(node, timeout_sec=0.01)

            if key in ('w', 'W', '\x1b[A'):
                node.set_motion(1.0, 0.0)
                motion_desc = "FORWARD ▲"
            elif key in ('s', 'S', '\x1b[B'):
                node.set_motion(-1.0, 0.0)
                motion_desc = "BACKWARD ▼"
            elif key in ('a', 'A', '\x1b[D'):
                node.set_motion(0.0, 1.0)
                motion_desc = "PIVOT LEFT ◄"
            elif key in ('d', 'D', '\x1b[C'):
                node.set_motion(0.0, -1.0)
                motion_desc = "PIVOT RIGHT ►"
            elif key in (' ', 'x', 'X'):
                node.stop_robot()
                motion_desc = "STOPPED ■"
            elif key in ('t', 'T'):
                node.toggle_turn_mode()
            elif key in ('+', '='):
                node.adjust_linear_speed(0.05)
            elif key in ('-', '_'):
                node.adjust_linear_speed(-0.05)
            elif key in (']', '}'):
                node.adjust_angular_speed(0.05)
            elif key in ('[', '{'):
                node.adjust_angular_speed(-0.05)

            # Hotkeys for capturing points
            elif key == '1':  # Dock
                p = node.get_robot_pose_tf(timeout_sec=0.5)
                src = "ROBOT TF"
                if not p and node.rviz_goal_pose and (time.time() - node.last_rviz_click_time) < 10.0:
                    p = node.rviz_goal_pose
                    src = "RVIZ"
                if p:
                    dock_pose = p
                    dock_set = True
                    print(f"\n\033[1;32m[✓ DOCK CAPTURED ({src})]\033[0m X={p['x']:.2f}, Y={p['y']:.2f}")
                else:
                    print("\n\033[1;31m[ERROR]\033[0m TF pose not available yet. Drive a bit or click in RViz.")

            elif key == '2':  # Pit
                p = node.get_robot_pose_tf(timeout_sec=0.5)
                src = "ROBOT TF"
                if not p and node.rviz_goal_pose and (time.time() - node.last_rviz_click_time) < 10.0:
                    p = node.rviz_goal_pose
                    src = "RVIZ"
                if p:
                    pit_pose = p
                    pit_set = True
                    print(f"\n\033[1;32m[✓ PIT CAPTURED ({src})]\033[0m X={p['x']:.2f}, Y={p['y']:.2f}")
                else:
                    print("\n\033[1;31m[ERROR]\033[0m TF pose not available yet. Drive a bit or click in RViz.")

            elif key == '3':  # Bottom-Left corner
                p = node.get_robot_pose_tf(timeout_sec=0.5)
                src = "ROBOT TF"
                if not p and node.rviz_goal_pose and (time.time() - node.last_rviz_click_time) < 10.0:
                    p = node.rviz_goal_pose
                    src = "RVIZ"
                if p:
                    sweep_bounds['xmin'] = p['x']
                    sweep_bounds['ymin'] = p['y']
                    bl_set = True
                    print(f"\n\033[1;32m[✓ SWEEP BL CORNER ({src})]\033[0m Xmin={p['x']:.2f}, Ymin={p['y']:.2f}")
                else:
                    print("\n\033[1;31m[ERROR]\033[0m TF pose not available yet.")

            elif key == '4':  # Top-Right corner
                p = node.get_robot_pose_tf(timeout_sec=0.5)
                src = "ROBOT TF"
                if not p and node.rviz_goal_pose and (time.time() - node.last_rviz_click_time) < 10.0:
                    p = node.rviz_goal_pose
                    src = "RVIZ"
                if p:
                    sweep_bounds['xmax'] = p['x']
                    sweep_bounds['ymax'] = p['y']
                    tr_set = True
                    print(f"\n\033[1;32m[✓ SWEEP TR CORNER ({src})]\033[0m Xmax={p['x']:.2f}, Ymax={p['y']:.2f}")
                else:
                    print("\n\033[1;31m[ERROR]\033[0m TF pose not available yet.")

            elif key in ('m', 'M'):  # Manual input
                node.stop_robot()
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
                try:
                    print("\n\n--- Enter Sweep Bounds Manually ---")
                    sweep_bounds['xmin'] = float(input(f"Xmin [{sweep_bounds['xmin']:.2f}]: ") or sweep_bounds['xmin'])
                    sweep_bounds['xmax'] = float(input(f"Xmax [{sweep_bounds['xmax']:.2f}]: ") or sweep_bounds['xmax'])
                    sweep_bounds['ymin'] = float(input(f"Ymin [{sweep_bounds['ymin']:.2f}]: ") or sweep_bounds['ymin'])
                    sweep_bounds['ymax'] = float(input(f"Ymax [{sweep_bounds['ymax']:.2f}]: ") or sweep_bounds['ymax'])
                    bl_set, tr_set = True, True
                    print(f"-> Sweep bounds set to X:[{sweep_bounds['xmin']:.2f}, {sweep_bounds['xmax']:.2f}], Y:[{sweep_bounds['ymin']:.2f}, {sweep_bounds['ymax']:.2f}]")
                except ValueError as ve:
                    print(f"Invalid input: {ve}")

            elif key in ('\x0d', '\r', '\n'):  # Confirm and continue
                node.stop_robot()
                xmin = min(float(sweep_bounds['xmin']), float(sweep_bounds['xmax']))
                xmax = max(float(sweep_bounds['xmin']), float(sweep_bounds['xmax']))
                ymin = min(float(sweep_bounds['ymin']), float(sweep_bounds['ymax']))
                ymax = max(float(sweep_bounds['ymin']), float(sweep_bounds['ymax']))
                sweep_bounds['xmin'] = xmin
                sweep_bounds['xmax'] = xmax
                sweep_bounds['ymin'] = ymin
                sweep_bounds['ymax'] = ymax

                dx = xmax - xmin
                dy = ymax - ymin
                if dx < 0.25 or dy < 0.25:
                    print(f"\n\033[1;31m[ERROR]\033[0m Sweep area too small ({dx:.2f}m x {dy:.2f}m). Minimum is 0.25m x 0.25m.")
                    print("  Please capture BL [3] and TR [4] corners again, or press [M] to enter manually.")
                    continue

                print(f"\n\n\033[1;32m[CONFIRMED]\033[0m Area Confirmed: Width={dx:.2f}m, Length={dy:.2f}m")
                break
            elif key == '\x03':  # Ctrl+C
                node.stop_robot()
                print("\n[ABORTED] User pressed Ctrl+C.")
                return

            turn_lbl = "SINGLE ⮵" if node.single_side_turn else "TANK ⟲"
            status = f"\r[{motion_desc:<10}] D:{'✓' if dock_set else '·'} P:{'✓' if pit_set else '·'} BL:{'✓' if bl_set else '·'} TR:{'✓' if tr_set else '·'} | [{node.linear_speed:.2f}m/s|{turn_lbl}] [T]Turn [+-]Spd [1-4]Set [Enter]Done "
            sys.stdout.write(status)
            sys.stdout.flush()

        # Save locations and generate snake pattern
        save_locations(dock_pose, pit_pose, sweep_bounds)
        snake_waypoints = generate_and_save_snake(sweep_bounds, active_map_file, path_publisher=node.path_pub)

        if not snake_waypoints:
            print("\033[1;31m[ERROR]\033[0m Could not generate snake waypoints. Check bounds.")
            return

        # ======================================================================
        # STEP 4: LAUNCH APP BRIDGE & ROSBRIDGE SERVER
        # ======================================================================
        print("\n\033[1;34m" + "=" * 72 + "\033[0m")
        print("\033[1;32m   STEP 4: ROVER APP BRIDGE & ROSBRIDGE SERVER BRINGUP   \033[0m")
        print("\033[1;34m" + "=" * 72 + "\033[0m")

        # Launch rover_bridge_node (port 8000)
        bridge_proc = launch_proc(f"python3 {BRIDGE_SCRIPT}", "App_Bridge_Server", "/tmp/app_bridge.log")
        print(" -> Rover App Bridge HTTP (:8000) and WebSocket (/ws/rover) launched.")

        # Launch rosbridge_websocket (port 9090)
        rosbridge_proc = launch_proc("ros2 launch rosbridge_server rosbridge_websocket_launch.xml", "Rosbridge_Server", "/tmp/rosbridge.log")
        print(" -> ROSBridge WebSocket (:9090) launched.")

        time.sleep(2.0)
        print("\033[1;32m[READY]\033[0m All communication channels active!")

        # ======================================================================
        # STEP 5: STANDBY & EXECUTION (TRIGGER FROM APP OR CLI)
        # ======================================================================
        print("\n\033[1;34m" + "=" * 72 + "\033[0m")
        print("\033[1;32m   STEP 5: ROBOT READY IN STANDBY (WAITING FOR APP OR CLI COMMAND)   \033[0m")
        print("\033[1;34m" + "=" * 72 + "\033[0m")
        print(f"  • Snake Waypoints   : \033[1;37m{len(snake_waypoints)}\033[0m points ready ({SNAKE_WAYPOINTS_FILE})")
        print(f"  • Turning Mode      : \033[1;32mSINGLE-SIDE MOTOR TURNING\033[0m (precise pivot, zero skid scrub)")
        print(f"  • App Bridge Server : \033[1;36mhttp://0.0.0.0:8000\033[0m (REST + /ws/rover)")
        print(f"  • Rosbridge Server  : \033[1;36mws://0.0.0.0:9090\033[0m")
        print(f"  • RViz Path View    : Visualized on /snake_path")
        print("\n\033[1;32m>>> YOU CAN NOW TRIGGER THE SNAKE PATTERN FROM THE APP OR CLI <<<\033[0m")
        print("  CLI Controls in Standby:")
        print("    \033[1;33m[ G ] or [ S ]\033[0m  : Run AUTONOMOUS SNAKE PATTERN SWEEP (Single-Side Turns)")
        print("    \033[1;33m[ 1 ] or [ D ]\033[0m  : Navigate to DOCK")
        print("    \033[1;33m[ 2 ] or [ P ]\033[0m  : Navigate to PIT")
        print("    \033[1;33m[ T ]\033[0m          : TOGGLE TURN MODE (Point Turn ⟲ <-> Single-Side Pivot ⮵)")
        print("    \033[1;33m[ + / = ]\033[0m      : Increase Linear Speed (+0.05 m/s)  [ - / _ ] : Decrease (-0.05 m/s)")
        print("    \033[1;33m[ [ / ] ]\033[0m      : Adjust Turn Rate (+/- 0.05 rad/s)")
        print("    \033[1;37m[ WASD ]\033[0m        : Manual Teleop Override")
        print("    \033[1;31m[ Space / X ]\033[0m   : Immediate Stop / E-Stop")
        print("    \033[1;37m[ Ctrl+C ]\033[0m      : Exit Allrun Pipeline cleanly")
        print("\033[1;43;30m >>> ATTENTION: CLICK THIS TERMINAL WINDOW TO FOCUS BEFORE TYPING! <<< \033[0m\n")
        print("\033[1;34m" + "-" * 72 + "\033[0m")

        node.stop_robot()
        node.set_turn_mode(False) # Tank turn default for navigation and teleop!
        node.sync_nav2_speed()
        node.start_teleop_publisher()
        motion_desc = "STANDBY ■"

        while rclpy.ok():
            key = get_key(orig_settings, timeout=0.06)
            rclpy.spin_once(node, timeout_sec=0.01)

            if key in ('g', 'G', 's', 'S'):
                run_snake_navigation(node, snake_waypoints, dock_pose, orig_settings=orig_settings)
                motion_desc = "STANDBY ■"
                print("\n\033[1;32m[STANDBY]\033[0m Robot in idle standby. Waiting for App or CLI command...")
            elif key in ('1', 'd', 'D'):
                navigate_to_location(node, dock_pose, "DOCK", orig_settings=orig_settings)
                motion_desc = "STANDBY ■"
            elif key in ('2', 'p', 'P'):
                navigate_to_location(node, pit_pose, "PIT", orig_settings=orig_settings)
                motion_desc = "STANDBY ■"
            elif key in ('t', 'T'):
                node.toggle_turn_mode()
            elif key in ('+', '='):
                node.adjust_linear_speed(0.05)
            elif key in ('-', '_'):
                node.adjust_linear_speed(-0.05)
            elif key in (']', '}'):
                node.adjust_angular_speed(0.05)
            elif key in ('[', '{'):
                node.adjust_angular_speed(-0.05)
            elif key in ('w', 'W', '\x1b[A'):
                node.set_motion(1.0, 0.0)
                motion_desc = "FORWARD ▲"
            elif key in ('s', 'S', '\x1b[B'):
                node.set_motion(-1.0, 0.0)
                motion_desc = "BACKWARD ▼"
            elif key in ('a', 'A', '\x1b[D'):
                node.set_motion(0.0, 1.0)
                motion_desc = "PIVOT LEFT ◄"
            elif key in ('d', 'D', '\x1b[C'):
                node.set_motion(0.0, -1.0)
                motion_desc = "PIVOT RIGHT ►"
            elif key in (' ', 'x', 'X'):
                node.stop_robot()
                motion_desc = "STOPPED ■"
            elif key == '\x03':  # Ctrl+C
                node.stop_robot()
                print("\n[EXIT] Exiting Allrun Pipeline...")
                break

            turn_lbl = "SINGLE ⮵" if node.single_side_turn else "TANK ⟲"
            status = f"\r[STATUS: {motion_desc:<10}] [SPD: {node.linear_speed:.2f}m/s] [{turn_lbl}] [G]Sweep [D]Dock [P]Pit [T]Turn [+-]Spd [Space]Stop "
            sys.stdout.write(status)
            sys.stdout.flush()

    finally:
        node.stop_robot()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("\n\033[1;36m[CLEANUP]\033[0m Shutting down all spawned processes...")
        for p, name in ACTIVE_PROCESSES:
            kill_proc(p)
        kill_all_zombies(clean_shm=False)
        print("\033[1;32m[ALLRUN TERMINATED CLEANLY]\033[0m")


def run_headless_boot(node: PipelineSupervisorNode, map_file: str = MAP_FILE):
    """Reliable headless boot manager for systemd / no-display operation."""
    print("==================================================")
    print("  HEADLESS AUTONOMOUS ROBOT BOOT MANAGER          ")
    print("==================================================")

    # 1. pigpiod
    ensure_pigpiod()

    # 2. Launch Nav2 & Hardware Bringup with retries
    nav2_proc = None
    max_retries = 3
    success = False

    for attempt in range(1, max_retries + 1):
        print(f"[BOOT] Attempt {attempt}/{max_retries}: Launching hardware & Nav2 bringup with map: {map_file}...")
        cmd = f"ros2 launch motor_control nav2_bringup.launch.py map:={map_file}"
        nav2_proc = launch_proc(cmd, "nav2_bringup")

        print("[BOOT] Waiting for Nav2 lifecycle nodes to become active...")
        if check_lifecycle_nodes(node, timeout_sec=30.0):
            print("\033[1;32m[SUCCESS]\033[0m Nav2 lifecycle nodes active!")
            success = True
            break

        print(f"\033[1;33m[WARNING]\033[0m Attempt {attempt} failed to reach active state! Retrying...")
        kill_proc(nav2_proc)
        kill_all_zombies(clean_shm=False)
        time.sleep(3.0)

    if not success:
        print("\033[1;31m[ERROR]\033[0m Nav2 bringup failed after maximum retries!")
        sys.exit(1)

    # 3. Publish Initial Pose
    dock_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    if os.path.exists(LOCATIONS_FILE):
        try:
            with open(LOCATIONS_FILE, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'dock' in data:
                    dock_pose = data['dock']
        except Exception as e:
            print(f"[Warning] Failed to read dock pose: {e}")
    node.publish_initial_pose(dock_pose)

    # 4. Launch App Bridge Node (port 8000) & Rosbridge Server (port 9090)
    print("[BOOT] Launching rover_bridge_node (HTTP :8000)...")
    bridge_proc = launch_proc(f"python3 {BRIDGE_SCRIPT}", "rover_bridge")

    print("[BOOT] Launching rosbridge_server (:9090)...")
    rosbridge_proc = launch_proc("ros2 launch rosbridge_server rosbridge_websocket_launch.xml", "rosbridge")

    print("\033[1;32m[BOOT COMPLETE]\033[0m System operational on ports 8000 and 9090.")

    try:
        bridge_proc.wait()
    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Shutting down headless supervisor...")
        for p, name in ACTIVE_PROCESSES:
            kill_proc(p)
        kill_all_zombies(clean_shm=False)


def main():
    parser = argparse.ArgumentParser(description="Autonomous Farm Cleaning Robot - Allrun Master Pipeline")
    parser.add_argument('--headless', '--service', action='store_true', help="Run headless service boot (no terminal TTY / systemd)")
    parser.add_argument('--interactive', '-i', action='store_true', help="Run full sequential mission interactively")
    parser.add_argument('--map', action='store_true', help="Force start with SLAM mapping mode")
    parser.add_argument('--skip-map', '--skip-mapping', action='store_true', help="Skip SLAM mapping and use latest saved map")
    parser.add_argument('--map-file', type=str, default=None, help="Explicit path to map YAML file")
    args = parser.parse_args()

    kill_all_zombies(clean_shm=False)
    ensure_pigpiod()
    rclpy.init()
    node = PipelineSupervisorNode()

    latest_map, _ = find_latest_saved_map()
    selected_map = args.map_file or latest_map or MAP_FILE

    # If headless requested or non-TTY (e.g. systemd service without terminal)
    if args.headless or not sys.stdin.isatty():
        try:
            run_headless_boot(node, map_file=selected_map)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    force_map = None
    if args.map:
        force_map = True
    elif args.skip_map:
        force_map = False

    orig_settings = termios.tcgetattr(sys.stdin)
    try:
        run_pipeline(node, orig_settings, force_map=force_map)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
