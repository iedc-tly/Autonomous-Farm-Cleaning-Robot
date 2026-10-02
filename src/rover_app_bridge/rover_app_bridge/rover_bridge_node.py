#!/usr/bin/env python3
"""
rover_bridge_node.py

ROS 2 Bridge & Boss Server for Autonomous Farm Cleaning Robot App.
Connects HTTP/WebSocket requests from the app to Nav2 execution & emergency stop:
  - POST /dock, /api/dock        -> Go to Docking (initial position)
  - POST /pit, /api/pit          -> Go to Pit location
  - POST /sweep, /api/sweep      -> Execute Snake Pattern sweep with 2D pose self-correction
  - POST /cmd_vel, /drive        -> Manual drive velocity control
  - POST /e_stop, /stop          -> Immediate Emergency Stop (Hardware PWM halt + Nav2 goal cancellation)
"""

import json
import os
import math
import time
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty as EmptyMsg, Bool
from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from lifecycle_msgs.srv import GetState
from flask import Flask, jsonify, request, Response
from flask_sock import Sock

from motor_control.farm_cleaner import Cleaner, map_pgm_to_png_bytes, make_snake, launch, stop, S

LOCATIONS_FILE = "/home/iedctly/ros2_ws/src/motor_control/config/locations.yaml"
SNAKE_WAYPOINTS_FILE = "/home/iedctly/ros2_ws/snake_waypoints.yaml"


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_to_quaternion(yaw: float) -> dict:
    return {
        'x': 0.0,
        'y': 0.0,
        'z': float(round(math.sin(yaw / 2.0), 4)),
        'w': float(round(math.cos(yaw / 2.0), 4))
    }


def parse_twist_payload(data: dict, speed_scale: float = 1.0) -> tuple:
    """Parse linear_x and angular_z from any app payload format."""
    if not isinstance(data, dict):
        return 0.0, 0.0

    # 1. Check direction/mode format
    mode = str(data.get('mode') or data.get('direction') or data.get('action') or data.get('cmd') or data.get('dir') or '').lower()
    if mode in ('forward', 'up', 'w'):
        return 0.05 * speed_scale, 0.0
    elif mode in ('backward', 'back', 'down', 's'):
        return -0.05 * speed_scale, 0.0
    elif mode in ('left', 'a'):
        return 0.0, 0.15 * speed_scale
    elif mode in ('right', 'd'):
        return 0.0, -0.15 * speed_scale
    elif mode in ('stop', 'brake', 'halt'):
        return 0.0, 0.0

    # 2. Check ROS Twist structure ({"linear": {"x": ...}, "angular": {"z": ...}})
    linear_data = data.get('linear')
    angular_data = data.get('angular')

    linear_x = 0.0
    angular_z = 0.0

    def _to_float(v, default=0.0):
        try:
            return float(v)
        except (ValueError, TypeError):
            return default

    if isinstance(linear_data, dict):
        linear_x = _to_float(linear_data.get('x', 0.0))
    elif linear_data is not None:
        linear_x = _to_float(linear_data)
    elif 'linear_x' in data:
        linear_x = _to_float(data.get('linear_x'))
    elif 'x' in data:
        linear_x = _to_float(data.get('x'))
    elif 'vx' in data:
        linear_x = _to_float(data.get('vx'))
    elif 'v' in data:
        linear_x = _to_float(data.get('v'))

    if isinstance(angular_data, dict):
        angular_z = _to_float(angular_data.get('z', 0.0))
    elif angular_data is not None:
        angular_z = _to_float(angular_data)
    elif 'angular_z' in data:
        angular_z = _to_float(data.get('angular_z'))
    elif 'z' in data:
        angular_z = _to_float(data.get('z'))
    elif 'wz' in data:
        angular_z = _to_float(data.get('wz'))
    elif 'w' in data:
        angular_z = _to_float(data.get('w'))

    # Clamp manual teleop speeds to safe maximum limits (0.05 m/s, 0.15 rad/s)
    linear_x = _clamp(linear_x * speed_scale, -0.05, 0.05)
    angular_z = _clamp(angular_z * speed_scale, -0.15, 0.15)

    return linear_x, angular_z


def load_and_validate_locations():
    path = os.path.expanduser(LOCATIONS_FILE)
    if not os.path.exists(path):
        return None, "locations.yaml missing"
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
    except Exception as e:
        return None, f"locations.yaml parse error: {e}"

    if not isinstance(data, dict):
        return None, "locations.yaml must be a dictionary"

    # Validate dock
    dock = data.get('dock')
    if not isinstance(dock, dict) or not all(k in dock for k in ('x', 'y', 'yaw')):
        return None, "dock location invalid (missing x, y, yaw)"
    for k in ('x', 'y', 'yaw'):
        if not isinstance(dock[k], (int, float)) or isinstance(dock[k], bool):
            return None, f"dock.{k} must be numeric"

    # Validate pit
    pit = data.get('pit')
    if not isinstance(pit, dict) or not all(k in pit for k in ('x', 'y', 'yaw')):
        return None, "pit location invalid (missing x, y, yaw)"
    for k in ('x', 'y', 'yaw'):
        if not isinstance(pit[k], (int, float)) or isinstance(pit[k], bool):
            return None, f"pit.{k} must be numeric"

    # Validate sweep
    sweep = data.get('sweep')
    sweep_keys = ('xmin', 'xmax', 'ymin', 'ymax', 'row_gap', 'robot_radius')
    if not isinstance(sweep, dict) or not all(k in sweep for k in sweep_keys):
        return None, "sweep location invalid (missing required bounds)"
    for k in sweep_keys:
        if not isinstance(sweep[k], (int, float)) or isinstance(sweep[k], bool):
            return None, f"sweep.{k} must be numeric"

    if float(sweep['xmin']) >= float(sweep['xmax']):
        return None, "sweep.xmin must be less than sweep.xmax"
    if float(sweep['ymin']) >= float(sweep['ymax']):
        return None, "sweep.ymin must be less than sweep.ymax"

    res = {
        'dock': {'x': float(dock['x']), 'y': float(dock['y']), 'yaw': float(dock['yaw'])},
        'pit': {'x': float(pit['x']), 'y': float(pit['y']), 'yaw': float(pit['yaw'])},
        'sweep': {
            'xmin': float(sweep['xmin']),
            'xmax': float(sweep['xmax']),
            'ymin': float(sweep['ymin']),
            'ymax': float(sweep['ymax']),
            'row_gap': float(sweep['row_gap']),
            'robot_radius': float(sweep['robot_radius']),
        }
    }
    return res, None


class BridgeNode(Node):
    def __init__(self):
        super().__init__('rover_bridge_node')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_hw_estop = self.create_publisher(EmptyMsg, '/estop', 10)
        self.pub_turn_mode = self.create_publisher(Bool, '/single_side_turn', 10)
        self.single_side_turn = False

        # Native ROS 2 Action Clients for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.cleaner = Cleaner()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle_clients = {}

        self.lock = threading.Lock()

        # State Machine Fields
        self.state = "IDLE"  # IDLE | SWEEPING | DOCKING | GOING_TO_PIT
        self.task = None      # sweep | dock | pit | None
        self.percent = 0.0
        self.waypoint_index = 0
        self.waypoint_total = 0
        self.last_task = None
        self.last_result = None
        self.error = None

        self.current_goal_handle = None
        self.cancel_requested = False

        self.status = {
            'e_stop': False,
            'auto_clean_active': False,
            'cycles': 3,
            'speed_scale': 1.0,
        }
        self.telemetry = {
            'battery': 82,
            'signal': 'Strong',
            'operation_time': '1h 15m',
            'zone': 'Sector 4-B',
            'diagnostic_ok': True,
        }
        self.get_logger().info('rover_bridge_node ready (App Bridge & Emergency Stop connected)!')

    def estop_guard_loop(self):
        """Guard loop holding motors at 0 velocity when E-STOP active without flooding /estop."""
        while rclpy.ok():
            if self.status.get('e_stop', False):
                self.publish_twist(0.0, 0.0)
            time.sleep(0.5)

    def is_nav2_active(self) -> bool:
        """Check if Nav2 Action Servers are ready to accept goals."""
        return (self.nav_to_pose_client.server_is_ready() and 
                self.follow_waypoints_client.server_is_ready())

    def ensure_nav2_active(self) -> bool:
        """Ensure Nav2 stack is active, launching it automatically if needed."""
        if self.is_nav2_active():
            return True

        self.get_logger().info("[BridgeNode] Nav2 navigation stack not active. Auto-bringing up Nav2...")
        map_yaml = getattr(self.cleaner, 'map_yaml', '/home/iedctly/my_map.yaml')
        if not os.path.exists(map_yaml):
            map_yaml = "/home/iedctly/my_map.yaml"

        if not os.path.exists(map_yaml):
            self.get_logger().error(f"[BridgeNode] Map file missing at {map_yaml}! Cannot bring up Nav2.")
            return False

        if self.cleaner.nav is None or self.cleaner.nav.poll() is not None:
            cmd = f"ros2 launch motor_control nav2_bringup.launch.py map:={map_yaml}"
            self.cleaner.nav = launch(cmd)

        start_t = time.time()
        while (time.time() - start_t) < 20.0:
            time.sleep(0.5)
            if self.is_nav2_active():
                self.get_logger().info("[BridgeNode] Nav2 navigation stack active!")
                self.publish_initial_dock_pose()
                return True

        self.get_logger().error("[BridgeNode] Timed out waiting for Nav2 action servers.")
        return False

    def restart_nav2(self) -> bool:
        """Restart Nav2 stack cleanly without touching hardware drivers."""
        import subprocess
        self.get_logger().warn("[BridgeNode] Restarting Nav2 navigation stack...")
        if self.cleaner.nav is not None:
            stop(self.cleaner.nav, timeout=3.0)
            self.cleaner.nav = None
        # Clean up only Nav2 nodes, preserving motor_node, encoder_odom_node, rplidar_node
        nav2_targets = [
            "bt_navigator", "controller_server", "planner_server",
            "behavior_server", "waypoint_follower", "amcl", "map_server"
        ]
        for target in nav2_targets:
            subprocess.run(f"pkill -9 -f {target} >/dev/null 2>&1", shell=True)
        time.sleep(1.0)
        return self.ensure_nav2_active()


    def publish_initial_dock_pose(self):
        """Publish initial pose estimation (dock position) to AMCL /initialpose."""
        locs, _ = load_and_validate_locations()
        dock = locs['dock'] if locs else {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.pose.position.x = float(dock['x'])
        p.pose.pose.position.y = float(dock['y'])
        q = yaw_to_quaternion(float(dock['yaw']))
        p.pose.pose.orientation.x = q['x']
        p.pose.pose.orientation.y = q['y']
        p.pose.pose.orientation.z = q['z']
        p.pose.pose.orientation.w = q['w']
        p.pose.covariance[0] = 0.05
        p.pose.covariance[7] = 0.05
        p.pose.covariance[35] = 0.05

        for _ in range(3):
            self.pub_initial_pose.publish(p)
            time.sleep(0.05)

    def get_current_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=0.2)
            )
            t = trans.transform.translation
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return {
                'x': float(round(t.x, 3)),
                'y': float(round(t.y, 3)),
                'yaw': float(round(yaw, 3))
            }
        except Exception:
            return None

    def publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub_cmd_vel.publish(msg)

    def get_status_response(self):
        with self.lock:
            st = {
                "state": self.state,
                "task": self.task if self.state != "IDLE" else None,
                "percent": self.percent,
                "waypoint_index": self.waypoint_index,
                "waypoint_total": self.waypoint_total,
                "pose": self.get_current_pose(),
                "nav2_active": self.is_nav2_active(),
                "last_task": self.last_task,
                "last_result": self.last_result,
                "error": self.error,
                "e_stop": self.status.get('e_stop', False)
            }
            return st

    def trigger_e_stop(self):
        """Execute complete Emergency Stop across hardware motors, Nav2 goals, and orchestrator without killing process."""
        self.get_logger().warn("\033[1;31m[EMERGENCY STOP TRIGGERED]\033[0m Halting hardware motors and cancelling all navigation tasks!")

        with self.lock:
            was_running = (self.state != "IDLE")
            running_task = self.task
            self.cancel_requested = True
            self.status['e_stop'] = True
            if was_running:
                self.last_task = running_task
                self.last_result = "canceled"
                self.error = "canceled"
            self.state = "IDLE"
            self.task = None

        # 1. Publish hardware E-STOP to /estop (motor_ros_node triggers mc.stop_all(immediate=True))
        try:
            empty_msg = EmptyMsg()
            for _ in range(5):
                self.pub_hw_estop.publish(empty_msg)
                time.sleep(0.01)
        except Exception as e:
            self.get_logger().error(f"Error publishing hardware estop: {e}")

        # 2. Cancel Nav2 active action goal safely
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().error(f"Error cancelling Nav2 goal: {e}")

        # 3. Cleanly cancel cleaner task if running without killing navigation or hardware stack
        try:
            if hasattr(self.cleaner, 'navigator') and self.cleaner.navigator:
                self.cleaner.navigator.cancelTask()
            self.cleaner.state = S.IDLE
        except Exception as e:
            self.get_logger().error(f"Error cancelling cleaner task: {e}")

        # 4. Burst zero velocity command override to /cmd_vel
        try:
            for _ in range(10):
                self.publish_twist(0.0, 0.0)
                time.sleep(0.02)
        except Exception as e:
            self.get_logger().error(f"Error publishing zero twist: {e}")

    def handle_dock_request(self):
        try:
            with self.lock:
                if self.status.get('e_stop', False):
                    return jsonify(error="e_stop_active"), 403
                if self.state != "IDLE":
                    return jsonify(error="busy"), 409

            locs, err = load_and_validate_locations()
            if err:
                return jsonify(error=err), 412

            with self.lock:
                self.state = "DOCKING"
                self.task = "dock"
                self.last_task = None
                self.last_result = None
                self.error = None
                self.percent = 0.0
                self.waypoint_index = 0
                self.waypoint_total = 1
                self.cancel_requested = False

            threading.Thread(
                target=self._run_goto_pose,
                args=("dock", locs['dock']),
                daemon=True
            ).start()

            return jsonify(accepted=True, waypoint_total=1), 202
        except Exception as e:
            self.get_logger().error(f"Error handling dock request: {e}")
            return jsonify(ok=False, error=str(e)), 500

    def handle_pit_request(self):
        try:
            with self.lock:
                if self.status.get('e_stop', False):
                    return jsonify(error="e_stop_active"), 403
                if self.state != "IDLE":
                    return jsonify(error="busy"), 409

            locs, err = load_and_validate_locations()
            if err:
                return jsonify(error=err), 412

            with self.lock:
                self.state = "GOING_TO_PIT"
                self.task = "pit"
                self.last_task = None
                self.last_result = None
                self.error = None
                self.percent = 0.0
                self.waypoint_index = 0
                self.waypoint_total = 1
                self.cancel_requested = False

            threading.Thread(
                target=self._run_goto_pose,
                args=("pit", locs['pit']),
                daemon=True
            ).start()

            return jsonify(accepted=True, waypoint_total=1), 202
        except Exception as e:
            self.get_logger().error(f"Error handling pit request: {e}")
            return jsonify(ok=False, error=str(e)), 500

    def _validate_waypoints_safety(self, waypoints: list, map_yaml: str, robot_radius: float = 0.48) -> bool:
        """Check if all waypoints are inside map free space with sufficient clearance."""
        if not waypoints or not os.path.exists(map_yaml):
            return False
        try:
            with open(map_yaml, 'r') as f:
                meta = yaml.safe_load(f)
            from PIL import Image
            import numpy as np
            img_rel = meta['image']
            img_path = img_rel if img_rel.startswith('/') else os.path.join(os.path.dirname(map_yaml), img_rel)
            if not os.path.exists(img_path):
                return True
            img = Image.open(img_path)
            grid = np.array(img)
            h, w = grid.shape
            res = float(meta['resolution'])
            ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
            radius_px = int(math.ceil(robot_radius / res))

            for wp in waypoints:
                x, y = float(wp['x']), float(wp['y'])
                c_col = int(math.floor((x - ox) / res))
                c_row = int(math.floor(h - 1 - (y - oy) / res))
                if c_col < 0 or c_col >= w or c_row < 0 or c_row >= h:
                    return False
                for dr in range(-radius_px, radius_px + 1):
                    for dc in range(-radius_px, radius_px + 1):
                        if dr * dr + dc * dc > radius_px * radius_px:
                            continue
                        r_i, c_i = c_row + dr, c_col + dc
                        if r_i < 0 or r_i >= h or c_i < 0 or c_i >= w:
                            return False
                        if grid[r_i, c_i] < 100:
                            return False
            return True
        except Exception as e:
            self.get_logger().warn(f"Waypoint validation exception: {e}")
            return True

    def handle_sweep_request(self):
        try:
            with self.lock:
                if self.status.get('e_stop', False):
                    return jsonify(error="e_stop_active"), 403
                if self.state != "IDLE":
                    return jsonify(error="busy"), 409

            locs, err = load_and_validate_locations()
            if err:
                return jsonify(error=err), 412

            sweep_cfg = locs['sweep']
            map_yaml = "/home/iedctly/my_map.yaml"
            waypoints = None
            if os.path.exists(SNAKE_WAYPOINTS_FILE):
                try:
                    with open(SNAKE_WAYPOINTS_FILE, 'r') as f:
                        sw_data = yaml.safe_load(f)
                        if sw_data and 'waypoints' in sw_data and len(sw_data['waypoints']) > 0:
                            cand_wps = sw_data['waypoints']
                            if self._validate_waypoints_safety(cand_wps, map_yaml, robot_radius=0.48):
                                waypoints = cand_wps
                                self.get_logger().info(f"[BridgeNode] Loaded {len(waypoints)} validated safe waypoints from {SNAKE_WAYPOINTS_FILE}")
                            else:
                                self.get_logger().warn(f"[BridgeNode] {SNAKE_WAYPOINTS_FILE} contains waypoints in collision or out of map range! Regenerating with map filter...")
                except Exception as e:
                    self.get_logger().warn(f"Failed to read {SNAKE_WAYPOINTS_FILE}: {e}")

            if not waypoints:
                bounds = [sweep_cfg['xmin'], sweep_cfg['xmax'], sweep_cfg['ymin'], sweep_cfg['ymax']]
                if abs(bounds[1] - bounds[0]) < 0.5 or abs(bounds[3] - bounds[2]) < 0.5:
                    from motor_control.farm_cleaner import get_entire_room_bounds
                    bounds = get_entire_room_bounds(map_yaml, robot_radius=0.48)
                    self.get_logger().info(f"[BridgeNode] Auto-expanded bounds to Entire Room: {bounds}")

                radius = max(float(sweep_cfg.get('robot_radius', 0.48)), 0.48)
                waypoints = make_snake(
                    map_yaml,
                    bounds=bounds,
                    row_gap=sweep_cfg.get('row_gap', 0.5),
                    step_x=sweep_cfg.get('row_gap', 0.5),
                    robot_radius=radius
                )
                if waypoints and len(waypoints) > 0:
                    try:
                        with open(SNAKE_WAYPOINTS_FILE, 'w') as f:
                            yaml.dump({'num_waypoints': len(waypoints), 'waypoints': waypoints}, f, default_flow_style=False)
                        self.get_logger().info(f"[BridgeNode] Regenerated and saved {len(waypoints)} safe waypoints to {SNAKE_WAYPOINTS_FILE}")
                    except Exception as e:
                        self.get_logger().warn(f"Failed to update {SNAKE_WAYPOINTS_FILE}: {e}")

            if not waypoints or len(waypoints) == 0:
                self.get_logger().error("[BridgeNode] Failed to generate waypoints: area out of map range or blocked by obstacles!")
                return jsonify(ok=False, error="Sweep area is out of map range or blocked by obstacles. Please reconfigure bounds in configure_locations.sh"), 412

            with self.lock:
                self.state = "SWEEPING"
                self.task = "sweep"
                self.last_task = None
                self.last_result = None
                self.error = None
                self.percent = 0.0
                self.waypoint_index = 0
                self.waypoint_total = len(waypoints)
                self.cancel_requested = False

            threading.Thread(
                target=self._run_sweep,
                args=(waypoints,),
                daemon=True
            ).start()

            return jsonify(accepted=True, waypoints=len(waypoints)), 202
        except Exception as e:
            self.get_logger().error(f"Error handling sweep request: {e}")
            return jsonify(ok=False, error=str(e)), 500

    def set_turn_mode(self, single_side: bool):
        with self.lock:
            self.single_side_turn = bool(single_side)
        msg = Bool()
        msg.data = self.single_side_turn
        for _ in range(2):
            self.pub_turn_mode.publish(msg)
            time.sleep(0.01)
        mode_str = "SINGLE-SIDE PIVOT" if self.single_side_turn else "TANK TURN (DUAL-SIDE ZERO RADIUS)"
        self.get_logger().info(f"[BridgeNode] Turn mode switched to: {mode_str}")

    def _run_goto_pose(self, task_name, target):
        outcome = "failed"
        err = "navigation failed"
        try:
            self.set_turn_mode(False) # Point turn for dock/pit orientation
            if not self.ensure_nav2_active():
                with self.lock:
                    self.last_task = task_name
                    self.last_result = "failed"
                    self.error = "nav2 bringup failed"
                    self.state = "IDLE"
                    self.task = None
                return

            self.get_logger().info(f"[BridgeNode] Executing {task_name} goal: X={target['x']}, Y={target['y']}")

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(target['x'])
            goal_msg.pose.pose.position.y = float(target['y'])
            q = yaw_to_quaternion(float(target['yaw']))
            goal_msg.pose.pose.orientation.x = q['x']
            goal_msg.pose.pose.orientation.y = q['y']
            goal_msg.pose.pose.orientation.z = q['z']
            goal_msg.pose.pose.orientation.w = q['w']

            future_goal = self.nav_to_pose_client.send_goal_async(goal_msg)

            start_t = time.time()
            while not future_goal.done() and (time.time() - start_t) < 5.0:
                time.sleep(0.02)

            if not future_goal.done() or not future_goal.result().accepted:
                self.get_logger().error(f"[BridgeNode] {task_name} goal was rejected by Nav2!")
                with self.lock:
                    self.last_task = task_name
                    self.last_result = "failed"
                    self.error = "goal rejected"
                    self.state = "IDLE"
                    self.task = None
                return

            self.current_goal_handle = future_goal.result()
            self.get_logger().info(f"[BridgeNode] {task_name} goal accepted by Nav2!")

            future_res = self.current_goal_handle.get_result_async()

            while not future_res.done():
                with self.lock:
                    if self.cancel_requested or self.status.get('e_stop', False):
                        self.get_logger().info(f"[BridgeNode] Cancelling {task_name} goal...")
                        try:
                            self.current_goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                        break
                time.sleep(0.1)

            with self.lock:
                if self.cancel_requested or self.status.get('e_stop', False):
                    outcome = "canceled"
                    err = "canceled"
                else:
                    action_res = future_res.result()
                    if action_res and action_res.status == GoalStatus.STATUS_SUCCEEDED:
                        outcome = "succeeded"
                        err = None
                    elif action_res and action_res.status == GoalStatus.STATUS_CANCELED:
                        outcome = "canceled"
                        err = "canceled"
                    else:
                        outcome = "failed"
                        err = "navigation failed"

        except Exception as e:
            self.get_logger().error(f"Error in {task_name}: {e}")
            outcome = "failed"
            err = str(e)

        with self.lock:
            self.last_task = task_name
            self.last_result = outcome
            self.error = err
            self.state = "IDLE"
            self.task = None
            self.current_goal_handle = None
            if outcome == "succeeded":
                self.percent = 100.0
                self.waypoint_index = 1
                self.waypoint_total = 1

    def _on_sweep_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        cw = getattr(feedback, 'current_waypoint', None)
        if cw is not None:
            cur_idx = cw + 1
            with self.lock:
                if cur_idx > self.waypoint_index:
                    self.waypoint_index = cur_idx
                    if self.waypoint_total > 0:
                        self.percent = min(100.0, float(round((self.waypoint_index / self.waypoint_total) * 100.0, 1)))

    def _run_sweep(self, waypoints):
        outcome = "failed"
        err = "navigation failed"
        try:
            if not self.ensure_nav2_active():
                with self.lock:
                    self.last_task = "sweep"
                    self.last_result = "failed"
                    self.error = "nav2 bringup failed"
                    self.state = "IDLE"
                    self.task = None
                return

            self.set_turn_mode(False) # Tank turn (dual-side zero radius) for lawnmower snake sweep
            self.get_logger().info(f"[BridgeNode] Executing sweep with {len(waypoints)} waypoints (Tank Turn Motor Turning)...")

            goal_msg = FollowWaypoints.Goal()
            pose_list = []
            for wp in waypoints:
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.header.stamp = self.get_clock().now().to_msg()
                p.pose.position.x = float(wp['x'])
                p.pose.position.y = float(wp['y'])
                q = wp.get('orientation') or yaw_to_quaternion(float(wp.get('yaw', 0.0)))
                p.pose.orientation.x = float(q['x'])
                p.pose.orientation.y = float(q['y'])
                p.pose.orientation.z = float(q['z'])
                p.pose.orientation.w = float(q['w'])
                pose_list.append(p)
            goal_msg.poses = pose_list

            future_goal = self.follow_waypoints_client.send_goal_async(
                goal_msg, 
                feedback_callback=self._on_sweep_feedback
            )

            start_t = time.time()
            while not future_goal.done() and (time.time() - start_t) < 5.0:
                time.sleep(0.02)

            if not future_goal.done() or not future_goal.result().accepted:
                self.get_logger().error("[BridgeNode] Sweep waypoints goal rejected by Nav2!")
                with self.lock:
                    self.last_task = "sweep"
                    self.last_result = "failed"
                    self.error = "waypoints goal rejected"
                    self.state = "IDLE"
                    self.task = None
                return

            self.current_goal_handle = future_goal.result()
            self.get_logger().info("[BridgeNode] Sweep goal accepted by Nav2!")

            # 2D pose self-corrector sharing parent_node (BridgeNode) to prevent context corruption
            corrector = None
            try:
                from motor_control.pose_corrector import AutonomousPoseCorrector
                corrector = AutonomousPoseCorrector(parent_node=self)
                corrector.sharpen_pose(timeout_sec=0.1)
            except Exception as e:
                self.get_logger().warn(f"Pose corrector note: {e}")

            future_res = self.current_goal_handle.get_result_async()
            last_wp_idx = -1
            last_sharpen_time = time.time()

            while not future_res.done():
                with self.lock:
                    if self.cancel_requested or self.status.get('e_stop', False):
                        self.get_logger().info("[BridgeNode] Cancelling sweep task...")
                        try:
                            self.current_goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                        break

                if corrector:
                    cw = self.waypoint_index
                    now_t = time.time()
                    if cw != last_wp_idx:
                        last_wp_idx = cw
                        last_sharpen_time = now_t
                        corrector.sharpen_pose(timeout_sec=0.05)
                    elif (now_t - last_sharpen_time) > 3.0:
                        # Periodic AMCL particle sharpening during long straight lines
                        last_sharpen_time = now_t
                        corrector.sharpen_pose(timeout_sec=0.05)

                time.sleep(0.1)

            with self.lock:
                if self.cancel_requested or self.status.get('e_stop', False):
                    outcome = "canceled"
                    err = "canceled"
                else:
                    action_res = future_res.result()
                    if action_res and action_res.status == GoalStatus.STATUS_SUCCEEDED:
                        outcome = "succeeded"
                        err = None
                    elif action_res and action_res.status == GoalStatus.STATUS_CANCELED:
                        outcome = "canceled"
                        err = "canceled"
                    else:
                        outcome = "failed"
                        err = "navigation failed"

        except Exception as e:
            self.get_logger().error(f"Error in sweep: {e}")
            outcome = "failed"
            err = str(e)

        with self.lock:
            self.last_task = "sweep"
            self.last_result = outcome
            self.error = err
            self.state = "IDLE"
            self.task = None
            self.current_goal_handle = None
            if outcome == "succeeded":
                self.percent = 100.0
                self.waypoint_index = self.waypoint_total

        # Autonomously navigate to Pit after completing all sweep positions
        if outcome == "succeeded" and not self.cancel_requested and not self.status.get('e_stop', False):
            self.get_logger().info("\033[1;32m[BridgeNode SWEEP COMPLETE]\033[0m All waypoints hit! Autonomously proceeding to Pit...")
            locs, _ = load_and_validate_locations()
            if locs and 'pit' in locs:
                with self.lock:
                    self.state = "NAVIGATING_TO_PIT"
                    self.task = "pit"
                self._run_goto_pose("pit", locs['pit'])

    def get_status(self):
        with self.lock:
            st = dict(self.status)
            st['boss_state'] = self.cleaner.state.name
            st['boss_status'] = self.cleaner.get_status_dict()
            return st

    def get_telemetry(self):
        with self.lock:
            return dict(self.telemetry)

    def set_estop(self, value: bool):
        if value:
            self.trigger_e_stop()
        else:
            with self.lock:
                self.status['e_stop'] = False
                self.cancel_requested = False

    def set_speed_scale(self, scale: float):
        with self.lock:
            self.status['speed_scale'] = _clamp(float(scale), 0.0, 1.0)


# Connection registry for WebSocket subscribers
connections = []
connections_lock = threading.Lock()


def broadcast(topic: str, msg: dict):
    payload = json.dumps({'op': 'publish', 'topic': topic, 'msg': msg})
    with connections_lock:
        dead = []
        for conn in connections:
            if topic in conn['subs']:
                try:
                    conn['ws'].send(payload)
                except Exception:
                    dead.append(conn)
        for d in dead:
            if d in connections:
                connections.remove(d)


def handle_message(node: BridgeNode, conn: dict, raw: str):
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return

    op = data.get('op')

    if op == 'subscribe':
        topic = data.get('topic')
        if topic:
            conn['subs'].add(topic)
            if topic == '/rover/status':
                conn['ws'].send(json.dumps({
                    'op': 'publish', 'topic': '/rover/status', 'msg': node.get_status()
                }))
            elif topic == '/rover/telemetry':
                conn['ws'].send(json.dumps({
                    'op': 'publish', 'topic': '/rover/telemetry', 'msg': node.get_telemetry()
                }))
            elif topic == '/rover/sweep_status':
                conn['ws'].send(json.dumps({
                    'op': 'publish', 'topic': '/rover/sweep_status', 'msg': node.cleaner.get_status_dict()
                }))
        return

    if op == 'ping':
        return

    if op != 'publish':
        return

    topic = data.get('topic')
    msg = data.get('msg') or {}

    if topic in ('/cmd_vel', '/api/cmd_vel', '/drive', '/api/drive', '/teleop', '/api/teleop'):
        if node.status.get('e_stop', False) or node.state != "IDLE":
            node.publish_twist(0.0, 0.0)
            return

        linear_x, angular_z = parse_twist_payload(msg, speed_scale=node.status.get('speed_scale', 1.0))
        node.publish_twist(linear_x, angular_z)

    elif topic in ('/rover/e_stop', '/rover/estop', '/rover/stop', '/estop', '/stop'):
        val = bool(msg.get('data', True)) if isinstance(msg, dict) else True
        node.set_estop(val)
        broadcast('/rover/status', node.get_status_response())

    elif topic == '/rover/speed_scale':
        node.set_speed_scale(float(msg.get('data', 1.0)))
        broadcast('/rover/status', node.get_status())


def make_flask_app(node: BridgeNode) -> Flask:
    app = Flask(__name__)
    app.url_map.strict_slashes = False
    sock = Sock(app)

    @app.after_request
    def add_cors_headers(response):
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type,Authorization'
        response.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
        return response

    @app.errorhandler(404)
    def page_not_found(e):
        node.get_logger().warn(f"\033[1;33m[BridgeNode 404]\033[0m Request to unknown route: {request.method} {request.path}")
        path_lower = request.path.lower()
        if any(k in path_lower for k in ('cmd', 'vel', 'drive', 'move', 'teleop', 'control', 'dir', 'nav', 'steer')):
            node.get_logger().info(f"[BridgeNode Fallback] Auto-routing unrecognized motion endpoint '{request.path}' to cmd_vel!")
            data = {}
            if request.is_json:
                data.update(request.get_json(silent=True) or {})
            if request.args:
                data.update(request.args.to_dict())
            if request.form:
                data.update(request.form.to_dict())
            linear_x, angular_z = parse_twist_payload(data, speed_scale=node.status.get('speed_scale', 1.0))
            node.publish_twist(linear_x, angular_z)
            return jsonify(ok=True, linear_x=linear_x, angular_z=angular_z, notice="auto_routed_motion_fallback"), 200
        return jsonify(error="Not Found", path=request.path, method=request.method), 404

    @app.route('/health', methods=['GET'])
    @app.route('/api/health', methods=['GET'])
    def health():
        try:
            return jsonify(ok=True), 200
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/locations', methods=['GET'])
    @app.route('/api/locations', methods=['GET'])
    def get_locations():
        try:
            data, err = load_and_validate_locations()
            if err:
                return jsonify(error=err), 412
            return jsonify(data), 200
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/status', methods=['GET'])
    @app.route('/api/status', methods=['GET'])
    def get_status():
        try:
            st = node.get_status_response()
            return jsonify(st), 200
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/dock', methods=['POST'])
    @app.route('/api/dock', methods=['POST'])
    def post_dock():
        return node.handle_dock_request()

    @app.route('/pit', methods=['POST'])
    @app.route('/api/pit', methods=['POST'])
    def post_pit():
        return node.handle_pit_request()

    @app.route('/sweep', methods=['POST'])
    @app.route('/api/sweep', methods=['POST'])
    def post_sweep():
        return node.handle_sweep_request()

    # Manual driving velocity commands (HTTP POST & GET across all route aliases)
    @app.route('/cmd_vel', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/api/cmd_vel', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/cmdvel', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/api/cmdvel', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/drive', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/api/drive', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/teleop', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/api/teleop', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/manual', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/api/manual', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/move', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/api/move', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/control', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/api/control', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/rover/cmd_vel', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/api/rover/cmd_vel', methods=['GET', 'POST', 'OPTIONS'])
    def post_cmd_vel():
        try:
            if request.method == 'OPTIONS':
                return jsonify(ok=True), 200

            data = {}
            if request.is_json:
                data.update(request.get_json(silent=True) or {})
            if request.args:
                data.update(request.args.to_dict())
            if request.form:
                data.update(request.form.to_dict())

            if node.status.get('e_stop', False):
                node.publish_twist(0.0, 0.0)
                return jsonify(ok=False, error="e_stop_active"), 403

            linear_x, angular_z = parse_twist_payload(data, speed_scale=node.status.get('speed_scale', 1.0))
            node.publish_twist(linear_x, angular_z)
            return jsonify(ok=True, linear_x=linear_x, angular_z=angular_z), 200
        except Exception as e:
            node.get_logger().error(f"Error handling cmd_vel HTTP request: {e}")
            return jsonify(ok=False, error=str(e)), 500

    # Supported Emergency Stop HTTP endpoints across all app versions
    @app.route('/e_stop', methods=['POST'])
    @app.route('/estop', methods=['POST'])
    @app.route('/stop', methods=['POST'])
    @app.route('/cancel', methods=['POST'])
    @app.route('/api/stop', methods=['POST'])
    @app.route('/api/estop', methods=['POST'])
    @app.route('/api/e_stop', methods=['POST'])
    def post_e_stop():
        try:
            node.trigger_e_stop()
            broadcast('/rover/status', node.get_status_response())
            return jsonify(ok=True, status="stopped", nav2_active=node.is_nav2_active()), 200
        except Exception as e:
            node.get_logger().error(f"Error handling post_e_stop: {e}")
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/clear_estop', methods=['POST'])
    @app.route('/resume', methods=['POST'])
    @app.route('/api/resume', methods=['POST'])
    def post_clear_estop():
        try:
            with node.lock:
                node.status['e_stop'] = False
                node.cancel_requested = False
            # If Nav2 was stopped or died, auto-restore Nav2 in background
            if not node.is_nav2_active():
                node.get_logger().info("[BridgeNode] Resume triggered and Nav2 inactive, auto-restoring Nav2...")
                threading.Thread(target=node.ensure_nav2_active, daemon=True).start()
            broadcast('/rover/status', node.get_status_response())
            return jsonify(ok=True, status="cleared", nav2_active=node.is_nav2_active()), 200
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/restart_nav', methods=['POST'])
    @app.route('/restart_nav2', methods=['POST'])
    @app.route('/api/restart_nav', methods=['POST'])
    @app.route('/api/restart_nav2', methods=['POST'])
    def post_restart_nav():
        try:
            success = node.restart_nav2()
            broadcast('/rover/status', node.get_status_response())
            return jsonify(ok=success, nav2_active=node.is_nav2_active()), (200 if success else 500)
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/start_mapping', methods=['POST'])
    def start_mapping():
        try:
            success = node.cleaner.start_mapping()
            return jsonify(status='ok' if success else 'error', state=node.cleaner.state.name)
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/save_map', methods=['POST'])
    def save_map():
        try:
            success = node.cleaner.save_map()
            return jsonify(status='ok' if success else 'error', state=node.cleaner.state.name)
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/map_image', methods=['GET'])
    def map_image():
        try:
            png_bytes = map_pgm_to_png_bytes(node.cleaner.map_yaml)
            if not png_bytes:
                return jsonify(error='Map image not found or invalid'), 404
            return Response(png_bytes, mimetype='image/png')
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @sock.route('/ws/rover')
    def rover_ws(ws):
        conn = {'ws': ws, 'subs': set()}
        with connections_lock:
            connections.append(conn)
        node.get_logger().info('app connected')
        try:
            while True:
                data = ws.receive()
                if data is None:
                    break
                handle_message(node, conn, data)
        finally:
            with connections_lock:
                if conn in connections:
                    connections.remove(conn)
            node.get_logger().info('app disconnected')

    return app


def telemetry_loop(node: BridgeNode):
    while rclpy.ok():
        try:
            broadcast('/rover/telemetry', node.get_telemetry())
            broadcast('/rover/status', node.get_status_response())
        except Exception:
            pass
        time.sleep(1.0)


def main():
    rclpy.init()
    node = BridgeNode()
    flask_app = make_flask_app(node)

    def run_flask():
        flask_app.run(host='0.0.0.0', port=8000, threaded=True)

    threading.Thread(target=run_flask, daemon=True).start()
    threading.Thread(target=telemetry_loop, args=(node,), daemon=True).start()
    threading.Thread(target=node.estop_guard_loop, daemon=True).start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
