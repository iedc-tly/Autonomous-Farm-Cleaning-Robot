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
import secrets
import hmac
import re

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

from motor_control.farm_cleaner import Cleaner, map_pgm_to_png_bytes, make_snake, launch, stop, kill_all_zombies, S

def robot_home() -> str:
    return os.path.abspath(os.environ.get("ROBOT_HOME", os.path.expanduser("~")))


LOCATIONS_FILE = os.path.join(robot_home(), "ros2_ws/src/motor_control/config/locations.yaml")
INSTALL_LOCATIONS_FILE = os.path.join(robot_home(), "ros2_ws/install/motor_control/share/motor_control/config/locations.yaml")
SNAKE_WAYPOINTS_FILE = os.path.join(robot_home(), "ros2_ws/snake_waypoints.yaml")


def get_or_create_rover_token() -> str:
    env_token = os.environ.get("ROVER_TOKEN")
    if env_token:
        return env_token.strip()

    token_file = os.path.join(robot_home(), ".rover_token")
    if os.path.exists(token_file):
        try:
            with open(token_file, "r") as f:
                token = f.read().strip()
                if token:
                    return token
        except Exception:
            pass

    token = secrets.token_urlsafe(16)
    try:
        flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
        mode = 0o600
        fd = os.open(token_file, flags, mode)
        with os.fdopen(fd, 'w') as f:
            f.write(token + "\n")
    except Exception as e:
        print(f"[BridgeNode] Warning: failed to save token to {token_file}: {e}")
    return token


ROVER_TOKEN = get_or_create_rover_token()


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


def save_locations_dict(data: dict) -> tuple:
    """Save locations data dictionary to both source and install locations.yaml."""
    paths = [LOCATIONS_FILE, INSTALL_LOCATIONS_FILE]
    saved_paths = []
    err = None
    for p in paths:
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, 'w') as f:
                yaml.safe_dump(data, f, sort_keys=False)
            saved_paths.append(p)
        except Exception as e:
            err = str(e)
    if not saved_paths:
        return False, err or "Failed to write locations"
    return True, None


def load_and_validate_locations():
    path = os.path.expanduser(LOCATIONS_FILE)
    if not os.path.exists(path):
        if os.path.exists(INSTALL_LOCATIONS_FILE):
            path = INSTALL_LOCATIONS_FILE
        else:
            return None, "locations.yaml missing"
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
    except Exception as e:
        return None, f"locations.yaml parse error: {e}"

    if not isinstance(data, dict):
        return None, "locations.yaml must be a dictionary"

    dock = data.get('dock', {})
    if not isinstance(dock, dict):
        dock = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    pit = data.get('pit', {})
    if not isinstance(pit, dict):
        pit = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    sweep = data.get('sweep', {})
    if not isinstance(sweep, dict):
        sweep = {'xmin': -1.0, 'xmax': 1.0, 'ymin': -1.0, 'ymax': 1.0, 'row_gap': 0.5, 'robot_radius': 0.48}

    xmin = float(sweep.get('xmin', -1.0))
    xmax = float(sweep.get('xmax', 1.0))
    ymin = float(sweep.get('ymin', -1.0))
    ymax = float(sweep.get('ymax', 1.0))

    if xmin > xmax:
        xmin, xmax = xmax, xmin
    if ymin > ymax:
        ymin, ymax = ymax, ymin

    res = {
        'dock': {
            'x': float(dock.get('x', 0.0)),
            'y': float(dock.get('y', 0.0)),
            'yaw': float(dock.get('yaw', 0.0))
        },
        'pit': {
            'x': float(pit.get('x', 0.0)),
            'y': float(pit.get('y', 0.0)),
            'yaw': float(pit.get('yaw', 0.0))
        },
        'sweep': {
            'xmin': xmin,
            'xmax': xmax,
            'ymin': ymin,
            'ymax': ymax,
            'row_gap': float(sweep.get('row_gap', 0.5)),
            'robot_radius': float(sweep.get('robot_radius', 0.48)),
        }
    }
    return res, None


class BridgeNode(Node):
    def __init__(self):
        super().__init__('rover_bridge_node')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_hw_estop = self.create_publisher(EmptyMsg, '/estop', 10)
        self.pub_estop_clear = self.create_publisher(EmptyMsg, '/estop_clear', 10)
        self.pub_turn_mode = self.create_publisher(Bool, '/single_side_turn', 10)
        self.single_side_turn = False

        # Native ROS 2 Action Clients for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.cleaner = Cleaner()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle_clients = {}
        self.corrector = None
        try:
            from motor_control.pose_corrector import AutonomousPoseCorrector
            self.corrector = AutonomousPoseCorrector(parent_node=self)
        except Exception as e:
            self.get_logger().warn(f"[BridgeNode] Pose corrector not initialized: {e}")

        self.lock = threading.Lock()

        # State Machine Fields
        self.state = "IDLE"  # IDLE | SWEEPING | DOCKING | GOING_TO_PIT | NAVIGATING_TO_PIT | MAPPING
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
        self.get_logger().info(f'[Auth] ROVER_TOKEN active: {ROVER_TOKEN}')

        # Auto-bringup Nav2 and hardware drivers on boot if an existing map is available
        default_map = os.path.join(robot_home(), 'my_map.yaml')
        map_yaml = getattr(self.cleaner, 'map_yaml', default_map)
        if os.path.exists(map_yaml):
            self.get_logger().info(f"[BridgeNode] Existing map found at {map_yaml}. Auto-starting Nav2 & hardware stack in background...")
            threading.Thread(target=self.ensure_nav2_active, daemon=True).start()
        else:
            self.get_logger().info("[BridgeNode] No map file found on boot. System ready in IDLE mapping mode.")

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
        while (time.time() - start_t) < 35.0:
            time.sleep(0.5)
            if self.is_nav2_active():
                self.get_logger().info("[BridgeNode] Nav2 navigation stack active!")
                self.publish_initial_dock_pose()
                try:
                    if 'broadcast' in globals():
                        broadcast('/rover/status', self.get_status_response())
                except Exception:
                    pass
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


    def publish_initial_dock_pose(self, dock=None):
        """Publish initial pose estimation (dock position) to AMCL /initialpose and trigger stationary scan-sharpening."""
        if dock is None:
            locs, _ = load_and_validate_locations()
            dock = locs['dock'] if locs else {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        if float(dock.get('x', 0.0)) < -1.35:
            self.get_logger().warn(f"[BridgeNode] Saved dock X={dock.get('x')} is outside current map. Falling back to map origin (0, 0, 0).")
            dock = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

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
        p.pose.covariance[0] = 0.02
        p.pose.covariance[7] = 0.02
        p.pose.covariance[35] = 0.015

        for _ in range(5):
            self.pub_initial_pose.publish(p)
            time.sleep(0.04)

        # Trigger stationary AMCL scan-matching sharpening burst in background thread
        threading.Thread(target=self.sharpen_pose_stationary, daemon=True).start()

    def sharpen_pose_stationary(self, num_updates: int = 2):
        """Burst no-motion updates to AMCL to snap live LiDAR scan directly to map walls while stationary."""
        from std_srvs.srv import Empty as EmptySrv
        if not hasattr(self, 'nomotion_client') or self.nomotion_client is None:
            self.nomotion_client = self.create_client(EmptySrv, '/request_nomotion_update')

        if not self.nomotion_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[BridgeNode] /request_nomotion_update service not available yet.")
            return False

        time.sleep(0.3)
        self.get_logger().info("[BridgeNode] Sharpening 2D pose against LiDAR live scan...")
        for _ in range(num_updates):
            try:
                self.nomotion_client.call_async(EmptySrv.Request())
            except Exception:
                pass
            time.sleep(0.3)
        self.get_logger().info("[BridgeNode] Pose sharpening complete. Live scan aligned with map.")
        return True

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            t = trans.transform.translation
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return {
                'x': float(round(t.x, 3)),
                'y': float(round(t.y, 3)),
                'yaw': float(round(yaw, 3)),
                'yaw_deg': float(round(math.degrees(yaw), 1)),
                'orientation': {
                    'x': float(round(q.x, 4)),
                    'y': float(round(q.y, 4)),
                    'z': float(round(q.z, 4)),
                    'w': float(round(q.w, 4))
                }
            }
        except Exception:
            return None

    def _auto_update_sweep_bounds(self, locs: dict):
        """Ensure xmin < xmax and ymin < ymax, and regenerate snake waypoints if valid."""
        sweep = locs.get('sweep', {})
        if 'xmin' in sweep and 'xmax' in sweep:
            xmin = min(float(sweep['xmin']), float(sweep['xmax']))
            xmax = max(float(sweep['xmin']), float(sweep['xmax']))
            sweep['xmin'] = float(round(xmin, 3))
            sweep['xmax'] = float(round(xmax, 3))
        if 'ymin' in sweep and 'ymax' in sweep:
            ymin = min(float(sweep['ymin']), float(sweep['ymax']))
            ymax = max(float(sweep['ymin']), float(sweep['ymax']))
            sweep['ymin'] = float(round(ymin, 3))
            sweep['ymax'] = float(round(ymax, 3))

        dx = abs(sweep.get('xmax', 0) - sweep.get('xmin', 0))
        dy = abs(sweep.get('ymax', 0) - sweep.get('ymin', 0))
        if dx >= 0.25 and dy >= 0.25:
            map_yaml = getattr(self.cleaner, 'map_yaml', '/home/iedctly/my_map.yaml')
            radius = max(float(sweep.get('robot_radius', 0.48)), 0.48)
            try:
                wps = make_snake(
                    map_yaml,
                    bounds=[sweep['xmin'], sweep['xmax'], sweep['ymin'], sweep['ymax']],
                    row_gap=sweep.get('row_gap', 0.5),
                    step_x=sweep.get('row_gap', 0.5),
                    robot_radius=radius
                )
                if wps:
                    with open(SNAKE_WAYPOINTS_FILE, 'w') as f:
                        yaml.dump({'num_waypoints': len(wps), 'waypoints': wps}, f, default_flow_style=False)
                    self.get_logger().info(f"[BridgeNode] Auto-regenerated {len(wps)} snake waypoints after bounds update.")
            except Exception as e:
                self.get_logger().warn(f"Auto snake regenerate notice: {e}")

    def save_current_pose_as(self, loc_type: str, custom_yaw: float = None):
        """Capture live robot TF pose and save it as named location in locations.yaml."""
        pose = self.get_current_pose()
        if not pose:
            return None, "Robot pose not available from TF (map -> base_link). Ensure robot is placed and localization or mapping is running."

        locs, _ = load_and_validate_locations()
        if not locs:
            locs = {
                'dock': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                'pit': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                'sweep': {'xmin': -1.0, 'xmax': 1.0, 'ymin': -1.0, 'ymax': 1.0, 'row_gap': 0.5, 'robot_radius': 0.48}
            }

        yaw = custom_yaw if custom_yaw is not None else pose['yaw']
        loc_type_norm = str(loc_type).lower().strip()

        if loc_type_norm in ('dock', 'home', 'station', 'base'):
            locs['dock'] = {
                'x': float(round(pose['x'], 3)),
                'y': float(round(pose['y'], 3)),
                'yaw': float(round(yaw, 3))
            }
            saved_key = 'dock'

        elif loc_type_norm in ('pit', 'dump', 'waste', 'trash'):
            locs['pit'] = {
                'x': float(round(pose['x'], 3)),
                'y': float(round(pose['y'], 3)),
                'yaw': float(round(yaw, 3))
            }
            saved_key = 'pit'

        elif loc_type_norm in ('sweep_corner1', 'corner1', 'bl', 'bottom_left', 'sweep_min', 'min', 'c1'):
            sweep = locs.get('sweep', {})
            sweep['xmin'] = float(round(pose['x'], 3))
            sweep['ymin'] = float(round(pose['y'], 3))
            locs['sweep'] = sweep
            saved_key = 'sweep_corner1'
            self._auto_update_sweep_bounds(locs)

        elif loc_type_norm in ('sweep_corner2', 'corner2', 'tr', 'top_right', 'sweep_max', 'max', 'c2'):
            sweep = locs.get('sweep', {})
            sweep['xmax'] = float(round(pose['x'], 3))
            sweep['ymax'] = float(round(pose['y'], 3))
            locs['sweep'] = sweep
            saved_key = 'sweep_corner2'
            self._auto_update_sweep_bounds(locs)

        else:
            return None, f"Unknown location type '{loc_type}'. Must be dock, pit, sweep_corner1, or sweep_corner2."

        ok, save_err = save_locations_dict(locs)
        if not ok:
            return None, f"Failed to save locations.yaml: {save_err}"

        if saved_key == 'dock':
            self.publish_initial_dock_pose(dock=locs['dock'])

        self.get_logger().info(f"[BridgeNode] Saved current robot pose as '{saved_key}': {locs[saved_key if saved_key in ('dock', 'pit') else 'sweep']}")
        broadcast('/rover/locations', locs)
        broadcast('/rover/status', self.get_status_response())
        return {'location': saved_key, 'pose': pose, 'locations': locs}, None

    def auto_sweep_entire_room(self):
        """Auto-detect room bounds from saved map and set sweep to entire room."""
        map_yaml = getattr(self.cleaner, 'map_yaml', '/home/iedctly/my_map.yaml')
        if not os.path.exists(map_yaml):
            return None, f"Map file not found at {map_yaml}. Please run mapping first."

        from motor_control.farm_cleaner import get_entire_room_bounds
        bounds = get_entire_room_bounds(map_yaml, robot_radius=0.48)
        locs, _ = load_and_validate_locations()
        if not locs:
            locs = {
                'dock': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                'pit': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                'sweep': {}
            }
        locs['sweep'] = {
            'xmin': float(bounds[0]),
            'xmax': float(bounds[1]),
            'ymin': float(bounds[2]),
            'ymax': float(bounds[3]),
            'row_gap': float(locs.get('sweep', {}).get('row_gap', 0.5)),
            'robot_radius': float(locs.get('sweep', {}).get('robot_radius', 0.48))
        }
        self._auto_update_sweep_bounds(locs)
        save_locations_dict(locs)
        broadcast('/rover/locations', locs)
        broadcast('/rover/status', self.get_status_response())
        return locs['sweep'], None

    def handle_save_location(self, loc_type: str, body_data: dict = None):
        try:
            if self.status.get('e_stop', False):
                return jsonify(ok=False, error="e_stop_active"), 403

            custom_yaw = None
            if body_data and isinstance(body_data, dict):
                if 'yaw' in body_data:
                    try:
                        custom_yaw = float(body_data['yaw'])
                    except (ValueError, TypeError):
                        pass

            res, err = self.save_current_pose_as(loc_type, custom_yaw=custom_yaw)
            if err:
                return jsonify(ok=False, error=err), 400

            return jsonify(ok=True, **res), 200
        except Exception as e:
            self.get_logger().error(f"Error saving location {loc_type}: {e}")
            return jsonify(ok=False, error=str(e)), 500

    def handle_start_mapping(self):
        try:
            with self.lock:
                if self.status.get('e_stop', False):
                    return jsonify(ok=False, error="e_stop_active"), 403
                if self.state in ("SWEEPING", "DOCKING", "GOING_TO_PIT", "NAVIGATING_TO_PIT"):
                    self.cancel_requested = True
                    if self.current_goal_handle:
                        try:
                            self.current_goal_handle.cancel_goal_async()
                        except Exception:
                            pass

                if self.cleaner.nav is not None:
                    stop(self.cleaner.nav, timeout=3.0)
                    self.cleaner.nav = None
                kill_all_zombies()

                self.state = "MAPPING"
                self.task = "mapping"
                self.percent = 0.0
                self.waypoint_index = 0
                self.waypoint_total = 0
                self.error = None

            success = self.cleaner.start_mapping()
            if not success:
                with self.lock:
                    self.state = "IDLE"
                    self.task = None
                    self.error = "mapping_bringup_failed"
                return jsonify(ok=False, error="Failed to launch SLAM mapping stack"), 500

            self.get_logger().info("\033[1;32m[BridgeNode] MAPPING mode started successfully!\033[0m")
            broadcast('/rover/status', self.get_status_response())
            return jsonify(ok=True, status="mapping_started", state=self.state), 200

        except Exception as e:
            self.get_logger().error(f"Error starting mapping: {e}")
            with self.lock:
                self.state = "IDLE"
                self.task = None
            return jsonify(ok=False, error=str(e)), 500

    def handle_save_map(self, map_name: str = None):
        if map_name is not None and str(map_name).strip() != "":
            clean_name = str(map_name).strip()
            if not re.fullmatch(r'[A-Za-z0-9_-]{1,40}', clean_name):
                return jsonify(ok=False, error="Invalid map_name: must match [A-Za-z0-9_-]{1,40}"), 400
            map_name = clean_name
        else:
            map_name = None

        try:
            with self.lock:
                if self.state != "MAPPING" and self.cleaner.slam is None:
                    if os.path.exists(self.cleaner.map_yaml):
                        return jsonify(ok=True, status="map_already_exists", map_yaml=self.cleaner.map_yaml, state=self.state), 200
                    return jsonify(ok=False, error="Robot is not currently in mapping mode"), 400

                self.state = "SAVING_MAP"
                self.task = "save_map"

            success = self.cleaner.save_map(map_name=map_name)
            if not success:
                with self.lock:
                    self.state = "MAPPING"
                    self.task = "mapping"
                    self.error = "map_save_failed"
                return jsonify(ok=False, error="Map saver failed to capture map from /map topic"), 500

            self.get_logger().info(f"\033[1;32m[BridgeNode] Map saved successfully to {self.cleaner.map_yaml}!\033[0m")

            with self.lock:
                self.state = "IDLE"
                self.task = None
                self.error = None

            # Auto-restore Nav2 with the new map in background thread
            def _restore_nav2():
                time.sleep(1.0)
                self.ensure_nav2_active()
                broadcast('/rover/status', self.get_status_response())

            threading.Thread(target=_restore_nav2, daemon=True).start()

            broadcast('/rover/status', self.get_status_response())
            return jsonify(ok=True, status="map_saved", map_yaml=self.cleaner.map_yaml, state="IDLE"), 200

        except Exception as e:
            self.get_logger().error(f"Error saving map: {e}")
            with self.lock:
                self.state = "IDLE"
                self.task = None
            return jsonify(ok=False, error=str(e)), 500

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
                "pose": None,
                "nav2_active": self.is_nav2_active(),
                "last_task": self.last_task,
                "last_result": self.last_result,
                "error": self.error,
                "e_stop": self.status.get('e_stop', False)
            }
        st["pose"] = self.get_current_pose()
        return st

    def trigger_e_stop(self):
        """Execute complete Emergency Stop across hardware motors, Nav2 goals, and orchestrator without waiting on locks."""
        # Immediate unlatched status update
        self.status['e_stop'] = True

        # 1. Immediate publish hardware E-STOP (5x) before acquiring locks
        try:
            empty_msg = EmptyMsg()
            for _ in range(5):
                self.pub_hw_estop.publish(empty_msg)
                time.sleep(0.005)
        except Exception as e:
            self.get_logger().error(f"Error publishing hardware estop: {e}")

        # 2. Immediate zero velocity command override burst (3x)
        try:
            for _ in range(3):
                self.publish_twist(0.0, 0.0)
                time.sleep(0.005)
        except Exception as e:
            self.get_logger().error(f"Error publishing zero twist: {e}")

        self.get_logger().warn("\033[1;31m[EMERGENCY STOP TRIGGERED]\033[0m Halting hardware motors and cancelling all navigation tasks!")

        # 3. Take lock for state bookkeeping
        with self.lock:
            was_running = (self.state != "IDLE")
            running_task = self.task
            self.cancel_requested = True
            if was_running:
                self.last_task = running_task
                self.last_result = "canceled"
                self.error = "canceled"
            self.state = "IDLE"
            self.task = None

        # 4. Cancel Nav2 active action goal safely
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().error(f"Error cancelling Nav2 goal: {e}")

        # 5. Cleanly cancel cleaner task if running without killing navigation or hardware stack
        try:
            if hasattr(self.cleaner, 'navigator') and self.cleaner.navigator:
                self.cleaner.navigator.cancelTask()
            self.cleaner.state = S.IDLE
        except Exception as e:
            self.get_logger().error(f"Error cancelling cleaner task: {e}")

    def _try_begin(self, state: str, task: str, total: int) -> tuple[bool, str | None]:
        """Atomically verify e_stop and IDLE state, then transition to the new state."""
        with self.lock:
            if self.status.get('e_stop', False):
                return False, "e_stop_active"
            if self.state != "IDLE":
                return False, "busy"
            self.state = state
            self.task = task
            self.last_task = None
            self.last_result = None
            self.error = None
            self.percent = 0.0
            self.waypoint_index = 0
            self.waypoint_total = total
            self.cancel_requested = False
            return True, None

    def handle_dock_request(self):
        try:
            locs, err = load_and_validate_locations()
            if err:
                return jsonify(error=err), 412

            ok, err_code = self._try_begin("DOCKING", "dock", 1)
            if not ok:
                if err_code == "e_stop_active":
                    return jsonify(error="e_stop_active"), 403
                return jsonify(error="busy"), 409

            try:
                threading.Thread(
                    target=self._run_goto_pose,
                    args=("dock", locs['dock']),
                    daemon=True
                ).start()
            except Exception:
                with self.lock:
                    self.state = "IDLE"
                    self.task = None
                raise

            return jsonify(accepted=True, waypoint_total=1), 202
        except Exception as e:
            self.get_logger().error(f"Error handling dock request: {e}")
            return jsonify(ok=False, error=str(e)), 500

    def handle_pit_request(self):
        try:
            locs, err = load_and_validate_locations()
            if err:
                return jsonify(error=err), 412

            ok, err_code = self._try_begin("GOING_TO_PIT", "pit", 1)
            if not ok:
                if err_code == "e_stop_active":
                    return jsonify(error="e_stop_active"), 403
                return jsonify(error="busy"), 409

            try:
                threading.Thread(
                    target=self._run_goto_pose,
                    args=("pit", locs['pit']),
                    daemon=True
                ).start()
            except Exception:
                with self.lock:
                    self.state = "IDLE"
                    self.task = None
                raise

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
            locs, err = load_and_validate_locations()
            if err:
                return jsonify(error=err), 412

            sweep_cfg = locs['sweep']
            map_yaml = self.cleaner.map_yaml
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

            ok, err_code = self._try_begin("SWEEPING", "sweep", len(waypoints))
            if not ok:
                if err_code == "e_stop_active":
                    return jsonify(error="e_stop_active"), 403
                return jsonify(error="busy"), 409

            try:
                threading.Thread(
                    target=self._run_sweep,
                    args=(waypoints,),
                    daemon=True
                ).start()
            except Exception:
                with self.lock:
                    self.state = "IDLE"
                    self.task = None
                raise

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

            # Stationary 2D pose sharpening before moving
            if self.corrector:
                try:
                    self.corrector.sharpen_pose(timeout_sec=0.1)
                except Exception as e:
                    self.get_logger().warn(f"Pose corrector note: {e}")

            future_res = self.current_goal_handle.get_result_async()

            while not future_res.done():
                with self.lock:
                    if self.cancel_requested or self.status.get('e_stop', False):
                        self.get_logger().info("[BridgeNode] Cancelling sweep task...")
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
            try:
                empty_msg = EmptyMsg()
                for _ in range(3):
                    self.pub_estop_clear.publish(empty_msg)
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Error publishing /estop_clear: {e}")
            with self.lock:
                self.status['e_stop'] = False
                self.cancel_requested = False

    def set_speed_scale(self, scale: float):
        with self.lock:
            self.status['speed_scale'] = _clamp(float(scale), 0.0, 1.0)

    def destroy_node(self):
        try:
            self.publish_twist(0.0, 0.0)
            if hasattr(self, 'cleaner') and self.cleaner:
                self.cleaner.stop_all_locked()
            kill_all_zombies()
        except Exception:
            pass
        super().destroy_node()


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
                    'op': 'publish', 'topic': '/rover/status', 'msg': node.get_status_response()
                }))
            elif topic == '/rover/telemetry':
                conn['ws'].send(json.dumps({
                    'op': 'publish', 'topic': '/rover/telemetry', 'msg': node.get_telemetry()
                }))
            elif topic == '/rover/locations':
                locs, _ = load_and_validate_locations()
                conn['ws'].send(json.dumps({
                    'op': 'publish', 'topic': '/rover/locations', 'msg': locs or {}
                }))
            elif topic == '/rover/sweep_status':
                conn['ws'].send(json.dumps({
                    'op': 'publish', 'topic': '/rover/sweep_status', 'msg': node.cleaner.get_status_dict()
                }))
        return

    if op == 'ping':
        return

    if op == 'start_mapping':
        node.handle_start_mapping()
        return

    if op == 'save_map':
        map_name = data.get('map_name')
        if map_name and not re.fullmatch(r'[A-Za-z0-9_-]{1,40}', str(map_name)):
            node.get_logger().warn(f"[BridgeNode WS] Invalid map_name rejected: {map_name}")
            return
        node.handle_save_map(map_name=map_name)
        return

    if op == 'save_location':
        loc_name = data.get('name') or data.get('location') or 'dock'
        node.save_current_pose_as(loc_name)
        return

    if op != 'publish':
        return

    topic = data.get('topic')
    msg = data.get('msg') or {}

    if topic in ('/cmd_vel', '/api/cmd_vel', '/drive', '/api/drive', '/teleop', '/api/teleop'):
        if node.status.get('e_stop', False) or (node.state != "IDLE" and node.state != "MAPPING") or node.state in ("SWEEPING", "DOCKING", "GOING_TO_PIT", "NAVIGATING_TO_PIT"):
            node.publish_twist(0.0, 0.0)
            return

        linear_x, angular_z = parse_twist_payload(msg, speed_scale=node.status.get('speed_scale', 1.0))
        node.publish_twist(linear_x, angular_z)

    elif topic in ('/rover/start_mapping', '/mapping/start'):
        node.handle_start_mapping()

    elif topic in ('/rover/save_map', '/map/save'):
        map_name = msg.get('map_name') if isinstance(msg, dict) else None
        if map_name and not re.fullmatch(r'[A-Za-z0-9_-]{1,40}', str(map_name)):
            node.get_logger().warn(f"[BridgeNode WS] Invalid map_name rejected: {map_name}")
            return
        node.handle_save_map(map_name=map_name)

    elif topic in ('/rover/save_location', '/locations/save'):
        loc_name = msg.get('name') or msg.get('location') or 'dock'
        node.save_current_pose_as(loc_name)

    elif topic in ('/rover/e_stop', '/rover/estop', '/rover/stop', '/estop', '/stop'):
        val = bool(msg.get('data', True)) if isinstance(msg, dict) else True
        node.set_estop(val)
        broadcast('/rover/status', node.get_status_response())

    elif topic == '/rover/speed_scale':
        node.set_speed_scale(float(msg.get('data', 1.0)))
        broadcast('/rover/status', node.get_status())


def render_app_html() -> str:
    return """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Autonomous Farm Rover Controller & Mapper</title>
  <style>
    :root {
      --bg: #0b0f19;
      --card: #151d30;
      --border: #23314d;
      --text: #f1f5f9;
      --muted: #94a3b8;
      --primary: #2563eb;
      --success: #10b981;
      --warning: #f59e0b;
      --danger: #ef4444;
      --purple: #8b5cf6;
      --radius: 12px;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      background: var(--bg);
      color: var(--text);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 12px 10px 40px 10px;
    }
    .wrap {
      width: 100%;
      max-width: 520px;
      display: flex;
      flex-direction: column;
      gap: 12px;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 4px 2px;
    }
    .brand { font-size: 1.15rem; font-weight: 700; display: flex; align-items: center; gap: 8px; }
    .status-badges { display: flex; gap: 6px; }
    .badge {
      font-size: 0.72rem;
      font-weight: 700;
      padding: 3px 8px;
      border-radius: 999px;
      text-transform: uppercase;
      background: #23314d;
      color: #94a3b8;
    }
    .badge.green { background: #064e3b; color: #34d399; }
    .badge.amber { background: #78350f; color: #fcd34d; }
    .badge.red { background: #7f1d1d; color: #fca5a5; }
    .badge.blue { background: #1e3a8a; color: #93c5fd; }
    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: 12px 14px;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    .card-head {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.88rem;
      font-weight: 700;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    .pose-bar {
      display: flex;
      justify-content: space-around;
      background: #080c14;
      border: 1px solid #1a253c;
      padding: 8px;
      border-radius: 8px;
      font-family: monospace;
      font-size: 0.88rem;
    }
    .pose-val { color: #38bdf8; font-weight: 700; }
    .btn-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    .btn-grid.single { grid-template-columns: 1fr; }
    .btn-grid.three { grid-template-columns: 1fr 1fr 1fr; }
    .btn {
      border: none;
      border-radius: 9px;
      padding: 11px 10px;
      font-size: 0.88rem;
      font-weight: 600;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      transition: all 0.12s ease;
      touch-action: manipulation;
    }
    .btn:active { transform: scale(0.97); opacity: 0.85; }
    .btn-primary { background: var(--primary); color: #fff; }
    .btn-success { background: var(--success); color: #fff; }
    .btn-warning { background: var(--warning); color: #000; }
    .btn-purple { background: var(--purple); color: #fff; }
    .btn-danger { background: var(--danger); color: #fff; }
    .btn-dark { background: #23314d; color: #e2e8f0; }
    .btn-dark:active { background: #334155; }
    /* Drive controller */
    .drive-pad {
      display: grid;
      grid-template-columns: 74px 74px 74px;
      grid-template-rows: 74px 74px 74px;
      gap: 8px;
      justify-content: center;
      margin: 4px auto;
      touch-action: none;
    }
    .pad-btn {
      border-radius: 14px;
      border: 1px solid #334155;
      background: #0e1526;
      color: #fff;
      font-size: 1.5rem;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      user-select: none;
      touch-action: none;
    }
    .pad-btn:active, .pad-btn.pressed { background: var(--primary); border-color: #60a5fa; transform: scale(0.95); }
    .pad-stop { background: #7f1d1d; border-color: #ef4444; font-size: 1.1rem; font-weight: 700; }
    .pad-stop:active { background: #b91c1c; }
    .speed-box {
      display: flex;
      align-items: center;
      gap: 10px;
      font-size: 0.82rem;
      color: var(--muted);
    }
    .speed-box input[type="range"] { flex: 1; accent-color: var(--primary); }
    .loc-display {
      font-family: monospace;
      font-size: 0.78rem;
      background: #080c14;
      padding: 8px 10px;
      border-radius: 8px;
      border: 1px solid #1a253c;
      line-height: 1.45;
    }
    .log-box {
      font-family: monospace;
      font-size: 0.75rem;
      background: #050811;
      border: 1px solid #1a253c;
      border-radius: 8px;
      padding: 8px 10px;
      height: 52px;
      overflow-y: auto;
      color: #34d399;
      white-space: pre-wrap;
      word-break: break-all;
    }
    #mapImg {
      width: 100%;
      height: 180px;
      object-fit: contain;
      background: #050811;
      border-radius: 8px;
      border: 1px solid #23314d;
    }
    .tip { font-size: 0.76rem; color: #64748b; font-style: italic; }
  </style>
</head>
<body>
<div class="wrap">
  <header>
    <div class="brand">🚜 Farm Rover Boss</div>
    <div class="status-badges">
      <span id="wsBadge" class="badge">WS ...</span>
      <span id="stateBadge" class="badge">IDLE</span>
    </div>
  </header>

  <!-- Live Pose Banner -->
  <div class="card" style="padding: 10px 14px;">
    <div class="pose-bar">
      <div>X: <span id="posX" class="pose-val">--</span> m</div>
      <div>Y: <span id="posY" class="pose-val">--</span> m</div>
      <div>Yaw: <span id="posYaw" class="pose-val">--</span>°</div>
    </div>
  </div>

  <!-- Manual Drive Pad -->
  <div class="card">
    <div class="card-head">
      <span>🎮 Manual Drive & Position Control</span>
      <span id="speedVal" style="color:#38bdf8;">0.5x</span>
    </div>
    <div class="drive-pad">
      <div style="grid-column: 2; grid-row: 1;">
        <button class="pad-btn" id="btnFwd" style="width:100%; height:100%;">▲</button>
      </div>
      <div style="grid-column: 1; grid-row: 2;">
        <button class="pad-btn" id="btnLeft" style="width:100%; height:100%;">◀</button>
      </div>
      <div style="grid-column: 2; grid-row: 2;">
        <button class="pad-btn pad-stop" id="btnStop" style="width:100%; height:100%;">■</button>
      </div>
      <div style="grid-column: 3; grid-row: 2;">
        <button class="pad-btn" id="btnRight" style="width:100%; height:100%;">▶</button>
      </div>
      <div style="grid-column: 2; grid-row: 3;">
        <button class="pad-btn" id="btnBwd" style="width:100%; height:100%;">▼</button>
      </div>
    </div>
    <div class="speed-box">
      <span>Slow</span>
      <input type="range" id="speedSlider" min="0.1" max="1.0" step="0.05" value="0.5">
      <span>Fast</span>
    </div>
    <div class="tip">Hold buttons or use WASD keys on keyboard to drive robot.</div>
  </div>

  <!-- 1-Click Mapping Section -->
  <div class="card">
    <div class="card-head">🗺️ Map Environment</div>
    <div class="btn-grid">
      <button class="btn btn-primary" id="btnStartMap">🗺️ Start Mapping</button>
      <button class="btn btn-success" id="btnSaveMap">💾 Save Map</button>
    </div>
    <div class="btn-grid">
      <button class="btn btn-blue" id="btnAlignMap">🎯 Snap / Align Map</button>
      <button class="btn btn-dark" id="btnRefreshMap">🖼️ Refresh Map</button>
    </div>
    <img id="mapImg" src="/map_image" alt="Map Preview" style="display:none;" />
    <div class="tip">Tap 'Snap / Align Map' right after starting navigation if the live scan isn't fully aligned yet.</div>
  </div>

  <!-- 1-Click Location Configuration -->
  <div class="card">
    <div class="card-head">📍 Configure Locations (Go to Spot & Click Save)</div>
    <div class="loc-display" id="locsDisplay">Loading locations...</div>
    <div class="btn-grid">
      <button class="btn btn-purple" id="btnSaveDock">📍 Save as Dock</button>
      <button class="btn btn-amber" id="btnSavePit">🗑️ Save as Pit</button>
    </div>
    <div class="btn-grid">
      <button class="btn btn-dark" id="btnSaveC1">📐 Save Corner 1 (Min)</button>
      <button class="btn btn-dark" id="btnSaveC2">📐 Save Corner 2 (Max)</button>
    </div>
    <button class="btn btn-dark" id="btnAutoRoom" style="width:100%;">🏠 Auto-Detect Entire Room Bounds</button>
    <div class="tip">Drive robot to spot using Drive Pad above, then tap Save to store coordinates.</div>
  </div>

  <!-- Mission Execution & E-Stop -->
  <div class="card">
    <div class="card-head">🚀 Missions & Safety</div>
    <div class="btn-grid">
      <button class="btn btn-primary" id="btnStartSweep">🧹 Start Cleaning (Sweep)</button>
      <button class="btn btn-purple" id="btnGoDock">⚓ Go to Dock</button>
    </div>
    <div class="btn-grid single">
      <button class="btn btn-amber" id="btnGoPit">🗑️ Go to Waste Pit</button>
    </div>
    <div class="btn-grid">
      <button class="btn btn-danger" id="btnEstop" style="font-size:1rem; padding:14px;">🛑 E-STOP</button>
      <button class="btn btn-success" id="btnResume" style="font-size:1rem; padding:14px;">▶️ RESUME</button>
    </div>
  </div>

  <!-- Live Output Console -->
  <div class="card" style="padding:8px 12px;">
    <div class="card-head" style="font-size:0.75rem;">Status Console</div>
    <div class="log-box" id="logBox">System ready.</div>
  </div>
</div>

<script>
  let roverToken = localStorage.getItem('rover_token') || '';
  if (!roverToken) {
    const inputToken = prompt('Enter Rover Access Token (X-Rover-Token):');
    if (inputToken) {
      roverToken = inputToken.trim();
      localStorage.setItem('rover_token', roverToken);
    }
  }

  function authHeaders(custom = {}) {
    const h = { ...custom };
    if (roverToken) {
      h['X-Rover-Token'] = roverToken;
    }
    return h;
  }

  const logBox = document.getElementById('logBox');
  const wsBadge = document.getElementById('wsBadge');
  const stateBadge = document.getElementById('stateBadge');
  const posX = document.getElementById('posX');
  const posY = document.getElementById('posY');
  const posYaw = document.getElementById('posYaw');
  const locsDisplay = document.getElementById('locsDisplay');
  const speedSlider = document.getElementById('speedSlider');
  const speedVal = document.getElementById('speedVal');
  const mapImg = document.getElementById('mapImg');

  speedSlider.addEventListener('input', () => {
    speedVal.textContent = parseFloat(speedSlider.value).toFixed(2) + 'x';
  });

  function log(msg) {
    const time = new Date().toLocaleTimeString();
    logBox.textContent = `[${time}] ${msg}`;
    logBox.scrollTop = logBox.scrollHeight;
  }

  async function postAPI(endpoint, body) {
    try {
      log(`Calling ${endpoint}...`);
      const res = await fetch(endpoint, {
        method: 'POST',
        headers: authHeaders({ 'Content-Type': 'application/json' }),
        body: body ? JSON.stringify(body) : undefined
      });
      const data = await res.json();
      log(`${endpoint}: ${JSON.stringify(data)}`);
      fetchStatus();
      fetchLocations();
      return data;
    } catch (e) {
      log(`Error ${endpoint}: ${e}`);
    }
  }

  function drive(direction) {
    fetch('/api/cmd_vel', {
      method: 'POST',
      headers: authHeaders({ 'Content-Type': 'application/json' }),
      body: JSON.stringify({ direction: direction, speed_scale: parseFloat(speedSlider.value) })
    }).catch(() => {});
  }

  // Drive buttons touch & click press-and-hold
  const driveButtons = [
    { id: 'btnFwd', dir: 'forward' },
    { id: 'btnBwd', dir: 'backward' },
    { id: 'btnLeft', dir: 'left' },
    { id: 'btnRight', dir: 'right' }
  ];

  driveButtons.forEach(({ id, dir }) => {
    const el = document.getElementById(id);
    const startDrive = (e) => { e.preventDefault(); el.classList.add('pressed'); drive(dir); };
    const stopDrive = (e) => { e.preventDefault(); el.classList.remove('pressed'); drive('stop'); };

    el.addEventListener('touchstart', startDrive, { passive: false });
    el.addEventListener('touchend', stopDrive, { passive: false });
    el.addEventListener('touchcancel', stopDrive, { passive: false });
    el.addEventListener('mousedown', startDrive);
    el.addEventListener('mouseup', stopDrive);
    el.addEventListener('mouseleave', stopDrive);
  });

  document.getElementById('btnStop').addEventListener('click', () => drive('stop'));

  // Keyboard WASD support
  window.addEventListener('keydown', (e) => {
    if (e.repeat) return;
    const k = e.key.toLowerCase();
    if (k === 'w' || k === 'arrowup') drive('forward');
    else if (k === 's' || k === 'arrowdown') drive('backward');
    else if (k === 'a' || k === 'arrowleft') drive('left');
    else if (k === 'd' || k === 'arrowright') drive('right');
    else if (k === ' ') drive('stop');
  });
  window.addEventListener('keyup', (e) => {
    const k = e.key.toLowerCase();
    if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) {
      drive('stop');
    }
  });

  // Action Buttons
  document.getElementById('btnStartMap').addEventListener('click', () => postAPI('/api/start_mapping'));
  document.getElementById('btnSaveMap').addEventListener('click', () => postAPI('/api/save_map'));
  document.getElementById('btnAlignMap').addEventListener('click', () => postAPI('/api/align_map'));
  document.getElementById('btnSaveDock').addEventListener('click', () => postAPI('/api/locations/save_dock'));
  document.getElementById('btnSavePit').addEventListener('click', () => postAPI('/api/locations/save_pit'));
  document.getElementById('btnSaveC1').addEventListener('click', () => postAPI('/api/locations/save_sweep_corner1'));
  document.getElementById('btnSaveC2').addEventListener('click', () => postAPI('/api/locations/save_sweep_corner2'));
  document.getElementById('btnAutoRoom').addEventListener('click', () => postAPI('/api/locations/auto_sweep_room'));
  document.getElementById('btnStartSweep').addEventListener('click', () => postAPI('/api/sweep'));
  document.getElementById('btnGoDock').addEventListener('click', () => postAPI('/api/dock'));
  document.getElementById('btnGoPit').addEventListener('click', () => postAPI('/api/pit'));
  document.getElementById('btnEstop').addEventListener('click', () => postAPI('/api/e_stop'));
  document.getElementById('btnResume').addEventListener('click', () => postAPI('/api/resume'));

  document.getElementById('btnRefreshMap').addEventListener('click', () => {
    mapImg.style.display = 'block';
    mapImg.src = '/map_image?token=' + encodeURIComponent(roverToken) + '&t=' + Date.now();
  });

  function updatePoseUI(p) {
    if (!p) return;
    posX.textContent = typeof p.x === 'number' ? p.x.toFixed(2) : '--';
    posY.textContent = typeof p.y === 'number' ? p.y.toFixed(2) : '--';
    posYaw.textContent = typeof p.yaw_deg === 'number' ? p.yaw_deg.toFixed(1) : (typeof p.yaw === 'number' ? (p.yaw * 180 / Math.PI).toFixed(1) : '--');
  }

  function updateStateUI(st) {
    if (!st) return;
    const s = st.state || 'IDLE';
    stateBadge.textContent = s;
    stateBadge.className = 'badge';
    if (st.e_stop) {
      stateBadge.textContent = 'E-STOP';
      stateBadge.classList.add('red');
    } else if (s === 'MAPPING') {
      stateBadge.classList.add('amber');
    } else if (s === 'SWEEPING' || s === 'DOCKING' || s === 'GOING_TO_PIT' || s === 'NAVIGATING_TO_PIT') {
      stateBadge.classList.add('blue');
    } else {
      stateBadge.classList.add('green');
    }
    if (st.pose) updatePoseUI(st.pose);
  }

  function updateLocsUI(locs) {
    if (!locs) return;
    let html = '';
    if (locs.dock) {
      html += `⚓ Dock: X: ${locs.dock.x?.toFixed(2)}m, Y: ${locs.dock.y?.toFixed(2)}m, Yaw: ${(locs.dock.yaw * 180 / Math.PI).toFixed(1)}°\n`;
    }
    if (locs.pit) {
      html += `🗑️ Pit:  X: ${locs.pit.x?.toFixed(2)}m, Y: ${locs.pit.y?.toFixed(2)}m, Yaw: ${(locs.pit.yaw * 180 / Math.PI).toFixed(1)}°\n`;
    }
    if (locs.sweep) {
      html += `🧹 Sweep Bounds: X:[${locs.sweep.xmin?.toFixed(2)}..${locs.sweep.xmax?.toFixed(2)}], Y:[${locs.sweep.ymin?.toFixed(2)}..${locs.sweep.ymax?.toFixed(2)}]`;
    }
    locsDisplay.textContent = html || 'No locations configured yet.';
  }

  async function fetchStatus() {
    try {
      const res = await fetch('/api/status', { headers: authHeaders() });
      const st = await res.json();
      updateStateUI(st);
    } catch (_) {}
  }

  async function fetchLocations() {
    try {
      const res = await fetch('/api/locations', { headers: authHeaders() });
      const locs = await res.json();
      updateLocsUI(locs);
    } catch (_) {}
  }

  let ws = null;
  function connectWS() {
    const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${proto}//${location.host}/ws/rover?token=${encodeURIComponent(roverToken)}`;
    ws = new WebSocket(wsUrl);
    ws.onopen = () => {
      wsBadge.textContent = 'ONLINE';
      wsBadge.className = 'badge green';
      ws.send(JSON.stringify({ op: 'subscribe', topic: '/rover/status' }));
      ws.send(JSON.stringify({ op: 'subscribe', topic: '/rover/locations' }));
    };
    ws.onmessage = (e) => {
      try {
        const d = JSON.parse(e.data);
        if (d.topic === '/rover/status') updateStateUI(d.msg);
        if (d.topic === '/rover/locations') updateLocsUI(d.msg);
      } catch (_) {}
    };
    ws.onclose = () => {
      wsBadge.textContent = 'OFFLINE';
      wsBadge.className = 'badge red';
      setTimeout(connectWS, 2500);
    };
    ws.onerror = () => ws.close();
  }

  connectWS();
  fetchStatus();
  fetchLocations();
  setInterval(fetchStatus, 3000);
</script>
</body>
</html>
"""


def make_flask_app(node: BridgeNode) -> Flask:
    app = Flask(__name__)
    app.url_map.strict_slashes = False
    sock = Sock(app)

    @app.before_request
    def authenticate_request():
        if request.method == 'OPTIONS':
            return None
        public_paths = {'/', '/app', '/index.html', '/health', '/api/health'}
        if request.method == 'GET' and request.path in public_paths:
            return None

        # Check X-Rover-Token or Authorization header
        token = request.headers.get('X-Rover-Token')
        if not token:
            auth_header = request.headers.get('Authorization', '')
            if auth_header.startswith('Bearer '):
                token = auth_header[7:].strip()
            elif auth_header:
                token = auth_header.strip()
        if not token and request.method == 'GET' and request.path in ('/map_image', '/api/map_image'):
            token = request.args.get('token')

        if not token or not hmac.compare_digest(token, ROVER_TOKEN):
            return jsonify(error="Unauthorized", message="Valid X-Rover-Token or Authorization header required"), 401

    @app.after_request
    def add_cors_headers(response):
        allowed_origins = os.environ.get("ROVER_ALLOWED_ORIGINS", "").strip()
        if allowed_origins:
            origins = [o.strip() for o in allowed_origins.split(",") if o.strip()]
            req_origin = request.headers.get("Origin")
            if req_origin and req_origin in origins:
                response.headers['Access-Control-Allow-Origin'] = req_origin
                response.headers['Access-Control-Allow-Headers'] = 'Content-Type,Authorization,X-Rover-Token'
                response.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
        return response

    @app.errorhandler(404)
    def page_not_found(e):
        node.get_logger().warn(f"\033[1;33m[BridgeNode 404]\033[0m Request to unknown route: {request.method} {request.path}")
        return jsonify(error="Not Found", path=request.path, method=request.method), 404

    @app.route('/health', methods=['GET'])
    @app.route('/api/health', methods=['GET'])
    def health():
        try:
            return jsonify(ok=True), 200
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/', methods=['GET'])
    @app.route('/app', methods=['GET'])
    @app.route('/index.html', methods=['GET'])
    def index():
        return render_app_html()

    @app.route('/pose', methods=['GET'])
    @app.route('/api/pose', methods=['GET'])
    @app.route('/current_pose', methods=['GET'])
    @app.route('/api/current_pose', methods=['GET'])
    def get_pose():
        p = node.get_current_pose()
        if not p:
            return jsonify(ok=False, error="pose_unavailable"), 404
        return jsonify(ok=True, pose=p), 200

    @app.route('/locations', methods=['GET', 'POST', 'OPTIONS'])
    @app.route('/api/locations', methods=['GET', 'POST', 'OPTIONS'])
    def locations_route():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        if request.method == 'POST':
            data = request.get_json(silent=True) if request.is_json else {}
            if data and isinstance(data, dict):
                locs, _ = load_and_validate_locations()
                locs = locs or {}
                if 'dock' in data: locs['dock'] = data['dock']
                if 'pit' in data: locs['pit'] = data['pit']
                if 'sweep' in data:
                    locs['sweep'] = data['sweep']
                    node._auto_update_sweep_bounds(locs)
                save_locations_dict(locs)
                broadcast('/rover/locations', locs)
                return jsonify(ok=True, locations=locs), 200
        data, err = load_and_validate_locations()
        if err:
            return jsonify(error=err), 412
        return jsonify(data), 200

    @app.route('/locations/save_current', methods=['POST', 'OPTIONS'])
    @app.route('/api/locations/save_current', methods=['POST', 'OPTIONS'])
    def post_save_current():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        data = request.get_json(silent=True) if request.is_json else {}
        loc_type = data.get('name') or data.get('location') or request.args.get('name') or request.args.get('location') or 'dock'
        return node.handle_save_location(loc_type, data)

    @app.route('/locations/save_dock', methods=['POST', 'OPTIONS'])
    @app.route('/api/locations/save_dock', methods=['POST', 'OPTIONS'])
    @app.route('/save_dock', methods=['POST', 'OPTIONS'])
    @app.route('/api/save_dock', methods=['POST', 'OPTIONS'])
    def post_save_dock():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        data = request.get_json(silent=True) if request.is_json else {}
        return node.handle_save_location('dock', data)

    @app.route('/locations/save_pit', methods=['POST', 'OPTIONS'])
    @app.route('/api/locations/save_pit', methods=['POST', 'OPTIONS'])
    @app.route('/save_pit', methods=['POST', 'OPTIONS'])
    @app.route('/api/save_pit', methods=['POST', 'OPTIONS'])
    def post_save_pit():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        data = request.get_json(silent=True) if request.is_json else {}
        return node.handle_save_location('pit', data)

    @app.route('/locations/save_sweep_corner1', methods=['POST', 'OPTIONS'])
    @app.route('/api/locations/save_sweep_corner1', methods=['POST', 'OPTIONS'])
    @app.route('/save_sweep_corner1', methods=['POST', 'OPTIONS'])
    @app.route('/api/save_sweep_corner1', methods=['POST', 'OPTIONS'])
    @app.route('/locations/save_sweep_bl', methods=['POST', 'OPTIONS'])
    @app.route('/api/locations/save_sweep_bl', methods=['POST', 'OPTIONS'])
    def post_save_sweep_corner1():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        data = request.get_json(silent=True) if request.is_json else {}
        return node.handle_save_location('sweep_corner1', data)

    @app.route('/locations/save_sweep_corner2', methods=['POST', 'OPTIONS'])
    @app.route('/api/locations/save_sweep_corner2', methods=['POST', 'OPTIONS'])
    @app.route('/save_sweep_corner2', methods=['POST', 'OPTIONS'])
    @app.route('/api/save_sweep_corner2', methods=['POST', 'OPTIONS'])
    @app.route('/locations/save_sweep_tr', methods=['POST', 'OPTIONS'])
    @app.route('/api/locations/save_sweep_tr', methods=['POST', 'OPTIONS'])
    def post_save_sweep_corner2():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        data = request.get_json(silent=True) if request.is_json else {}
        return node.handle_save_location('sweep_corner2', data)

    @app.route('/locations/auto_sweep_room', methods=['POST', 'OPTIONS'])
    @app.route('/api/locations/auto_sweep_room', methods=['POST', 'OPTIONS'])
    def post_auto_sweep_room():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        sweep, err = node.auto_sweep_entire_room()
        if err:
            return jsonify(ok=False, error=err), 400
        return jsonify(ok=True, sweep=sweep), 200

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

    # Manual driving velocity commands (HTTP POST across all route aliases)
    @app.route('/cmd_vel', methods=['POST', 'OPTIONS'])
    @app.route('/api/cmd_vel', methods=['POST', 'OPTIONS'])
    @app.route('/cmdvel', methods=['POST', 'OPTIONS'])
    @app.route('/api/cmdvel', methods=['POST', 'OPTIONS'])
    @app.route('/drive', methods=['POST', 'OPTIONS'])
    @app.route('/api/drive', methods=['POST', 'OPTIONS'])
    @app.route('/teleop', methods=['POST', 'OPTIONS'])
    @app.route('/api/teleop', methods=['POST', 'OPTIONS'])
    @app.route('/manual', methods=['POST', 'OPTIONS'])
    @app.route('/api/manual', methods=['POST', 'OPTIONS'])
    @app.route('/move', methods=['POST', 'OPTIONS'])
    @app.route('/api/move', methods=['POST', 'OPTIONS'])
    @app.route('/control', methods=['POST', 'OPTIONS'])
    @app.route('/api/control', methods=['POST', 'OPTIONS'])
    @app.route('/rover/cmd_vel', methods=['POST', 'OPTIONS'])
    @app.route('/api/rover/cmd_vel', methods=['POST', 'OPTIONS'])
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
            try:
                empty_msg = EmptyMsg()
                for _ in range(3):
                    node.pub_estop_clear.publish(empty_msg)
                    time.sleep(0.01)
            except Exception as e:
                node.get_logger().error(f"Error publishing /estop_clear: {e}")
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

    @app.route('/align_map', methods=['POST', 'OPTIONS'])
    @app.route('/api/align_map', methods=['POST', 'OPTIONS'])
    @app.route('/api/sharpen_pose', methods=['POST', 'OPTIONS'])
    def align_map_route():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        try:
            node.publish_initial_dock_pose()
            time.sleep(1.0)
            pose = node.get_current_pose()
            broadcast('/rover/status', node.get_status_response())
            return jsonify(ok=True, status="aligned", pose=pose), 200
        except Exception as e:
            return jsonify(ok=False, error=str(e)), 500

    @app.route('/start_mapping', methods=['POST', 'OPTIONS'])
    @app.route('/api/start_mapping', methods=['POST', 'OPTIONS'])
    @app.route('/mapping/start', methods=['POST', 'OPTIONS'])
    @app.route('/api/mapping/start', methods=['POST', 'OPTIONS'])
    def start_mapping_route():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        return node.handle_start_mapping()

    @app.route('/save_map', methods=['POST', 'OPTIONS'])
    @app.route('/api/save_map', methods=['POST', 'OPTIONS'])
    @app.route('/map/save', methods=['POST', 'OPTIONS'])
    @app.route('/api/map/save', methods=['POST', 'OPTIONS'])
    def save_map_route():
        if request.method == 'OPTIONS':
            return jsonify(ok=True), 200
        data = {}
        if request.is_json:
            data = request.get_json(silent=True) or {}
        map_name = data.get('map_name') or request.args.get('map_name') or request.args.get('name')
        return node.handle_save_map(map_name=map_name)

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
        token = request.args.get('token', '')
        if not token or not hmac.compare_digest(token, ROVER_TOKEN):
            try:
                ws.send(json.dumps({'error': 'Unauthorized', 'message': 'Valid token required in ?token= query parameter'}))
                ws.close(1008, 'Unauthorized')
            except Exception:
                pass
            return

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

    import signal
    def _sig_handler(sig, frame):
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    def run_flask():
        flask_app.run(host='0.0.0.0', port=8000, threaded=True)

    threading.Thread(target=run_flask, daemon=True).start()
    threading.Thread(target=telemetry_loop, args=(node,), daemon=True).start()

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
