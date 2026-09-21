import json
import os
import math
import time
import threading
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
from lifecycle_msgs.srv import GetState
from flask import Flask, jsonify, request, Response
from flask_sock import Sock

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from motor_control.farm_cleaner import Cleaner, map_pgm_to_png_bytes, make_snake

LOCATIONS_FILE = "/home/iedctly/ros2_ws/src/motor_control/config/locations.yaml"


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_to_quaternion(yaw: float) -> dict:
    return {
        'x': 0.0,
        'y': 0.0,
        'z': float(round(math.sin(yaw / 2.0), 4)),
        'w': float(round(math.cos(yaw / 2.0), 4))
    }


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

        self.navigator = None
        self.cancel_requested = False

        self.status = {
            'e_stop': False,
            'auto_clean_active': False,
            'cycles': 3,
            'speed_scale': 0.6,
        }
        self.telemetry = {
            'battery': 82,
            'signal': 'Strong',
            'operation_time': '1h 15m',
            'zone': 'Sector 4-B',
            'diagnostic_ok': True,
        }
        self.get_logger().info('rover_bridge_node ready (3-button autonomous bridge initialized)!')

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

    def check_nav2_active(self) -> bool:
        required_nodes = ['bt_navigator', 'planner_server', 'controller_server']
        for node_name in required_nodes:
            if node_name not in self.lifecycle_clients:
                self.lifecycle_clients[node_name] = self.create_client(GetState, f'{node_name}/get_state')
            client = self.lifecycle_clients[node_name]
            if not client.service_is_ready():
                return False
            req = GetState.Request()
            future = client.call_async(req)
            start_t = time.time()
            while not future.done() and (time.time() - start_t) < 0.3:
                time.sleep(0.01)
            if not future.done() or future.result() is None:
                return False
            if future.result().current_state.label != 'active':
                return False
        return True

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
                "nav2_active": self.check_nav2_active(),
                "last_task": self.last_task,
                "last_result": self.last_result,
                "error": self.error
            }
            return st

    def trigger_e_stop(self):
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

        if self.navigator:
            try:
                self.navigator.cancelTask()
            except Exception:
                pass

        for _ in range(5):
            self.publish_twist(0.0, 0.0)
            time.sleep(0.05)

    def handle_dock_request(self):
        with self.lock:
            if self.state != "IDLE":
                return jsonify(error="busy"), 409

        locs, err = load_and_validate_locations()
        if err:
            return jsonify(error=err), 412

        if not self.check_nav2_active():
            return jsonify(error="nav2 not active"), 412

        with self.lock:
            self.state = "DOCKING"
            self.task = "dock"
            self.last_task = None
            self.last_result = None
            self.error = None
            self.percent = 0.0
            self.waypoint_index = 0
            self.waypoint_total = 0
            self.cancel_requested = False

        threading.Thread(
            target=self._run_goto_pose,
            args=("dock", locs['dock']),
            daemon=True
        ).start()

        return jsonify(accepted=True), 202

    def handle_pit_request(self):
        with self.lock:
            if self.state != "IDLE":
                return jsonify(error="busy"), 409

        locs, err = load_and_validate_locations()
        if err:
            return jsonify(error=err), 412

        if not self.check_nav2_active():
            return jsonify(error="nav2 not active"), 412

        with self.lock:
            self.state = "GOING_TO_PIT"
            self.task = "pit"
            self.last_task = None
            self.last_result = None
            self.error = None
            self.percent = 0.0
            self.waypoint_index = 0
            self.waypoint_total = 0
            self.cancel_requested = False

        threading.Thread(
            target=self._run_goto_pose,
            args=("pit", locs['pit']),
            daemon=True
        ).start()

        return jsonify(accepted=True), 202

    def handle_sweep_request(self):
        with self.lock:
            if self.state != "IDLE":
                return jsonify(error="busy"), 409

        locs, err = load_and_validate_locations()
        if err:
            return jsonify(error=err), 412

        if not self.check_nav2_active():
            return jsonify(error="nav2 not active"), 412

        sweep_cfg = locs['sweep']
        map_yaml = "/home/iedctly/my_map.yaml"
        waypoints = make_snake(
            map_yaml,
            bounds=[sweep_cfg['xmin'], sweep_cfg['xmax'], sweep_cfg['ymin'], sweep_cfg['ymax']],
            row_gap=sweep_cfg['row_gap'],
            step_x=sweep_cfg['row_gap'],
            robot_radius=sweep_cfg['robot_radius']
        )

        if not waypoints or len(waypoints) == 0:
            return jsonify(error="0 waypoints"), 412

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

    def _run_goto_pose(self, task_name, target):
        outcome = "failed"
        err = "navigation failed"
        try:
            if self.navigator is None:
                self.navigator = BasicNavigator('app_navigator')

            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.navigator.get_clock().now().to_msg()
            p.pose.position.x = float(target['x'])
            p.pose.position.y = float(target['y'])
            q = yaw_to_quaternion(float(target['yaw']))
            p.pose.orientation.x = q['x']
            p.pose.orientation.y = q['y']
            p.pose.orientation.z = q['z']
            p.pose.orientation.w = q['w']

            self.navigator.goToPose(p)

            while not self.navigator.isTaskComplete():
                with self.lock:
                    if self.cancel_requested:
                        self.navigator.cancelTask()
                        break
                time.sleep(0.1)

            with self.lock:
                if self.cancel_requested:
                    outcome = "canceled"
                    err = "canceled"
                else:
                    res = self.navigator.getResult()
                    if res == TaskResult.SUCCEEDED:
                        outcome = "succeeded"
                        err = None
                    elif res == TaskResult.CANCELED:
                        outcome = "canceled"
                        err = "canceled"
                    else:
                        outcome = "failed"
                        err = "navigation failed"
        except Exception as e:
            self.get_logger().error(f"Error in {task_name}: {e}")
            outcome = "failed"
            err = f"navigation failed: {e}"

        with self.lock:
            self.last_task = task_name
            self.last_result = outcome
            self.error = err
            self.state = "IDLE"
            self.task = None

    def _run_sweep(self, waypoints):
        outcome = "failed"
        err = "navigation failed"
        try:
            if self.navigator is None:
                self.navigator = BasicNavigator('app_navigator')

            pose_list = []
            for wp in waypoints:
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.header.stamp = self.navigator.get_clock().now().to_msg()
                p.pose.position.x = float(wp['x'])
                p.pose.position.y = float(wp['y'])
                q = wp['orientation']
                p.pose.orientation.x = q['x']
                p.pose.orientation.y = q['y']
                p.pose.orientation.z = q['z']
                p.pose.orientation.w = q['w']
                pose_list.append(p)

            self.navigator.followWaypoints(pose_list)

            while not self.navigator.isTaskComplete():
                with self.lock:
                    if self.cancel_requested:
                        self.navigator.cancelTask()
                        break

                feedback = self.navigator.getFeedback()
                if feedback:
                    cw = getattr(feedback, 'current_waypoint', None)
                    if cw is not None:
                        cur_idx = cw + 1
                        with self.lock:
                            if cur_idx > self.waypoint_index:
                                self.waypoint_index = cur_idx
                                if self.waypoint_total > 0:
                                    self.percent = min(100.0, float(round((self.waypoint_index / self.waypoint_total) * 100.0, 1)))
                time.sleep(0.2)

            with self.lock:
                if self.cancel_requested:
                    outcome = "canceled"
                    err = "canceled"
                else:
                    res = self.navigator.getResult()
                    if res == TaskResult.SUCCEEDED:
                        outcome = "succeeded"
                        err = None
                    elif res == TaskResult.CANCELED:
                        outcome = "canceled"
                        err = "canceled"
                    else:
                        outcome = "failed"
                        err = "navigation failed"
        except Exception as e:
            self.get_logger().error(f"Error in sweep: {e}")
            outcome = "failed"
            err = f"navigation failed: {e}"

        with self.lock:
            self.last_task = "sweep"
            self.last_result = outcome
            self.error = err
            self.state = "IDLE"
            self.task = None
            if outcome == "succeeded":
                self.percent = 100.0
                self.waypoint_index = self.waypoint_total

    # Legacy helper methods
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
        with self.lock:
            self.status['e_stop'] = bool(value)
        if value:
            self.trigger_e_stop()
            self.cleaner.e_stop()

    def set_speed_scale(self, scale: float):
        with self.lock:
            self.status['speed_scale'] = _clamp(float(scale), 0.0, 1.0)


# ---- connection registry (each WS client + its subscribed topics) ----
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

    if topic == '/cmd_vel':
        if node.state != "IDLE" or node.get_status().get('e_stop', False):
            node.publish_twist(0.0, 0.0)
            return

        linear = msg.get('linear', {}) or {}
        angular = msg.get('angular', {}) or {}
        node.publish_twist(
            float(linear.get('x', 0.0)),
            float(angular.get('z', 0.0)),
        )

    elif topic == '/rover/e_stop':
        val = bool(msg.get('data', False))
        if val:
            node.set_estop(True)
        broadcast('/rover/status', node.get_status())

    elif topic == '/rover/speed_scale':
        node.set_speed_scale(float(msg.get('data', 0.6)))
        broadcast('/rover/status', node.get_status())


def make_flask_app(node: BridgeNode) -> Flask:
    app = Flask(__name__)
    sock = Sock(app)

    @app.after_request
    def add_cors_headers(response):
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type,Authorization'
        response.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
        return response

    @app.route('/health', methods=['GET'])
    def health():
        return jsonify(ok=True), 200

    @app.route('/locations', methods=['GET'])
    def get_locations():
        data, err = load_and_validate_locations()
        if err:
            return jsonify(error=err), 412
        return jsonify(data), 200

    @app.route('/status', methods=['GET'])
    def get_status():
        st = node.get_status_response()
        return jsonify(st), 200

    @app.route('/dock', methods=['POST'])
    def post_dock():
        return node.handle_dock_request()

    @app.route('/pit', methods=['POST'])
    def post_pit():
        return node.handle_pit_request()

    @app.route('/sweep', methods=['POST'])
    def post_sweep():
        return node.handle_sweep_request()

    @app.route('/e_stop', methods=['POST'])
    def post_e_stop():
        node.trigger_e_stop()
        return jsonify(ok=True), 200

    @app.route('/start_mapping', methods=['POST'])
    def start_mapping():
        success = node.cleaner.start_mapping()
        return jsonify(status='ok' if success else 'error', state=node.cleaner.state.name)

    @app.route('/save_map', methods=['POST'])
    def save_map():
        success = node.cleaner.save_map()
        return jsonify(status='ok' if success else 'error', state=node.cleaner.state.name)

    @app.route('/map_image', methods=['GET'])
    def map_image():
        png_bytes = map_pgm_to_png_bytes(node.cleaner.map_yaml)
        if not png_bytes:
            return jsonify(error='Map image not found or invalid'), 404
        return Response(png_bytes, mimetype='image/png')

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
        broadcast('/rover/telemetry', node.get_telemetry())
        broadcast('/rover/status', node.get_status_response())
        time.sleep(1.0)


def main():
    rclpy.init()
    node = BridgeNode()
    flask_app = make_flask_app(node)

    def run_flask():
        flask_app.run(host='0.0.0.0', port=8000, threaded=True)

    threading.Thread(target=run_flask, daemon=True).start()
    threading.Thread(target=telemetry_loop, args=(node,), daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
