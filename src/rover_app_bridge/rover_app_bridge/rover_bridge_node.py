import json
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from flask import Flask, jsonify
from flask_sock import Sock


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class BridgeNode(Node):
    def __init__(self):
        super().__init__('rover_bridge_node')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('rover_bridge_node ready, publishing /cmd_vel')

        # Internal state, echoed back to app as /rover/status.
        # Nothing here needs real ROS2 messages -- it's only ever
        # set BY the app and read back BY the app.
        self.lock = threading.Lock()
        self.status = {
            'e_stop': False,
            'auto_clean_active': False,
            'cycles': 3,
            'speed_scale': 0.6,
        }

        # Telemetry stub -- replace with real subscriptions once you
        # have sensor/battery/LIDAR nodes publishing this data.
        self.telemetry = {
            'battery': 82,
            'signal': 'Strong',
            'operation_time': '1h 15m',
            'zone': 'Sector 4-B',
            'diagnostic_ok': True,
        }

    def publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub_cmd_vel.publish(msg)

    def get_status(self):
        with self.lock:
            return dict(self.status)

    def get_telemetry(self):
        with self.lock:
            return dict(self.telemetry)

    def set_estop(self, value: bool):
        with self.lock:
            self.status['e_stop'] = bool(value)
        if value:
            self.publish_twist(0.0, 0.0)

    def set_speed_scale(self, scale: float):
        with self.lock:
            self.status['speed_scale'] = _clamp(float(scale), 0.0, 1.0)

    def set_auto_clean(self, active: bool, cycles: int):
        with self.lock:
            self.status['auto_clean_active'] = bool(active)
            self.status['cycles'] = int(cycles)

    def start_snake_pattern(self, xmin: float, xmax: float, ymin: float, ymax: float, step_x: float = 0.5, step_y: float = 0.5):
        with self.lock:
            if self.status['auto_clean_active']:
                self.get_logger().warn('Auto-clean/snake pattern is already running!')
                return
            self.status['auto_clean_active'] = True

        def _run():
            try:
                from motor_control.snake_pattern_navigator import generate_snake_pattern, execute_navigation
                waypoints = generate_snake_pattern(xmin, xmax, ymin, ymax, step_x, step_y)
                self.get_logger().info(f"Generated {len(waypoints)} waypoints for snake pattern. Executing via Nav2...")
                execute_navigation(waypoints)
            except Exception as e:
                self.get_logger().error(f"Snake pattern execution error: {e}")
            finally:
                with self.lock:
                    self.status['auto_clean_active'] = False
                broadcast('/rover/status', self.get_status())

        threading.Thread(target=_run, daemon=True).start()


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
            # send an immediate snapshot so the app doesn't wait for the next tick
            if topic == '/rover/status':
                conn['ws'].send(json.dumps({
                    'op': 'publish', 'topic': '/rover/status', 'msg': node.get_status()
                }))
            elif topic == '/rover/telemetry':
                conn['ws'].send(json.dumps({
                    'op': 'publish', 'topic': '/rover/telemetry', 'msg': node.get_telemetry()
                }))
        return

    if op == 'ping':
        return

    if op != 'publish':
        return

    topic = data.get('topic')
    msg = data.get('msg') or {}

    if topic == '/cmd_vel':
        linear = msg.get('linear', {}) or {}
        angular = msg.get('angular', {}) or {}
        node.publish_twist(
            float(linear.get('x', 0.0)),
            float(angular.get('z', 0.0)),
        )

    elif topic == '/rover/e_stop':
        node.set_estop(bool(msg.get('data', False)))
        broadcast('/rover/status', node.get_status())

    elif topic == '/rover/speed_scale':
        node.set_speed_scale(float(msg.get('data', 0.6)))
        broadcast('/rover/status', node.get_status())

    elif topic == '/rover/auto_clean':
        active = bool(msg.get('active', False))
        if active:
            xmin = float(msg.get('xmin', 0.0))
            xmax = float(msg.get('xmax', 4.0))
            ymin = float(msg.get('ymin', 0.0))
            ymax = float(msg.get('ymax', 3.0))
            step_x = float(msg.get('step_x', 0.5))
            step_y = float(msg.get('step_y', 0.5))
            node.start_snake_pattern(xmin, xmax, ymin, ymax, step_x, step_y)
        else:
            node.set_auto_clean(False, 0)
        broadcast('/rover/status', node.get_status())

    elif topic == '/rover/snake_pattern':
        cmd = msg.get('command', 'start')
        if cmd == 'start':
            xmin = float(msg.get('xmin', 0.0))
            xmax = float(msg.get('xmax', 4.0))
            ymin = float(msg.get('ymin', 0.0))
            ymax = float(msg.get('ymax', 3.0))
            step_x = float(msg.get('step_x', 0.5))
            step_y = float(msg.get('step_y', 0.5))
            node.start_snake_pattern(xmin, xmax, ymin, ymax, step_x, step_y)
        broadcast('/rover/status', node.get_status())


def make_flask_app(node: BridgeNode) -> Flask:
    app = Flask(__name__)
    sock = Sock(app)

    @app.route('/health')
    def health():
        return jsonify(status='ok')

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
    # Periodic telemetry push. Replace node.get_telemetry() internals with
    # real sensor subscriptions once available -- this loop just needs to
    # keep calling broadcast() at whatever rate you want.
    while rclpy.ok():
        broadcast('/rover/telemetry', node.get_telemetry())
        time.sleep(2.0)


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

