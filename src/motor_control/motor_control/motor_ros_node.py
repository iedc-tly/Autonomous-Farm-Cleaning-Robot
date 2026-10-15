import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from motor_control import motor_node as mc  # your library file


def robot_home() -> str:
    return os.path.abspath(os.environ.get("ROBOT_HOME", os.path.expanduser("~")))


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('cmd_timeout', 1.0)
        self.declare_parameter('single_side_turn', False)
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.single_side_turn = self.get_parameter('single_side_turn').value
        mc.set_single_side_turn(self.single_side_turn)
        self.last_cmd_time = self.get_clock().now()

        self.estopped = False
        self._moving = False

        # Transient local QoS to ensure late-joining nodes know current e-stop state
        estop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_estop_state = self.create_publisher(Bool, '/estop_state', estop_qos)
        self._publish_estop_state()

        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.sub_estop = self.create_subscription(
            Empty, '/estop', self.estop_cb, 10)
        self.sub_estop_clear = self.create_subscription(
            Empty, '/estop_clear', self.estop_clear_cb, 10)
        self.sub_turn_mode = self.create_subscription(
            Bool, '/single_side_turn', self.turn_mode_cb, 10)
        self.add_on_set_parameters_callback(self.param_cb)

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_cb)
        self.get_logger().info(f'motor_node up. Listening /cmd_vel, /estop, /estop_clear, /single_side_turn (default single_side_turn={self.single_side_turn})')

    def _publish_estop_state(self):
        msg = Bool()
        msg.data = bool(self.estopped)
        self.pub_estop_state.publish(msg)

    def turn_mode_cb(self, msg: Bool):
        self.single_side_turn = bool(msg.data)
        mc.set_single_side_turn(self.single_side_turn)
        mode_str = "SINGLE-SIDE PIVOT" if self.single_side_turn else "TANK TURN (DUAL-SIDE ZERO-RADIUS SPIN)"
        self.get_logger().info(f"Turn mode switched to: {mode_str}")

    def param_cb(self, params):
        for p in params:
            if p.name == 'max_linear':
                if float(p.value) <= 0.0:
                    return SetParametersResult(successful=False, reason="max_linear must be > 0")
                self.max_linear = float(p.value)
            elif p.name == 'max_angular':
                if float(p.value) <= 0.0:
                    return SetParametersResult(successful=False, reason="max_angular must be > 0")
                self.max_angular = float(p.value)
            elif p.name == 'single_side_turn':
                self.single_side_turn = bool(p.value)
                mc.set_single_side_turn(self.single_side_turn)
        return SetParametersResult(successful=True)

    def cmd_vel_cb(self, msg: Twist):
        if self.estopped:
            mc.stop_all()
            self._moving = False
            return

        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            self.get_logger().warn('Non-finite cmd_vel received — stopping motors')
            mc.stop_all()
            self._moving = False
            return

        throttle = max(-1.0, min(1.0, msg.linear.x / self.max_linear))
        steer = max(-1.0, min(1.0, msg.angular.z / self.max_angular))
        self._moving = (abs(throttle) > 1e-4 or abs(steer) > 1e-4)
        mc.drive(throttle, steer)
        self.last_cmd_time = self.get_clock().now()

    def estop_cb(self, msg: Empty):
        self.get_logger().warn('ESTOP received — immediate stop')
        self.estopped = True
        self._moving = False
        mc.stop_all(immediate=True)
        self._publish_estop_state()

    def estop_clear_cb(self, msg: Empty):
        self.get_logger().warn('ESTOP cleared — resuming normal operation')
        self.estopped = False
        self.last_cmd_time = self.get_clock().now()
        self._publish_estop_state()

    def watchdog_cb(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            if self._moving:
                mc.stop_all()
                self._moving = False

    def destroy_node(self):
        mc.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
