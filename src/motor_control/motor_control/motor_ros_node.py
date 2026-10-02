import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool
from rcl_interfaces.msg import SetParametersResult
from motor_control import motor_node as mc  # your library file

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

        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.sub_estop = self.create_subscription(
            Empty, '/estop', self.estop_cb, 10)
        self.sub_turn_mode = self.create_subscription(
            Bool, '/single_side_turn', self.turn_mode_cb, 10)
        self.add_on_set_parameters_callback(self.param_cb)

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_cb)
        self.get_logger().info(f'motor_node up. Listening /cmd_vel, /estop, /single_side_turn (default single_side_turn={self.single_side_turn})')

    def turn_mode_cb(self, msg: Bool):
        self.single_side_turn = bool(msg.data)
        mc.set_single_side_turn(self.single_side_turn)
        mode_str = "SINGLE-SIDE PIVOT" if self.single_side_turn else "TANK TURN (DUAL-SIDE ZERO-RADIUS SPIN)"
        self.get_logger().info(f"Turn mode switched to: {mode_str}")

    def param_cb(self, params):
        for p in params:
            if p.name == 'single_side_turn':
                self.single_side_turn = bool(p.value)
                mc.set_single_side_turn(self.single_side_turn)
            elif p.name == 'max_linear':
                self.max_linear = float(p.value)
            elif p.name == 'max_angular':
                self.max_angular = float(p.value)
        return SetParametersResult(successful=True)

    def cmd_vel_cb(self, msg: Twist):
        throttle = max(-1.0, min(1.0, msg.linear.x / self.max_linear))
        steer = max(-1.0, min(1.0, msg.angular.z / self.max_angular))
        mc.drive(throttle, steer)
        self.last_cmd_time = self.get_clock().now()

    def estop_cb(self, msg: Empty):
        self.get_logger().warn('ESTOP received — immediate stop')
        mc.stop_all(immediate=True)

    def watchdog_cb(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            mc.stop_all()

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
