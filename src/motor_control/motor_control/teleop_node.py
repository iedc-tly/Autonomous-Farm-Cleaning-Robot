#!/usr/bin/env python3
"""
teleop_node.py

Simple and intuitive WASD / Arrow key teleop publisher for ROS 2.
Publishes geometry_msgs/msg/Twist to /cmd_vel.
"""

import os
import select
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
======================================================
           ROBOT KEYBOARD TELEOP CONTROL
======================================================
               [ W / Up ]    : Forward
 [ A / Left ]  [ S / Down ]  [ D / Right ] : Left / Back / Right
               [ Space / X ]: STOP

  Curved Motions:
    [ Q ] : Forward + Left    [ E ] : Forward + Right
    [ Z ] : Backward + Left   [ C ] : Backward + Right

  Speed Adjustment:
    [ + ] or [ = ] : Increase linear speed (+0.01 m/s)
    [ - ] or [ _ ] : Decrease linear speed (-0.01 m/s)
    [ ] ]          : Increase angular speed (+0.02 rad/s)
    [ [ ]          : Decrease angular speed (-0.02 rad/s)

  Press [ Ctrl+C ] to exit safely.
======================================================
"""

class SimpleTeleopNode(Node):
    def __init__(self):
        super().__init__('simple_teleop_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Speed settings
        self.linear_speed = 0.05  # m/s
        self.angular_speed = 0.15 # rad/s

        self.target_linear = 0.0
        self.target_angular = 0.0

        # Periodic publisher to prevent motor watchdog timeout
        self.publish_timer = self.create_timer(0.05, self.timer_cb)

    def set_motion(self, lin_factor: float, ang_factor: float):
        self.target_linear = lin_factor * self.linear_speed
        self.target_angular = ang_factor * self.angular_speed

    def stop(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self._publish_cmd(0.0, 0.0)

    def timer_cb(self):
        self._publish_cmd(self.target_linear, self.target_angular)

    def _publish_cmd(self, lin: float, ang: float):
        twist = Twist()
        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self.pub.publish(twist)


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Escape sequence for arrow keys
            extra = sys.stdin.read(2)
            key += extra
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = SimpleTeleopNode()

    print(BANNER)
    print(f"Current Speed: Linear = {node.linear_speed:.2f} m/s | Angular = {node.angular_speed:.2f} rad/s\n")

    current_state = "IDLE (STOPPED)"

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.08)

            if key in ('w', 'W', '\x1b[A'):      # Forward / Up Arrow
                node.set_motion(1.0, 0.0)
                current_state = "FORWARD  ▲"
            elif key in ('s', 'S', '\x1b[B'):    # Backward / Down Arrow
                node.set_motion(-1.0, 0.0)
                current_state = "BACKWARD ▼"
            elif key in ('a', 'A', '\x1b[D'):    # Spin Left / Left Arrow
                node.set_motion(0.0, 1.0)
                current_state = "TURN LEFT ◄"
            elif key in ('d', 'D', '\x1b[C'):    # Spin Right / Right Arrow
                node.set_motion(0.0, -1.0)
                current_state = "TURN RIGHT ►"
            elif key in ('q', 'Q'):              # Forward Left
                node.set_motion(1.0, 0.7)
                current_state = "FWD-LEFT ◄▲"
            elif key in ('e', 'E'):              # Forward Right
                node.set_motion(1.0, -0.7)
                current_state = "FWD-RIGHT ▲►"
            elif key in ('z', 'Z'):              # Backward Left
                node.set_motion(-1.0, -0.7)
                current_state = "REV-LEFT ◄▼"
            elif key in ('c', 'C'):              # Backward Right
                node.set_motion(-1.0, 0.7)
                current_state = "REV-RIGHT ▼►"
            elif key in (' ', 'x', 'X', 'k', 'K'):  # Stop
                node.stop()
                current_state = "STOPPED  ■"
            elif key in ('+', '='):
                node.linear_speed = min(1.0, round(node.linear_speed + 0.01, 2))
            elif key in ('-', '_'):
                node.linear_speed = max(0.01, round(node.linear_speed - 0.01, 2))
            elif key in (']', '}'):
                node.angular_speed = min(2.0, round(node.angular_speed + 0.02, 2))
            elif key in ('[', '{'):
                node.angular_speed = max(0.02, round(node.angular_speed - 0.02, 2))
            elif key == '\x03':                  # Ctrl+C
                break

            # Print status line
            sys.stdout.write(
                f"\r[STATUS: {current_state:<13}]  Lin: {node.target_linear:+.2f} (max {node.linear_speed:.2f} m/s)  |  Ang: {node.target_angular:+.2f} (max {node.angular_speed:.2f} rad/s)  "
            )
            sys.stdout.flush()

            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.spin_once(node, timeout_sec=0.1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n\nTeleop stopped. Robot commanded to full stop.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
