#!/usr/bin/env python3
"""
tools/odom_sign_check.py

CLI diagnostic tool to verify /odom sign conventions (REP-103) on physical hardware.

Manual Verification Procedure:
1. Start hardware bringup on robot (launching encoder_odom_node).
2. Run this script: python3 tools/odom_sign_check.py
3. Push robot forward by hand ~1.0 meter.
   Expected: x increases (x > 0).
4. Rotate robot counter-clockwise (CCW) by hand ~90 degrees (+1.57 rad).
   Expected: yaw increases (e.g., 0.0 -> +1.57 rad / +90°).
   If yaw decreases towards negative values, the yaw sign is inverted!
"""

import sys
import os
import math
import time

def robot_home() -> str:
    return os.path.abspath(os.environ.get("ROBOT_HOME", os.path.expanduser("~")))

def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw angle (radians) from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def run_sign_check(duration_sec: float = 10.0):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry

    class OdomSignChecker(Node):
        def __init__(self):
            super().__init__('odom_sign_checker')
            self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
            self.latest_odom = None

        def odom_cb(self, msg: Odometry):
            self.latest_odom = msg

    if not rclpy.ok():
        rclpy.init()

    node = OdomSignChecker()

    print("=" * 68)
    print("      ODOMETRY SIGN VERIFICATION CHECK (REP-103 COMPLIANCE)         ")
    print("=" * 68)
    print(" INSTRUCTIONS FOR TESTER:")
    print(" 1. Push robot FORWARD by hand ~1.0 meter.")
    print("    -> Expected: X > 0 (increases)")
    print(" 2. Rotate robot COUNTER-CLOCKWISE (CCW) by hand ~90° (+1.57 rad).")
    print("    -> Expected: Yaw INCREASES (0 -> +1.57 rad / +90°)")
    print(" If Yaw decreases (becomes negative), angular sign is inverted!")
    print("=" * 68)
    print(f"Sampling /odom for {duration_sec:.1f} seconds...\n")

    start_time = time.time()
    last_print = 0.0

    try:
        while time.time() - start_time < duration_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if now - last_print >= 0.5:
                last_print = now
                if node.latest_odom:
                    pos = node.latest_odom.pose.pose.position
                    ori = node.latest_odom.pose.pose.orientation
                    yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
                    yaw_deg = math.degrees(yaw)
                    elapsed = now - start_time
                    print(f"[{elapsed:4.1f}s] X: {pos.x:+.3f}m | Y: {pos.y:+.3f}m | Yaw: {yaw:+.3f} rad ({yaw_deg:+.1f}°)")
                else:
                    print(f"[{now - start_time:4.1f}s] Waiting for /odom messages...")
    except KeyboardInterrupt:
        print("\nVerification stopped by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("\n" + "=" * 68)
    print(" Check completed. Review readings above against expected results.")
    print("=" * 68)

if __name__ == '__main__':
    run_sign_check()
