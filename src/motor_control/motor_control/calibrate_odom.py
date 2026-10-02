#!/usr/bin/env python3
"""
calibrate_odom.py

Interactive / Command-line node to execute Odometry Calibration routines:
1. Linear Distance Test (Calibrate WHEEL_DIAMETER_M)
2. 360-degree In-Place Turn Test (Calibrate TRACK_WIDTH_M)
3. Full UMBmark Square Test (CW and CCW 1m x 1m squares)
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

class OdomCalibrator(Node):
    def __init__(self):
        super().__init__('odom_calibrator')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False

    def get_remote_parameter(self, target_node: str, param_name: str, default_val=None):
        client = self.create_client(GetParameters, f'{target_node}/get_parameters')
        if not client.wait_for_service(timeout_sec=1.5):
            return default_val
        req = GetParameters.Request()
        req.names.append(param_name)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            res = future.result()
            if res and res.values and res.values[0].type == ParameterType.PARAMETER_DOUBLE:
                return res.values[0].double_value
        return default_val

    def set_remote_parameter(self, target_node: str, param_name: str, double_val: float) -> bool:
        client = self.create_client(SetParameters, f'{target_node}/set_parameters')
        if not client.wait_for_service(timeout_sec=1.5):
            return False
        req = SetParameters.Request()
        p = Parameter()
        p.name = param_name
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float(double_val)
        req.parameters.append(p)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            res = future.result()
            if res and res.results and res.results[0].successful:
                return True
        return False


    def _odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def stop_robot(self):
        msg = Twist()
        for _ in range(5):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def drive_straight(self, distance_m: float, speed_mps: float = 0.2):
        self.get_logger().info(f"--- Starting Linear Test: Target Distance = {distance_m:.2f} m ---")
        start_wait = time.time()
        while not self.has_odom and rclpy.ok():
            if time.time() - start_wait > 5.0:
                self.get_logger().error("Timeout waiting for /odom topic! Ensure encoder_odom_node is running.")
                return 0.0
            rclpy.spin_once(self, timeout_sec=0.1)

        start_x = self.current_x
        start_y = self.current_y
        dist_traveled = 0.0

        twist = Twist()
        twist.linear.x = speed_mps

        max_duration = max(20.0, (distance_m / max(0.05, abs(speed_mps))) * 3.0)
        start_drive_time = time.time()
        last_log_time = time.time()

        while dist_traveled < distance_m and rclpy.ok():
            if time.time() - start_drive_time > max_duration:
                self.get_logger().warn(f"Linear drive timeout ({max_duration:.1f}s)! Stopping robot.")
                break

            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            dist_traveled = math.hypot(dx, dy)

            if time.time() - last_log_time > 0.5:
                self.get_logger().info(f"Odom Linear Progress: {dist_traveled:.3f} m / {distance_m:.3f} m")
                last_log_time = time.time()

        self.stop_robot()
        self.get_logger().info(f"Odom reported linear distance: {dist_traveled:.4f} m")
        return dist_traveled

    def rotate_in_place(self, target_radians: float, speed_radps: float = 0.35):
        self.get_logger().info(f"--- Starting Rotation Test: Target Angle = {math.degrees(target_radians):.1f} deg at {speed_radps:.2f} rad/s ---")
        start_wait = time.time()
        while not self.has_odom and rclpy.ok():
            if time.time() - start_wait > 5.0:
                self.get_logger().error("Timeout waiting for /odom topic! Ensure encoder_odom_node is running.")
                return 0.0
            rclpy.spin_once(self, timeout_sec=0.1)

        last_yaw = self.current_yaw
        total_rotated = 0.0
        sign = 1.0 if target_radians >= 0 else -1.0
        abs_target = abs(target_radians)

        twist = Twist()
        twist.angular.z = sign * abs(speed_radps)

        max_duration = max(60.0, (abs_target / max(0.05, abs(speed_radps))) * 3.0)
        start_rot_time = time.time()
        last_log_time = time.time()

        while total_rotated < abs_target and rclpy.ok():
            if time.time() - start_rot_time > max_duration:
                self.get_logger().warn(f"Rotation timeout ({max_duration:.1f}s)! Stopping robot.")
                break

            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            
            dyaw = self.current_yaw - last_yaw
            # Normalize dyaw to [-pi, pi]
            while dyaw > math.pi: dyaw -= 2.0 * math.pi
            while dyaw < -math.pi: dyaw += 2.0 * math.pi
            
            # Accumulate magnitude of turning
            total_rotated += abs(dyaw)
            last_yaw = self.current_yaw

            if time.time() - last_log_time > 0.5:
                self.get_logger().info(f"Odom Rotation Progress: {math.degrees(total_rotated):.1f}° / {math.degrees(abs_target):.1f}°")
                last_log_time = time.time()

        self.stop_robot()
        self.get_logger().info(f"Odom reported rotation: {math.degrees(total_rotated):.2f} deg ({total_rotated:.4f} rad)")
        return total_rotated

    def run_square_test(self, side_length: float = 1.0, speed_linear: float = 0.4, speed_angular: float = 0.7, ccw: bool = True):
        dir_str = "CCW" if ccw else "CW"
        self.get_logger().info(f"--- Executing {dir_str} Square Test ({side_length}m x {side_length}m) ---")
        turn_angle = (math.pi / 2.0) if ccw else -(math.pi / 2.0)
        
        for i in range(4):
            self.get_logger().info(f"Square Side {i+1}/4: Driving straight {side_length}m...")
            self.drive_straight(side_length, speed_linear)
            time.sleep(1.0)
            self.get_logger().info(f"Square Corner {i+1}/4: Turning 90 degrees ({dir_str})...")
            self.rotate_in_place(turn_angle, speed_angular)
            time.sleep(1.0)

        self.get_logger().info(f"{dir_str} Square Test Completed!")


def main(args=None):
    rclpy.init(args=args)
    calibrator = OdomCalibrator()

    print("==========================================================")
    print("      ROBOT ENCODER ODOMETRY CALIBRATION TOOL             ")
    print("==========================================================")
    print("Select Calibration Test Mode:")
    print("1. Linear Distance Test (Calibrate WHEEL_DIAMETER_M)")
    print("2. 360-degree In-Place Turn Test (Calibrate TRACK_WIDTH_M)")
    print("3. UMBmark Square Test (1m x 1m Square Routine)")
    print("==========================================================")

    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("Enter choice (1/2/3): ").strip()

    if choice == '1':
        active_dia = calibrator.get_remote_parameter('/encoder_odom_node', 'wheel_diameter', 0.1013184)
        if active_dia is not None:
            print(f"\n[INFO] Current active WHEEL_DIAMETER_M in encoder_odom_node: {active_dia:.7f} m")
        else:
            active_dia = 0.1013184
            print(f"\n[WARN] Could not query encoder_odom_node directly, using default: {active_dia:.7f} m")

        dia_in = input(f"Enter WHEEL_DIAMETER_M to test with [press Enter to keep {active_dia:.7f}]: ").strip()
        if dia_in:
            try:
                active_dia = float(dia_in)
                success = calibrator.set_remote_parameter('/encoder_odom_node', 'wheel_diameter', active_dia)
                if success:
                    print(f"-> [OK] Dynamically updated encoder_odom_node wheel_diameter to {active_dia:.7f} m")
                else:
                    print(f"-> [WARN] Could not update live node parameter; proceeding with value for calculation.")
            except ValueError:
                print("Invalid number entered; keeping current value.")

        dist = float(input("Enter target distance in meters (e.g. 2.0) [default 2.0]: ") or "2.0")
        speed = float(input("Enter linear velocity in m/s [default 0.4]: ") or "0.4")
        
        odom_dist = calibrator.drive_straight(dist, speed_mps=speed)
        
        if odom_dist > 0.01:
            print("\n--- LINEAR CALIBRATION RESULTS ---")
            actual_dist = float(input(f"Measure and enter ACTUAL distance traveled on floor in meters (e.g. {odom_dist:.3f}): "))
            
            new_dia = active_dia * (actual_dist / odom_dist)
            scale_factor = actual_dist / odom_dist
            print(f"Odom Reported Distance: {odom_dist:.4f} m")
            print(f"Actual Measured Distance: {actual_dist:.4f} m")
            print(f"Linear Scale Factor (C_x): {scale_factor:.6f}")
            print(f"-> RECOMMENDED NEW WHEEL_DIAMETER_M = {new_dia:.7f}")

            apply_now = input(f"Apply new wheel_diameter ({new_dia:.7f} m) to live running node now? (y/n) [default: y]: ").strip().lower()
            if apply_now in ('', 'y', 'yes'):
                success = calibrator.set_remote_parameter('/encoder_odom_node', 'wheel_diameter', new_dia)
                if success:
                    print(f"-> [OK] Successfully applied {new_dia:.7f} m to running encoder_odom_node!")
                else:
                    print(f"-> [WARN] Could not set parameter on running node.")
        else:
            print("Linear test aborted or 0 distance recorded.")

    elif choice == '2':
        active_tw = calibrator.get_remote_parameter('/encoder_odom_node', 'track_width', 0.8014317)
        if active_tw is not None:
            print(f"\n[INFO] Current active TRACK_WIDTH_M in encoder_odom_node: {active_tw:.7f} m")
        else:
            active_tw = 0.8014317
            print(f"\n[WARN] Could not query encoder_odom_node directly, using default: {active_tw:.7f} m")

        tw_in = input(f"Enter TRACK_WIDTH_M to test with [press Enter to keep {active_tw:.7f}]: ").strip()
        if tw_in:
            try:
                active_tw = float(tw_in)
                success = calibrator.set_remote_parameter('/encoder_odom_node', 'track_width', active_tw)
                if success:
                    print(f"-> [OK] Dynamically updated encoder_odom_node track_width to {active_tw:.7f} m")
                else:
                    print(f"-> [WARN] Could not update live node parameter; proceeding with value for calculation.")
            except ValueError:
                print("Invalid number entered; keeping current value.")

        turns = float(input("\nEnter number of 360-deg turns to execute (e.g. 1.0 or 3.0) [default 1.0]: ") or "1.0")
        speed_radps = float(input("Enter turning velocity in rad/s [default 0.7]: ") or "0.7")
        
        target_rad = turns * 2.0 * math.pi
        odom_rot = calibrator.rotate_in_place(target_rad, speed_radps=speed_radps)
        
        if odom_rot > 0.01:
            print("\n--- ANGULAR / TRACK WIDTH CALIBRATION RESULTS ---")
            actual_turns = float(input(f"Enter ACTUAL number of turns completed in real life (e.g. 1.0 or 0.75): ") or str(turns))
            actual_rad = actual_turns * 2.0 * math.pi
            
            new_tw = active_tw * (odom_rot / actual_rad)
            print(f"Odom Reported Rotation: {math.degrees(odom_rot):.2f} deg ({odom_rot:.4f} rad)")
            print(f"Actual Real Rotation:   {math.degrees(actual_rad):.2f} deg ({actual_rad:.4f} rad)")
            print(f"-> RECOMMENDED NEW TRACK_WIDTH_M = {new_tw:.7f}")

            apply_now = input(f"Apply new track_width ({new_tw:.7f} m) to live running node now? (y/n) [default: y]: ").strip().lower()
            if apply_now in ('', 'y', 'yes'):
                success = calibrator.set_remote_parameter('/encoder_odom_node', 'track_width', new_tw)
                if success:
                    print(f"-> [OK] Successfully applied {new_tw:.7f} m to running encoder_odom_node!")
                    print("   You can run test 2 again immediately to verify that the robot now completes 1.0 full turn.")
                else:
                    print(f"-> [WARN] Could not set parameter on running node.")
        else:
            print("Rotation test aborted or 0 rotation recorded.")

    elif choice == '3':
        side = float(input("Enter square side length in meters (default 1.0): ") or "1.0")
        speed_lin = float(input("Enter linear velocity in m/s (default 0.4): ") or "0.4")
        speed_ang = float(input("Enter turning velocity in rad/s (default 0.7): ") or "0.7")
        print("Executing CCW Square Test...")
        calibrator.run_square_test(side_length=side, speed_linear=speed_lin, speed_angular=speed_ang, ccw=True)
        time.sleep(2.0)
        print("Executing CW Square Test...")
        calibrator.run_square_test(side_length=side, speed_linear=speed_lin, speed_angular=speed_ang, ccw=False)

    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
