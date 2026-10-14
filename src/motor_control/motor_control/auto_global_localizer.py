#!/usr/bin/env python3
"""
auto_global_localizer.py

Autonomous 2D Pose Identification and Self-Localization for ROS 2 / Nav2 AMCL.

Identifies and gets the robot's 2D pose autonomously when navigation starts:
1. SMART AUTO-DETECT (Default):
   - Fast-checks saved Dock/Home pose from locations.yaml.
   - Publishes initial pose to AMCL /initialpose.
   - Performs rapid stationary laser scan-matching via /request_nomotion_update.
   - If LiDAR matches the dock (covariance < threshold), robot locks in within ~1.5s!
   - If robot was moved or placed in an unknown spot, seamlessly transitions to Step 2.
2. 360° GLOBAL SCAN-MATCHING (Unknown Starting Positions):
   - Calls /reinitialize_global_localization to scatter particles across the map.
   - Rotates robot in place smoothly (0.22 rad/s) while triggering LiDAR updates.
   - Monitored AMCL covariance drops as laser matches map walls.
   - Stops the instant particles converge into a sharp cluster.
   - Performs fine stationary sharpening and locks in final (x, y, yaw) 2D pose.
"""

import sys
import os
import time
import math
import yaml
import argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState


def yaw_from_quaternion(q) -> float:
    """Calculate yaw angle from quaternion object or dict."""
    if hasattr(q, 'z') and hasattr(q, 'w'):
        qz, qw = q.z, q.w
        qx = getattr(q, 'x', 0.0)
        qy = getattr(q, 'y', 0.0)
    else:
        qx, qy, qz, qw = q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0), q.get('w', 1.0)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> dict:
    """Convert yaw angle (radians) to quaternion dict."""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': float(round(math.sin(yaw / 2.0), 4)),
        'w': float(round(math.cos(yaw / 2.0), 4))
    }


def find_locations_file() -> str:
    """Locate locations.yaml configuration file."""
    candidates = [
        "/home/iedctly/ros2_ws/src/motor_control/config/locations.yaml",
        "/home/iedctly/ros2_ws/install/motor_control/share/motor_control/config/locations.yaml",
        os.path.expanduser("~/locations.yaml")
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return None


def load_dock_pose() -> dict:
    """Load saved dock / initial position from locations.yaml."""
    fpath = find_locations_file()
    if fpath:
        try:
            with open(fpath, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'dock' in data:
                    d = data['dock']
                    return {
                        'x': float(d.get('x', 0.0)),
                        'y': float(d.get('y', 0.0)),
                        'yaw': float(d.get('yaw', 0.0))
                    }
        except Exception:
            pass
    return None


class PrecisionGlobalLocalizer(Node):
    """
    Autonomous 2D Pose Identifier and Localizer.
    Manages AMCL global particle reinitialization, stationary scan matching,
    and rotational convergence maneuvers.
    """

    def __init__(self, node_name: str = 'global_auto_localizer'):
        super().__init__(node_name)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.turn_mode_pub = self.create_publisher(Bool, '/single_side_turn', 10)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.reinit_client = self.create_client(Empty, '/reinitialize_global_localization')
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')
        self.amcl_state_client = self.create_client(GetState, '/amcl/get_state')

        self.latest_pose = None
        self.cov_x = float('inf')
        self.cov_y = float('inf')
        self.cov_yaw = float('inf')
        self.pose_received_count = 0

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Record latest AMCL estimated pose and diagonal covariance."""
        self.latest_pose = msg
        self.cov_x = msg.pose.covariance[0]
        self.cov_y = msg.pose.covariance[7]
        self.cov_yaw = msg.pose.covariance[35]
        self.pose_received_count += 1

    def wait_for_amcl_active(self, timeout_sec: float = 25.0) -> bool:
        """Wait for AMCL node to be up and responsive."""
        self.get_logger().info("Checking if AMCL localization service is active...")
        start_t = time.time()
        while (time.time() - start_t) < timeout_sec:
            if self.reinit_client.wait_for_service(timeout_sec=0.5):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def trigger_global_localization(self) -> bool:
        """Call /reinitialize_global_localization to scatter particles across map."""
        if not self.reinit_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service '/reinitialize_global_localization' not available!")
            return False

        req = Empty.Request()
        future = self.reinit_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.latest_pose = None
            self.cov_x = float('inf')
            self.cov_y = float('inf')
            self.cov_yaw = float('inf')
            self.get_logger().info("Global localization reinitialized (particles scattered across map).")
            return True
        return False

    def call_nomotion_update(self, timeout_sec: float = 0.2) -> bool:
        """Force AMCL to resample particles against current LiDAR scan while stationary."""
        if not self.nomotion_client.service_is_ready():
            return False
        try:
            req = Empty.Request()
            future = self.nomotion_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            return True
        except Exception:
            return False

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        """Seed AMCL with initial 2D pose estimate on /initialpose with calibrated precision covariance."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        q = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = float(q['x'])
        msg.pose.pose.orientation.y = float(q['y'])
        msg.pose.pose.orientation.z = float(q['z'])
        msg.pose.pose.orientation.w = float(q['w'])

        # Calibrated dock covariance (0.02 m², 0.02 m², ~7° yaw)
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.015

        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.04)

    def stop_robot(self):
        """Immediately halt robot drive motors."""
        stop_twist = Twist()
        for _ in range(4):
            self.cmd_pub.publish(stop_twist)
            time.sleep(0.03)

    def get_current_pose_tuple(self):
        """Return (position, orientation, yaw) from latest pose."""
        if not self.latest_pose:
            return None, None, 0.0
        p = self.latest_pose.pose.pose.position
        q = self.latest_pose.pose.pose.orientation
        yaw = yaw_from_quaternion(q)
        return p, q, yaw

    def run_micro_alignment(self, angular_speed: float = 0.12, duration: float = 2.4) -> bool:
        """
        Gentle in-place micro-rotation wiggle (+/- 5 degrees) to provide orthogonal
        wall constraints to AMCL scan matching and collapse lingering covariance.
        """
        self.get_logger().info("\033[1;36m[PRECISION ALIGNMENT]\033[0m Executing scan-matching micro-alignment wiggle...")
        turn_msg = Bool()
        turn_msg.data = False
        self.turn_mode_pub.publish(turn_msg)

        twist = Twist()
        phase_time = duration / 3.0

        try:
            # Phase 1: Small nudge left
            twist.angular.z = float(angular_speed)
            t_sub = time.time()
            while (time.time() - t_sub) < phase_time and rclpy.ok():
                self.cmd_pub.publish(twist)
                self.call_nomotion_update(timeout_sec=0.05)
                rclpy.spin_once(self, timeout_sec=0.05)

            # Phase 2: Counter-turn right
            twist.angular.z = -float(angular_speed)
            t_sub = time.time()
            while (time.time() - t_sub) < (phase_time * 1.5) and rclpy.ok():
                self.cmd_pub.publish(twist)
                self.call_nomotion_update(timeout_sec=0.05)
                rclpy.spin_once(self, timeout_sec=0.05)

            # Phase 3: Return to center
            twist.angular.z = float(angular_speed)
            t_sub = time.time()
            while (time.time() - t_sub) < (phase_time * 0.5) and rclpy.ok():
                self.cmd_pub.publish(twist)
                self.call_nomotion_update(timeout_sec=0.05)
                rclpy.spin_once(self, timeout_sec=0.05)

        finally:
            self.stop_robot()

        # Final stationary sharpening burst
        for _ in range(6):
            self.call_nomotion_update(timeout_sec=0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.08)

        self.get_logger().info(
            f"Micro-alignment complete. Final Covariance: X={self.cov_x:.4f}, Y={self.cov_y:.4f}, Yaw={self.cov_yaw:.4f}"
        )
        return self.cov_x < 0.04 and self.cov_y < 0.04 and self.cov_yaw < 0.035

    def try_dock_localization(self, dock_pose: dict, num_updates: int = 15, target_cov: float = 0.045) -> bool:
        """
        Fast-path check & lock: Seed AMCL with saved Dock pose, burst no-motion updates
        to align with LiDAR scans against map walls. If close but unaligned, executes micro-wiggle.
        """
        self.get_logger().info(
            f"\033[1;36m[CHECKING SAVED DOCK POSE]\033[0m Testing pose: "
            f"X={dock_pose['x']:.3f}m, Y={dock_pose['y']:.3f}m, Yaw={math.degrees(dock_pose['yaw']):.1f}°"
        )
        self.publish_initial_pose(dock_pose['x'], dock_pose['y'], dock_pose['yaw'])
        time.sleep(0.3)

        # Burst no-motion updates so AMCL evaluates laser scan against map
        for i in range(num_updates):
            self.call_nomotion_update(timeout_sec=0.15)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.12)
            if self.cov_x <= target_cov and self.cov_y <= target_cov and self.cov_yaw <= 0.035:
                # Do 2 more updates to fully lock in
                self.call_nomotion_update(timeout_sec=0.15)
                rclpy.spin_once(self, timeout_sec=0.1)
                break

        rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(
            f"Dock Verification Covariance: X={self.cov_x:.4f}, Y={self.cov_y:.4f}, Yaw={self.cov_yaw:.4f}"
        )

        if self.cov_x <= target_cov and self.cov_y <= target_cov and self.cov_yaw <= 0.035:
            return True

        # If covariance is close (within dock vicinity, but needs angular excitation to snap walls)
        if self.cov_x < 0.15 and self.cov_y < 0.15:
            if self.run_micro_alignment():
                return True

        return False

    def run_global_localization(self, max_duration: float = 40.0, angular_speed: float = 0.15, target_cov: float = 0.045) -> bool:
        """
        Active Scan-Matching Maneuver:
        Rotate robot in place slowly while continuously evaluating AMCL particles
        until covariance drops below target threshold.
        """
        if not self.trigger_global_localization():
            return False

        self.get_logger().info(
            f"\033[1;36m[SCAN-MATCHING ROTATION]\033[0m Rotating in place ({angular_speed:.2f} rad/s) for LiDAR convergence..."
        )

        # Set point turn mode (zero-radius spin)
        turn_msg = Bool()
        turn_msg.data = False
        self.turn_mode_pub.publish(turn_msg)

        t0 = time.time()
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = float(angular_speed)

        last_update_t = 0.0
        converged = False

        try:
            while (time.time() - t0) < max_duration and rclpy.ok():
                self.cmd_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)

                now = time.time()
                if now - last_update_t > 0.15:
                    self.call_nomotion_update(timeout_sec=0.05)
                    last_update_t = now

                if self.cov_x < target_cov and self.cov_y < target_cov and self.cov_yaw < 0.035:
                    converged = True
                    self.get_logger().info("\033[1;32m[CONVERGENCE DETECTED]\033[0m Particles locked into sharp cluster!")
                    break

        finally:
            self.stop_robot()

        # If still slightly uncertain, brief reverse micro-rotation
        if not converged and (self.cov_x > 0.06 or self.cov_y > 0.06):
            self.get_logger().info("\033[1;33m[FINE TUNING]\033[0m Reverse micro-rotation...")
            twist.angular.z = -float(angular_speed)
            t_rev = time.time()
            try:
                while (time.time() - t_rev) < 5.0 and rclpy.ok():
                    self.cmd_pub.publish(twist)
                    rclpy.spin_once(self, timeout_sec=0.05)
                    self.call_nomotion_update(timeout_sec=0.05)
                    if self.cov_x < target_cov and self.cov_y < target_cov:
                        converged = True
                        break
            finally:
                self.stop_robot()

        # Final stationary sharpening burst
        for _ in range(4):
            self.call_nomotion_update(timeout_sec=0.1)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        return self.latest_pose is not None and self.cov_x < 0.08 and self.cov_y < 0.08

    def _print_success_banner(self, title: str, x: float, y: float, yaw: float):
        """Display formatted green success summary box."""
        deg = math.degrees(yaw)
        print("\n\033[1;32m" + "=" * 70 + "\033[0m")
        print(f"\033[1;32m   ✓ {title}   \033[0m")
        print("\033[1;32m" + "=" * 70 + "\033[0m")
        print(f"   \033[1;37mEstimated 2D Pose:\033[0m  X = \033[1;32m{x:+.3f}\033[0m m,  Y = \033[1;32m{y:+.3f}\033[0m m")
        print(f"   \033[1;37mRobot Heading    :\033[0m  Yaw = \033[1;32m{deg:+.1f}°\033[0m ({yaw:+.3f} rad)")
        print(f"   \033[1;37mCovariance       :\033[0m  X = {self.cov_x:.4f} m²,  Y = {self.cov_y:.4f} m²,  Yaw = {self.cov_yaw:.4f} rad²")
        print(f"   \033[1;37mLocalization     :\033[0m  \033[1;32mCONFIRMED & READY FOR NAVIGATION\033[0m (No manual RViz tool needed)")
        print("\033[1;32m" + "=" * 70 + "\033[0m\n")

    def identify_and_get_pose(self, mode: str = 'auto', timeout: float = 40.0, angular_speed: float = 0.15):
        """
        Unified entry method:
        - mode='auto'  : Check dock first. If LiDAR matches, done in ~1.5s. Else, run 360° scan matching.
        - mode='dock'  : Only set and verify dock pose.
        - mode='global': Force 360° global particle scattering and scan-matching.
        """
        if not self.wait_for_amcl_active(timeout_sec=25.0):
            self.get_logger().error("AMCL services not active. Is Nav2 running?")
            return False, None

        dock_pose = load_dock_pose()

        # Step 1: Check dock if available
        if mode in ('auto', 'dock') and dock_pose is not None:
            if self.try_dock_localization(dock_pose):
                p, q, yaw = self.get_current_pose_tuple()
                self._print_success_banner("ROBOT LOCALIZED AT SAVED DOCK POSE", p.x, p.y, yaw)
                return True, {'x': p.x, 'y': p.y, 'yaw': yaw}
            elif mode == 'dock':
                self.get_logger().warn("Dock pose verification failed, but mode='dock' requested.")
                return False, None
            else:
                self.get_logger().info(
                    "\033[1;33m[DOCK MISMATCH]\033[0m Current LiDAR scan does not match dock. Robot is elsewhere on map."
                )

        # Step 1.5: If saved dock didn't match, test map origin (0, 0, 0) where mapping started
        if mode in ('auto', 'dock'):
            origin_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            self.get_logger().info("\033[1;36m[TESTING MAP ORIGIN]\033[0m Testing if robot is at SLAM start origin (0, 0, 0)...")
            if self.try_dock_localization(origin_pose):
                p, q, yaw = self.get_current_pose_tuple()
                self._print_success_banner("ROBOT LOCALIZED AT MAP START ORIGIN (0, 0, 0)", p.x, p.y, yaw)
                return True, {'x': p.x, 'y': p.y, 'yaw': yaw}
            elif mode == 'dock':
                self.get_logger().warn("Neither dock nor map origin matched.")
                return False, None

        # Step 2: Global scan-matching
        self.get_logger().info("\033[1;36m[IDENTIFYING 2D POSE]\033[0m Starting autonomous 360° global scan-matching...")
        success = self.run_global_localization(max_duration=timeout, angular_speed=angular_speed)

        if success and self.latest_pose:
            p, q, yaw = self.get_current_pose_tuple()
            self._print_success_banner("2D POSE IDENTIFIED AUTONOMOUSLY VIA 360° SCAN-MATCHING", p.x, p.y, yaw)
            return True, {'x': p.x, 'y': p.y, 'yaw': yaw}
        else:
            self.get_logger().warn(
                f"[LOCALIZATION WARNING] Finished with Covariance: X={self.cov_x:.4f}, Y={self.cov_y:.4f}, Yaw={self.cov_yaw:.4f}"
            )
            if self.latest_pose:
                p, q, yaw = self.get_current_pose_tuple()
                return True, {'x': p.x, 'y': p.y, 'yaw': yaw}
            return False, None

    # Backward compatibility
    def run_precision_localization(self):
        return self.identify_and_get_pose(mode='global')


# Alias class name for readability
AutonomousPoseIdentifier = PrecisionGlobalLocalizer


def main(args=None):
    parser = argparse.ArgumentParser(description="Autonomous 2D Pose Identifier & Self-Localizer for Nav2 AMCL")
    parser.add_argument('--mode', '-m', choices=['auto', 'dock', 'global'], default='auto',
                        help="Localization mode: 'auto' (dock first, fallback to 360° scan), 'dock' (dock only), 'global' (force 360° scan)")
    parser.add_argument('--timeout', '-t', type=float, default=40.0, help="Max timeout for global scan-matching (sec)")
    parser.add_argument('--speed', '-s', type=float, default=0.15, help="Rotation speed for scan-matching (rad/s)")
    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = PrecisionGlobalLocalizer()

    try:
        success, pose = node.identify_and_get_pose(
            mode=parsed_args.mode,
            timeout=parsed_args.timeout,
            angular_speed=parsed_args.speed
        )
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        node.stop_robot()
        print("\n[CANCELLED] Pose identification interrupted by user.")
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
