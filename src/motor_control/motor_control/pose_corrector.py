#!/usr/bin/env python3
"""
pose_corrector.py

Autonomous 2D Pose Estimation Corrector for ROS 2 / Nav2 AMCL.

Key Features:
1. Monitors 2D pose uncertainty (cov_x, cov_y, cov_yaw) from /amcl_pose.
2. Triggers AMCL particle filter sharpening via /request_nomotion_update.
3. Performs active scan-matching micro-rotations if covariance exceeds limits.
4. Corrects 2D pose estimates autonomously during snake pattern execution.
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped


def yaw_from_quaternion(q):
    """Calculate yaw angle from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AutonomousPoseCorrector:
    """
    Monitors 2D pose covariance and executes autonomous self-correction
    during snake pattern waypoint navigation.
    """

    def __init__(self, parent_node=None, node_name='autonomous_pose_corrector',
                 cov_limit_pos=0.15, cov_limit_yaw=0.10):
        if parent_node is not None:
            self.node = parent_node
            self._owns_node = False
        else:
            self.node = Node(node_name)
            self._owns_node = True

        self.cov_limit_pos = cov_limit_pos
        self.cov_limit_yaw = cov_limit_yaw

        self.latest_pose = None
        self.cov_x = float('inf')
        self.cov_y = float('inf')
        self.cov_yaw = float('inf')

        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.nomotion_client = self.node.create_client(Empty, '/request_nomotion_update')

        self.node.get_logger().info(
            f"Autonomous Pose Corrector ready (Thresholds: Pos={self.cov_limit_pos} m^2, Yaw={self.cov_limit_yaw} rad^2)"
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update current pose and covariance from AMCL."""
        self.latest_pose = msg
        self.cov_x = msg.pose.covariance[0]
        self.cov_y = msg.pose.covariance[7]
        self.cov_yaw = msg.pose.covariance[35]

    def get_covariance(self):
        """Return current covariance tuple (cov_x, cov_y, cov_yaw)."""
        return self.cov_x, self.cov_y, self.cov_yaw

    def get_pose_tuple(self):
        """Return (x, y, yaw) if pose is available, else None."""
        if self.latest_pose is None:
            return None
        p = self.latest_pose.pose.pose.position
        q = self.latest_pose.pose.pose.orientation
        return p.x, p.y, yaw_from_quaternion(q)

    def needs_correction(self) -> bool:
        """Check if 2D pose covariance exceeds target accuracy limits."""
        if self.latest_pose is None:
            return False
        return (self.cov_x > self.cov_limit_pos or 
                self.cov_y > self.cov_limit_pos or 
                self.cov_yaw > self.cov_limit_yaw)

    def request_nomotion_update(self, timeout_sec=0.5) -> bool:
        """
        Call AMCL /request_nomotion_update service to trigger particle weight
        resampling against static map LiDAR features without requiring motion.
        """
        if not self.nomotion_client.service_is_ready():
            return False

        try:
            req = Empty.Request()
            self.nomotion_client.call_async(req)
            return True
        except Exception as e:
            self.node.get_logger().warn(f"Failed no-motion update service call: {e}")
            return False

    def publish_twist_duration(self, linear_x, angular_z, duration_sec):
        """Publish velocity command and execute continuous no-motion updates."""
        t0 = time.time()
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)

        while (time.time() - t0) < duration_sec:
            self.cmd_pub.publish(twist)
            self.request_nomotion_update(timeout_sec=0.05)
            time.sleep(0.05)
            if not self.needs_correction():
                break

    def stop_robot(self):
        """Send zero velocity commands to halt robot."""
        stop_twist = Twist()
        for _ in range(4):
            self.cmd_pub.publish(stop_twist)
            time.sleep(0.03)

    def sharpen_pose(self, timeout_sec=0.1) -> bool:
        """
        Non-intrusive pose sharpening via AMCL no-motion update.
        Safe to call during navigation without interrupting Nav2 or motor control.
        """
        return self.request_nomotion_update(timeout_sec=timeout_sec)

    def correct_pose(self, force=False, allow_twist=True) -> bool:
        """
        Execute autonomous 2D pose correction:
        1. Fast stationary AMCL no-motion update burst.
        2. Scan-matching micro-rotational wiggles only if allow_twist=True and robot is not actively navigating.
        """
        if not force and not self.needs_correction():
            return True

        self.node.get_logger().info(
            f"\033[1;33m[2D POSE CORRECTION TRIGGERED]\033[0m Current Uncertainty: "
            f"X={self.cov_x:.4f}, Y={self.cov_y:.4f}, Yaw={self.cov_yaw:.4f}"
        )

        # Step 1: Stationary no-motion update burst
        for _ in range(5):
            self.request_nomotion_update(timeout_sec=0.2)
            time.sleep(0.1)
            if not self.needs_correction():
                self.node.get_logger().info(
                    f"\033[1;32m[2D POSE CORRECTED]\033[0m Sharp covariance achieved via no-motion update: "
                    f"X={self.cov_x:.4f}, Y={self.cov_y:.4f}, Yaw={self.cov_yaw:.4f}"
                )
                return True

        if not allow_twist:
            return not self.needs_correction()

        # Step 2: Active scan-matching micro-rotation wiggle (only if allow_twist=True)
        self.node.get_logger().info("\033[1;36m[2D POSE CORRECTION]\033[0m Executing scan-matching micro-rotation wiggle...")
        
        # Micro rotate left (+z)
        self.publish_twist_duration(linear_x=0.0, angular_z=0.15, duration_sec=1.2)
        self.stop_robot()
        time.sleep(0.2)

        # Micro rotate right (-z) if still needed
        if self.needs_correction():
            self.publish_twist_duration(linear_x=0.0, angular_z=-0.15, duration_sec=1.2)
            self.stop_robot()
            time.sleep(0.2)

        self.node.get_logger().info(
            f"\033[1;32m[2D POSE CORRECTION RESULT]\033[0m Final Covariance: "
            f"X={self.cov_x:.4f}, Y={self.cov_y:.4f}, Yaw={self.cov_yaw:.4f}"
        )
        return not self.needs_correction()

    def destroy_node(self):
        if self._owns_node and self.node:
            self.node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    corrector = AutonomousPoseCorrector()
    print("\n\033[1;36mRunning Standalone 2D Pose Corrector Test...\033[0m")
    
    time.sleep(1.0)
    cov_x, cov_y, cov_yaw = corrector.get_covariance()
    print(f"Current Covariance: X={cov_x:.4f}, Y={cov_y:.4f}, Yaw={cov_yaw:.4f}")
    
    success = corrector.correct_pose(force=True)
    if success:
        print("\033[1;32m[SUCCESS] 2D Pose estimate self-corrected successfully!\033[0m")
    else:
        print("\033[1;33m[NOTICE] Pose correction finished; review covariance values.\033[0m")

    corrector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
