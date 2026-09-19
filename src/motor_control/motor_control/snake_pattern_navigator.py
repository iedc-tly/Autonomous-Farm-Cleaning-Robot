#!/usr/bin/env python3
"""
snake_pattern_navigator.py

Autonomous waypoint recorder, snake (lawnmower) pattern generator, 
and Nav2 waypoint navigator for ROS 2.

Modes:
  1. RECORD: Listen to /goal_pose from RViz, collect points, and save to YAML.
  2. GENERATE: Create a snake (lawnmower) pattern from bounding box coordinates.
  3. EXECUTE: Send waypoints to Nav2 via BasicNavigator and move the robot.
"""

import sys
import os
import math
import time
import yaml
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw: float):
    """Convert yaw angle (radians) to quaternion (x, y, z, w)."""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0)
    }


def quaternion_to_yaw(q) -> float:
    """Convert quaternion object/dict to yaw angle (radians)."""
    if hasattr(q, 'z') and hasattr(q, 'w'):
        qz, qw = q.z, q.w
        qx = getattr(q, 'x', 0.0)
        qy = getattr(q, 'y', 0.0)
    else:
        qx, qy, qz, qw = q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0), q.get('w', 1.0)
    
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def generate_snake_pattern(x_min: float, x_max: float, y_min: float, y_max: float, 
                           step_x: float = 0.5, step_y: float = 0.5):
    """
    Generate waypoints in a snake (lawnmower / boustrophedon) pattern.
    
    :param x_min: Min X coordinate (meters)
    :param x_max: Max X coordinate (meters)
    :param y_min: Min Y coordinate (meters)
    :param y_max: Max Y coordinate (meters)
    :param step_x: Distance between points along a row (meters)
    :param step_y: Distance between rows (meters)
    :return: List of dicts representing poses [{'x': x, 'y': y, 'yaw': yaw, 'orientation': {...}}]
    """
    if x_min >= x_max or y_min >= y_max:
        raise ValueError("Invalid boundary: x_min must be < x_max and y_min must be < y_max")

    waypoints = []
    
    # Calculate number of rows
    num_rows = max(1, int(math.ceil((y_max - y_min) / step_y)) + 1)
    
    for row in range(num_rows):
        current_y = min(y_min + row * step_y, y_max)
        
        # Calculate X steps for this row
        num_cols = max(1, int(math.ceil((x_max - x_min) / step_x)) + 1)
        x_coords = [min(x_min + col * step_x, x_max) for col in range(num_cols)]
        
        if row % 2 == 0:
            # Moving left-to-right (+X)
            yaw = 0.0
        else:
            # Moving right-to-left (-X)
            x_coords.reverse()
            yaw = math.pi

        for x in x_coords:
            q = yaw_to_quaternion(yaw)
            waypoints.append({
                'x': float(round(x, 3)),
                'y': float(round(current_y, 3)),
                'yaw': float(round(yaw, 3)),
                'orientation': q
            })
            
    return waypoints


def generate_snake_pattern_from_poses(recorded_poses: list, step_x: float = 0.5, step_y: float = 0.5):
    """
    Calculate bounding box (x_min, x_max, y_min, y_max) from a list of recorded poses 
    and generate snake pattern waypoints to cover that defined area.
    """
    if not recorded_poses:
        raise ValueError("Cannot generate snake pattern from empty pose list.")

    x_coords = [p['x'] for p in recorded_poses]
    y_coords = [p['y'] for p in recorded_poses]

    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    # Handle edge case where points are collinear or single point
    if math.isclose(x_min, x_max):
        x_min -= 0.5
        x_max += 0.5
    if math.isclose(y_min, y_max):
        y_min -= 0.5
        y_max += 0.5

    print(f"\n\033[1;34m[AREA BOUNDS CALCULATED FROM GOAL POSES]\033[0m")
    print(f"  X Range: [{x_min:.3f}, {x_max:.3f}] (Span: {x_max - x_min:.2f} m)")
    print(f"  Y Range: [{y_min:.3f}, {y_max:.3f}] (Span: {y_max - y_min:.2f} m)")

    return generate_snake_pattern(x_min, x_max, y_min, y_max, step_x, step_y)


class PoseRecorderNode(Node):
    """ROS2 node to record goal poses clicked in RViz (/goal_pose)."""
    
    def __init__(self, target_count: int = None):
        super().__init__('pose_recorder_node')
        self.waypoints = []
        self.target_count = target_count
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        self.get_logger().info("=" * 60)
        self.get_logger().info("Pose Recorder Active!")
        if self.target_count:
            self.get_logger().info(f"Click '2D Goal Pose' in RViz to capture {self.target_count} goal poses for area definition.")
        else:
            self.get_logger().info("Click '2D Goal Pose' in RViz to capture waypoints.")
        self.get_logger().info("=" * 60)

    def goal_pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = quaternion_to_yaw(q)

        wp = {
            'x': float(round(x, 3)),
            'y': float(round(y, 3)),
            'yaw': float(round(yaw, 3)),
            'orientation': {
                'x': float(round(q.x, 4)),
                'y': float(round(q.y, 4)),
                'z': float(round(q.z, 4)),
                'w': float(round(q.w, 4))
            }
        }
        self.waypoints.append(wp)
        index = len(self.waypoints)
        if self.target_count:
            print(f"\033[1;32m[Goal Pose {index}/{self.target_count} Recorded]\033[0m X: {wp['x']:.2f} m, Y: {wp['y']:.2f} m, Yaw: {math.degrees(yaw):.1f}°")
        else:
            print(f"\033[1;32m[Point {index} Recorded]\033[0m X: {wp['x']:.2f} m, Y: {wp['y']:.2f} m, Yaw: {math.degrees(yaw):.1f}°")


def save_waypoints_to_yaml(waypoints, filepath):
    """Save waypoint list to a YAML file."""
    data = {
        'num_waypoints': len(waypoints),
        'waypoints': waypoints
    }
    with open(filepath, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"\033[1;34m[SAVED]\033[0m {len(waypoints)} waypoints written to: {filepath}")


def load_waypoints_from_yaml(filepath):
    """Load waypoint list from a YAML file."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Waypoint file not found: {filepath}")
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('waypoints', [])


def record_poses(output_filepath: str):
    """Record poses from /goal_pose until interrupted, then save."""
    rclpy.init()
    node = PoseRecorderNode()
    
    print("\n\033[1;36mPress Ctrl+C when finished recording waypoints.\033[0m\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.waypoints:
            save_waypoints_to_yaml(node.waypoints, output_filepath)
        else:
            print("\033[1;33m[NOTICE]\033[0m No waypoints were recorded.")
        node.destroy_node()
        rclpy.shutdown()


def record_area_and_generate(output_filepath: str = 'snake_waypoints.yaml', num_poses: int = 2, step_x: float = 0.5, step_y: float = 0.5, execute: bool = False):
    """
    Record minimum goal poses from /goal_pose in RViz to define a boundary area,
    generate a snake pattern to cover that area, save to YAML, and optionally execute navigation.
    """
    rclpy.init()
    node = PoseRecorderNode(target_count=num_poses if num_poses > 0 else None)
    
    if num_poses > 0:
        print(f"\n\033[1;36mRecording {num_poses} goal poses from RViz to define coverage area...")
        print("Click '2D Goal Pose' in RViz (or press Ctrl+C when done).\033[0m\n")
    else:
        print("\n\033[1;36mRecording goal poses from RViz to define coverage area... Press Ctrl+C when done.\033[0m\n")
        
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if num_poses > 0 and len(node.waypoints) >= num_poses:
                print(f"\n\033[1;32m[GOAL POSES RECORDED]\033[0m Captured {len(node.waypoints)} goal poses.")
                break
    except KeyboardInterrupt:
        print("\n\033[1;33m[RECORDING STOPPED]\033[0m Processing recorded poses...")
    finally:
        recorded_poses = list(node.waypoints)
        node.destroy_node()

    if len(recorded_poses) == 0:
        print("\033[1;31m[ERROR]\033[0m No goal poses recorded. Cannot generate snake pattern.")
        if rclpy.ok():
            rclpy.shutdown()
        return

    snake_waypoints = generate_snake_pattern_from_poses(recorded_poses, step_x=step_x, step_y=step_y)
    print(f"\n\033[1;32m[GENERATED]\033[0m {len(snake_waypoints)} snake pattern waypoints covering defined area.")
    save_waypoints_to_yaml(snake_waypoints, output_filepath)

    if execute:
        execute_navigation(snake_waypoints)
    
    if rclpy.ok():
        rclpy.shutdown()


def record_main():
    """Standalone entry point for ros2 run motor_control waypoint_recorder."""
    parser = argparse.ArgumentParser(description="Record 2D Goal Poses from RViz")
    parser.add_argument('--output', '-o', type=str, default='waypoints.yaml', help="Output YAML file")
    args = parser.parse_args()
    record_poses(args.output)


from lifecycle_msgs.srv import GetState


def wait_for_nav2_active_with_timeout(navigator, timeout_sec=10.0):
    """
    Check if Nav2 lifecycle nodes are running and guide the user if they are missing or uninitialized.
    """
    print("\033[1;36m[STATUS]\033[0m Checking if Nav2 navigation services are active...")
    
    bt_client = navigator.create_client(GetState, 'bt_navigator/get_state')
    start_time = time.time()
    
    service_available = False
    while time.time() - start_time < timeout_sec:
        if bt_client.wait_for_service(timeout_sec=1.0):
            service_available = True
            break
        print(f"\033[1;33m[WAITING]\033[0m Nav2 service ('bt_navigator') not found yet. Retrying...")
    
    if not service_available:
        print("\n\033[1;31m==============================================================\033[0m")
        print("\033[1;31m[ERROR] Nav2 Navigation System is NOT running!\033[0m")
        print("Please launch the navigation system in another terminal window first:")
        print("    \033[1;32m./run_navigation_system.sh\033[0m")
        print("\033[1;31m==============================================================\033[0m\n")
        return False

    print("\033[1;36m[STATUS]\033[0m Nav2 stack detected!")
    print("\033[1;33m[NOTICE]\033[0m If AMCL localization is enabled, make sure to set '2D Pose Estimate' in RViz2 if you haven't done so.")
    print("Waiting for Nav2 nodes to reach active state...")

    try:
        navigator.waitUntilNav2Active()
        return True
    except KeyboardInterrupt:
        print("\n\033[1;33m[CANCELLED]\033[0m Waiting for Nav2 active state interrupted.")
        return False


def execute_navigation(waypoints, frame_id='map', use_follow_waypoints=True):
    """Execute Nav2 navigation through given waypoints."""
    if not waypoints:
        print("\033[1;31m[ERROR]\033[0m Waypoint list is empty!")
        return

    navigator = BasicNavigator()
    
    print("\n\033[1;34m====================================================\033[0m")
    print(f"\033[1;32m      NAV2 WAYPOINT FOLLOWERS - EXECUTING {len(waypoints)} POSES      \033[0m")
    print("\033[1;34m====================================================\033[0m")

    if not wait_for_nav2_active_with_timeout(navigator, timeout_sec=8.0):
        return

    pose_stamped_list = []
    for i, wp in enumerate(waypoints):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.position.z = 0.0
        
        q = wp.get('orientation', yaw_to_quaternion(wp.get('yaw', 0.0)))
        pose.pose.orientation.x = q['x']
        pose.pose.orientation.y = q['y']
        pose.pose.orientation.z = q['z']
        pose.pose.orientation.w = q['w']
        
        pose_stamped_list.append(pose)

    print("\033[1;32m[READY]\033[0m Nav2 is active! Sending waypoints...")

    # Send poses to Nav2
    if use_follow_waypoints:
        # Use followWaypoints (sequential waypoint follower node)
        # Allows skipping unreachable points if stop_on_failure is False in nav2_params.yaml
        navigator.followWaypoints(pose_stamped_list)
    else:
        # Send continuous path through poses
        navigator.goThroughPoses(pose_stamped_list)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            current_waypoint = getattr(feedback, 'current_waypoint', None)
            if current_waypoint is None:
                number_of_poses_remaining = getattr(feedback, 'number_of_poses_remaining', 0)
                current_waypoint = max(0, len(waypoints) - number_of_poses_remaining)
            else:
                number_of_poses_remaining = len(waypoints) - current_waypoint

            distance_remaining = getattr(feedback, 'distance_remaining', 0.0)
            print(f" -> Waypoint {current_waypoint+1}/{len(waypoints)} | Distance remaining: {distance_remaining:.2f} m | Poses left: {number_of_poses_remaining}")
        time.sleep(0.5)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("\n\033[1;32m[SUCCESS]\033[0m Completed all waypoints successfully!")
    elif result == TaskResult.CANCELED:
        print("\n\033[1;33m[CANCELED]\033[0m Waypoint navigation was canceled.")
    elif result == TaskResult.FAILED:
        print("\n\033[1;31m[FAILED]\033[0m Navigation task failed! Check if waypoints are within map bounds and clear of obstacles.")


def main():
    parser = argparse.ArgumentParser(description="Snake Pattern Generator & Nav2 Waypoint Navigator")
    subparsers = parser.add_subparsers(dest='mode', help="Operation mode")

    # Mode 1: Record from RViz
    record_parser = subparsers.add_parser('record', help="Record points from RViz /goal_pose")
    record_parser.add_argument('--output', '-o', type=str, default='waypoints.yaml', help="Output YAML file")

    # Mode 2: Generate Snake Pattern
    gen_parser = subparsers.add_parser('generate', help="Generate snake pattern in a rectangular area")
    gen_parser.add_argument('--xmin', type=float, required=True, help="Min X position")
    gen_parser.add_argument('--xmax', type=float, required=True, help="Max X position")
    gen_parser.add_argument('--ymin', type=float, required=True, help="Min Y position")
    gen_parser.add_argument('--ymax', type=float, required=True, help="Max Y position")
    gen_parser.add_argument('--step-x', type=float, default=0.5, help="Step size along rows (m)")
    gen_parser.add_argument('--step-y', type=float, default=0.5, help="Step size between rows (m)")
    gen_parser.add_argument('--output', '-o', type=str, default='snake_waypoints.yaml', help="Output YAML file")

    # Mode 3: Record goal poses to define area & generate snake pattern
    rec_area_parser = subparsers.add_parser('record-area', help="Record goal poses from RViz to define area and generate snake pattern")
    rec_area_parser.add_argument('--num-poses', '-n', type=int, default=2, help="Number of goal poses to record (default: 2)")
    rec_area_parser.add_argument('--step-x', type=float, default=0.5, help="Step size along rows (m)")
    rec_area_parser.add_argument('--step-y', type=float, default=0.5, help="Step size between rows (m)")
    rec_area_parser.add_argument('--output', '-o', type=str, default='snake_waypoints.yaml', help="Output YAML file")
    rec_area_parser.add_argument('--execute', '-e', action='store_true', help="Execute navigation immediately after generating pattern")

    # Mode 4: Execute navigation
    exec_parser = subparsers.add_parser('execute', help="Execute waypoints using Nav2")
    exec_parser.add_argument('--input', '-i', type=str, default='snake_waypoints.yaml', help="Input YAML file")
    exec_parser.add_argument('--go-through', action='store_true', help="Use goThroughPoses (continuous path) instead of followWaypoints")

    args = parser.parse_args()

    if args.mode == 'record':
        record_poses(args.output)
    elif args.mode == 'generate':
        waypoints = generate_snake_pattern(args.xmin, args.xmax, args.ymin, args.ymax, args.step_x, args.step_y)
        print(f"\n\033[1;32m[GENERATED]\033[0m {len(waypoints)} snake pattern waypoints created.")
        save_waypoints_to_yaml(waypoints, args.output)
    elif args.mode == 'record-area':
        record_area_and_generate(
            output_filepath=args.output,
            num_poses=args.num_poses,
            step_x=args.step_x,
            step_y=args.step_y,
            execute=args.execute
        )
    elif args.mode == 'execute':
        rclpy.init()
        waypoints = load_waypoints_from_yaml(args.input)
        execute_navigation(waypoints, use_follow_waypoints=not args.go_through)
        rclpy.shutdown()
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
