#!/usr/bin/env python3
"""
slam_bringup.launch.py

Brings up everything needed to drive the robot around and build a map:
    - motor_node          (cmd_vel -> motors)
    - encoder_odom_node    (encoders -> /odom, odom->base_link TF)
    - rplidar C1 driver    (-> /scan)
    - static TF            (base_link -> laser_frame, lidar mount offset)
    - slam_toolbox         (online async SLAM -> map->odom TF, /map topic)

Place this file in: ~/ros2_ws/src/motor_control/launch/slam_bringup.launch.py

Adjust LIDAR_X_OFFSET / LIDAR_Z_HEIGHT below to match your actual mount
position (measure from robot's base_link center, in meters).

IMPORTANT: verify the lidar's actual TF frame name matches LIDAR_FRAME_ID.
Check with:  ros2 topic echo /scan --field header.frame_id
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# Lidar mount offset relative to base_link (meters) — EDIT THESE to match
# your actual robot. "Offset forward" means positive X.
# ---------------------------------------------------------------------------
LIDAR_X_OFFSET = "-0.10"   # forward offset, meters — MEASURE AND ADJUST
LIDAR_Y_OFFSET = "0.0"
LIDAR_Z_HEIGHT = "0.15"   # height above base_link, meters — MEASURE AND ADJUST
LIDAR_ROLL = "0"
LIDAR_PITCH = "0"
LIDAR_YAW = "3.14159265359" # radians; lidar faces backward (180 deg)

LIDAR_FRAME_ID = "laser"  # matches rplidar_c1_launch.py default frame_id


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ---- URDF / robot_state_publisher ----
    urdf_path = os.path.join(
        get_package_share_directory('motor_control'),
        'urdf',
        'robot.urdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    # ---- motor_node ----
    motor_node = Node(
        package='motor_control',
        executable='motor_node',
        name='motor_node',
        output='screen',
    )

    # ---- encoder_odom_node ----
    encoder_odom_node = Node(
        package='motor_control',
        executable='encoder_odom_node',
        name='encoder_odom_node',
        output='screen',
        parameters=[{
            'invert_all': True,
            'use_imu': False,
        }],
    )

    # ---- rplidar C1 driver ----
    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_c1_launch.py',
    )
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path)
    )

    # ---- slam_toolbox (online async) ----
    slam_params_path = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'slam_toolbox_params.yaml',
    )
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py',
    )
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'
        ),
        robot_state_publisher_node,
        motor_node,
        encoder_odom_node,
        rplidar_launch,
        slam_toolbox_launch,
    ])
