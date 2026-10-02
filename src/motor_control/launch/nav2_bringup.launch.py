#!/usr/bin/env python3
"""
nav2_bringup.launch.py

Brings up the robot hardware drivers and the Nav2 navigation stack:
    - robot_state_publisher   (URDF -> robot_description topic)
    - motor_node              (/cmd_vel -> motors)
    - encoder_odom_node       (encoders -> /odom, odom->base_link TF)
    - rplidar C1 driver       (-> /scan)
    - Nav2 bringup stack      (map_server, amcl, planner, controller, bt_navigator, lifecycle manager)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default='/home/iedctly/my_map.yaml')

    # Nav2 configuration params
    nav2_params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('motor_control'),
            'config',
            'nav2_params.yaml'
        )
    )

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

    # ---- mpu9150_node (IMU driver) ----
    mpu9150_node = Node(
        package='motor_control',
        executable='mpu9150_node',
        name='mpu9150_node',
        output='screen',
        parameters=[{
            'i2c_bus': 1,
            'device_address': 0x68,
            'frame_id': 'imu_link',
            'publish_rate': 50.0,
            'calibrate_samples': 200,
        }],
    )

    # ---- encoder_odom_node ----
    encoder_odom_node = Node(
        package='motor_control',
        executable='encoder_odom_node',
        name='encoder_odom_node',
        output='screen',
        parameters=[{
            'invert_all': True,
            'use_imu': True,
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

    # ---- Nav2 Stack Bringup ----
    nav2_bringup_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py',
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_path),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )

    # ---- Autonomous 2D Pose Identifier ----
    auto_localize = LaunchConfiguration('auto_localize', default='false')
    auto_localizer_node = Node(
        package='motor_control',
        executable='auto_global_localizer',
        name='auto_global_localizer',
        output='screen',
        condition=IfCondition(auto_localize),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'map', default_value='/home/iedctly/my_map.yaml',
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=nav2_params_file,
            description='Full path to the ROS2 parameters file to use for all deployed nodes'
        ),
        DeclareLaunchArgument(
            'auto_localize', default_value='false',
            description='Automatically identify and get 2D pose on launch'
        ),
        robot_state_publisher_node,
        motor_node,
        mpu9150_node,
        encoder_odom_node,
        rplidar_launch,
        nav2_launch,
        auto_localizer_node,
    ])
