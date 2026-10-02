#!/usr/bin/env python3
"""
hardware_bringup.launch.py

Brings up essential robot hardware drivers:
    - robot_state_publisher   (URDF -> robot_description topic)
    - motor_node              (/cmd_vel -> pigpio motor driver)
    - encoder_odom_node       (encoders -> /odom, odom->base_link TF)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    use_imu = LaunchConfiguration('use_imu', default='true')

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
            'use_imu': use_imu,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Enable IMU gyro fusion for odometry'
        ),
        robot_state_publisher_node,
        motor_node,
        mpu9150_node,
        encoder_odom_node,
    ])
