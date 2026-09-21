from setuptools import setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/slam_bringup.launch.py',
            'launch/nav2_bringup.launch.py',
            'launch/hardware_bringup.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/slam_toolbox_params.yaml',
            'config/nav2_params.yaml',
            'config/mapping.rviz',
            'config/locations.yaml'
        ]),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iedctly',
    maintainer_email='iedctly@todo.todo',
    description='Motor control and encoder odometry nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = motor_control.motor_ros_node:main',
            'encoder_odom_node = motor_control.encoder_odom_node:main',
            'teleop = motor_control.teleop_node:main',
            'calibrate_odom = motor_control.calibrate_odom:main',
            'snake_navigator = motor_control.snake_pattern_navigator:main',
            'waypoint_recorder = motor_control.snake_pattern_navigator:record_main',
            'farm_cleaner = motor_control.farm_cleaner:main',
            'set_location = motor_control.set_location:main',
        ],
    },
)
