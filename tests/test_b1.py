import unittest
from unittest.mock import MagicMock, patch
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))

class TestB1NoIMU(unittest.TestCase):
    def test_encoder_odom_node_default_use_imu_false(self):
        # Mock RPi.GPIO before importing or instantiating
        mock_gpio = MagicMock()
        with patch.dict('sys.modules', {'RPi': MagicMock(), 'RPi.GPIO': mock_gpio}):
            import rclpy
            if not rclpy.ok():
                rclpy.init()
            from motor_control.encoder_odom_node import EncoderOdomNode
            node = EncoderOdomNode()
            self.assertFalse(node.get_parameter("use_imu").value)
            node.destroy_node()

    def test_launch_files_have_no_mpu9150_and_use_imu_false(self):
        nav2_launch_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control/launch/nav2_bringup.launch.py'))
        slam_launch_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control/launch/slam_bringup.launch.py'))

        with open(nav2_launch_path, 'r') as f:
            nav2_content = f.read()
        with open(slam_launch_path, 'r') as f:
            slam_content = f.read()

        self.assertNotIn("mpu9150_node", nav2_content)
        self.assertIn("'use_imu': False", nav2_content)
        self.assertIn("'use_imu': False", slam_content)

if __name__ == '__main__':
    unittest.main()
