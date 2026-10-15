import unittest
from unittest.mock import MagicMock, patch
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestD2MapPathSingleSourceOfTruth(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROVER_TOKEN'] = 'test-token-d2'
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        with patch('rover_app_bridge.rover_bridge_node.Cleaner'), \
             patch('rover_app_bridge.rover_bridge_node.TransformListener'), \
             patch('rover_app_bridge.rover_bridge_node.Buffer'), \
             patch('rover_app_bridge.rover_bridge_node.BridgeNode.ensure_nav2_active'):

            from rover_app_bridge.rover_bridge_node import BridgeNode
            self.node = BridgeNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_cleaner_map_yaml_is_source_of_truth(self):
        custom_map = "/custom/path/custom_barn_map.yaml"
        self.node.cleaner.map_yaml = custom_map

        # auto_sweep_entire_room uses self.cleaner.map_yaml
        sweep, err = self.node.auto_sweep_entire_room()
        self.assertIn("custom_barn_map.yaml", err)

        # publish_initial_dock_pose checks self.cleaner.map_yaml
        with patch.object(self.node, 'get_logger') as mock_logger:
            self.node.publish_initial_dock_pose()
            mock_logger().warn.assert_called_with(f"[BridgeNode] Map {custom_map} not found. Cannot verify dock pose extent.")

if __name__ == '__main__':
    unittest.main()
