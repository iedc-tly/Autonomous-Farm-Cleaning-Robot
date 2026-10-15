import unittest
from unittest.mock import MagicMock, patch, call
import sys
import os
import inspect

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestC2ReducedNoMotionUpdates(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROVER_TOKEN'] = 'test-token-c2'
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

    def test_sharpen_pose_stationary_default_2_updates(self):
        sig = inspect.signature(self.node.sharpen_pose_stationary)
        self.assertEqual(sig.parameters['num_updates'].default, 2)

    def test_sharpen_pose_stationary_calls_service_twice_and_spaced(self):
        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        self.node.nomotion_client = mock_client

        with patch('time.sleep') as mock_sleep:
            res = self.node.sharpen_pose_stationary()
            self.assertTrue(res)
            self.assertEqual(mock_client.call_async.call_count, 2)
            # Verify 0.3s spacing
            mock_sleep.assert_has_calls([call(0.3), call(0.3), call(0.3)])

if __name__ == '__main__':
    unittest.main()
