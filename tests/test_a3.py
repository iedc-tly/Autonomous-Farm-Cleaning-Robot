import unittest
from unittest.mock import MagicMock, patch
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestA3NoMotionFallback(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_unknown_navigation_route_returns_404_and_does_not_publish(self):
        with patch('rover_app_bridge.rover_bridge_node.Cleaner'), \
             patch('rover_app_bridge.rover_bridge_node.TransformListener'), \
             patch('rover_app_bridge.rover_bridge_node.Buffer'), \
             patch('rover_app_bridge.rover_bridge_node.BridgeNode.ensure_nav2_active'):

            from rover_app_bridge.rover_bridge_node import BridgeNode, make_flask_app, ROVER_TOKEN
            node = BridgeNode()
            node.publish_twist = MagicMock()
            app = make_flask_app(node)
            client = app.test_client()
            headers = {'X-Rover-Token': ROVER_TOKEN}

            # Test unknown routes with keywords
            res = client.get('/api/navigation', headers=headers)
            self.assertEqual(res.status_code, 404)
            node.publish_twist.assert_not_called()

            res = client.post('/api/steer_left', headers=headers)
            self.assertEqual(res.status_code, 404)
            node.publish_twist.assert_not_called()

            res = client.get('/cmd_vel_unknown', headers=headers)
            self.assertEqual(res.status_code, 404)
            node.publish_twist.assert_not_called()

            node.destroy_node()

if __name__ == '__main__':
    unittest.main()
