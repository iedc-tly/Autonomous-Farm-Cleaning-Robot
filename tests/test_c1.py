import unittest
from unittest.mock import MagicMock, patch
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestC1DockSavePublishNewPose(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROVER_TOKEN'] = 'test-token-c1'
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

    def test_dock_save_passes_new_pose_to_publish_initial_dock_pose_after_save(self):
        call_order = []
        new_dock_pose = {'x': 2.345, 'y': 1.678, 'yaw': 0.785}

        def mock_save_locations(locs):
            call_order.append('save')
            return True, None

        def mock_publish_dock(dock=None):
            call_order.append(('publish', dock))

        self.node.get_current_pose = MagicMock(return_value=new_dock_pose)
        self.node.publish_initial_dock_pose = MagicMock(side_effect=mock_dock if False else mock_publish_dock)

        with patch('rover_app_bridge.rover_bridge_node.load_and_validate_locations', return_value=({}, None)), \
             patch('rover_app_bridge.rover_bridge_node.save_locations_dict', side_effect=mock_save_locations):

            res, err = self.node.save_current_pose_as('dock')
            self.assertIsNone(err)
            self.assertEqual(call_order[0], 'save')
            self.assertEqual(call_order[1][0], 'publish')
            published_dock = call_order[1][1]
            self.assertEqual(published_dock['x'], 2.345)
            self.assertEqual(published_dock['y'], 1.678)
            self.assertEqual(published_dock['yaw'], 0.785)

    def test_failed_save_does_not_publish(self):
        self.node.get_current_pose = MagicMock(return_value={'x': 1.0, 'y': 1.0, 'yaw': 0.0})
        self.node.publish_initial_dock_pose = MagicMock()

        with patch('rover_app_bridge.rover_bridge_node.load_and_validate_locations', return_value=({}, None)), \
             patch('rover_app_bridge.rover_bridge_node.save_locations_dict', return_value=(False, "disk error")):

            res, err = self.node.save_current_pose_as('dock')
            self.assertIsNotNone(err)
            self.node.publish_initial_dock_pose.assert_not_called()

if __name__ == '__main__':
    unittest.main()
