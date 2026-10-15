import unittest
from unittest.mock import MagicMock, patch
import sys
import os
import tempfile
import yaml
from PIL import Image

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestC4RemoveLabSpecificConstants(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROVER_TOKEN'] = 'test-token-c4'
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

            from rover_app_bridge.rover_bridge_node import BridgeNode, make_flask_app, ROVER_TOKEN
            self.node = BridgeNode()
            self.app = make_flask_app(self.node)
            self.client = self.app.test_client()
            self.token = ROVER_TOKEN

    def tearDown(self):
        self.node.destroy_node()

    def test_sweep_with_missing_map_returns_412(self):
        self.node.cleaner.map_yaml = "/tmp/non_existent_map_12345.yaml"
        headers = {'X-Rover-Token': self.token}
        locs_mock = {
            'sweep': {'xmin': 0.0, 'xmax': 2.0, 'ymin': 0.0, 'ymax': 2.0, 'row_gap': 0.5, 'robot_radius': 0.48},
            'dock': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'pit': {'x': 1.0, 'y': 1.0, 'yaw': 0.0}
        }
        with patch('rover_app_bridge.rover_bridge_node.load_and_validate_locations', return_value=(locs_mock, None)), \
             patch('os.path.exists', side_effect=lambda p: False if 'non_existent_map' in p else os.path.exists(p)):
            res = self.client.post('/api/sweep', headers=headers)
            self.assertEqual(res.status_code, 412)
            self.assertIn("Map file not found", res.get_json().get('error', ''))

    def test_get_entire_room_bounds_returns_none_on_missing_map(self):
        from motor_control.farm_cleaner import get_entire_room_bounds
        bounds = get_entire_room_bounds("/tmp/does_not_exist_xyz.yaml")
        self.assertIsNone(bounds)

    def test_publish_initial_dock_pose_checks_map_extent(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            img_path = os.path.join(tmpdir, "map.png")
            # 100 x 100 image, res=0.1 -> width=10m, height=10m. origin = [-5.0, -5.0, 0.0]
            # extent: X in [-5.0, 5.0], Y in [-5.0, 5.0]
            img = Image.new('L', (100, 100), color=254)
            img.save(img_path)

            map_yaml_path = os.path.join(tmpdir, "map.yaml")
            with open(map_yaml_path, 'w') as f:
                yaml.dump({
                    'image': 'map.png',
                    'resolution': 0.1,
                    'origin': [-5.0, -5.0, 0.0]
                }, f)

            self.node.cleaner.map_yaml = map_yaml_path
            self.node.pub_initial_pose = MagicMock()

            # Dock outside map extent: X=10.0 > 5.0
            out_dock = {'x': 10.0, 'y': 0.0, 'yaw': 0.0}
            self.node.publish_initial_dock_pose(dock=out_dock)
            self.node.pub_initial_pose.publish.assert_not_called()

            # Dock inside map extent: X=1.0, Y=2.0
            in_dock = {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
            self.node.publish_initial_dock_pose(dock=in_dock)
            self.assertTrue(self.node.pub_initial_pose.publish.called)

if __name__ == '__main__':
    unittest.main()
