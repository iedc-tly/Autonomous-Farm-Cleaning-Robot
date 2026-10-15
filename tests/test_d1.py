import unittest
from unittest.mock import MagicMock, patch
import sys
import os
import tempfile
import yaml
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestD1UnknownCellsNotFree(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROVER_TOKEN'] = 'test-token-d1'
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

    def test_synthetic_20x20_pgm_rejects_unknown_and_occupied(self):
        from motor_control.farm_cleaner import make_snake
        with tempfile.TemporaryDirectory() as tmpdir:
            # 20x20 synthetic grid
            # resolution = 0.1m -> 2.0m x 2.0m total size
            # origin = [0.0, 0.0, 0.0]
            # row 0 is top (y = 1.9m), row 19 is bottom (y = 0.0m)
            grid = np.full((20, 20), 205, dtype=np.uint8)  # default unknown
            # Top-left quadrant (rows 0..9, cols 0..9): free (254)
            grid[0:10, 0:10] = 254
            # Bottom-right quadrant (rows 10..19, cols 10..19): occupied (0)
            grid[10:20, 10:20] = 0

            pgm_path = os.path.join(tmpdir, "test_map.pgm")
            Image.fromarray(grid).save(pgm_path)

            yaml_path = os.path.join(tmpdir, "test_map.yaml")
            with open(yaml_path, 'w') as f:
                yaml.dump({
                    'image': 'test_map.pgm',
                    'resolution': 0.1,
                    'origin': [0.0, 0.0, 0.0]
                }, f)

            # 1. Test _validate_waypoints_safety
            # Point in free space (x=0.5, y=1.5 -> row 4, col 5) with small robot_radius=0.1
            free_wp = [{'x': 0.5, 'y': 1.5, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}]
            self.assertTrue(self.node._validate_waypoints_safety(free_wp, yaml_path, robot_radius=0.1))

            # Point in unknown space (x=0.5, y=0.5 -> row 14, col 5, value 205)
            unknown_wp = [{'x': 0.5, 'y': 0.5, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}]
            self.assertFalse(self.node._validate_waypoints_safety(unknown_wp, yaml_path, robot_radius=0.1))

            # Point in occupied space (x=1.5, y=0.5 -> row 14, col 15, value 0)
            occupied_wp = [{'x': 1.5, 'y': 0.5, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}]
            self.assertFalse(self.node._validate_waypoints_safety(occupied_wp, yaml_path, robot_radius=0.1))

            # 2. Test make_snake only generates waypoints in free space (y >= 1.0, x <= 1.0)
            wps = make_snake(yaml_path, bounds=[0.0, 2.0, 0.0, 2.0], row_gap=0.2, step_x=0.2, robot_radius=0.1)
            self.assertTrue(len(wps) > 0)
            for wp in wps:
                # Every waypoint must be inside the free quadrant
                self.assertLessEqual(wp['x'], 1.0)
                self.assertGreaterEqual(wp['y'], 1.0)

if __name__ == '__main__':
    unittest.main()
