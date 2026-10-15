import unittest
from unittest.mock import MagicMock, patch
import sys
import os
import concurrent.futures

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestA7AtomicStateTransitions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROVER_TOKEN'] = 'test-token-a7'
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
            self.node._run_sweep = MagicMock()
            self.node._run_goto_pose = MagicMock()
            self.app = make_flask_app(self.node)
            self.client = self.app.test_client()
            self.token = ROVER_TOKEN

    def tearDown(self):
        self.node.destroy_node()

    def test_parallel_sweep_requests_one_202_one_409(self):
        locs_mock = {
            'sweep': {'xmin': 0.0, 'xmax': 2.0, 'ymin': 0.0, 'ymax': 2.0, 'row_gap': 0.5, 'robot_radius': 0.48},
            'dock': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'pit': {'x': 1.0, 'y': 1.0, 'yaw': 0.0}
        }
        fake_waypoints = [{'x': 0.5, 'y': 0.5, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}]

        with patch('rover_app_bridge.rover_bridge_node.load_and_validate_locations', return_value=(locs_mock, None)), \
             patch('rover_app_bridge.rover_bridge_node.make_snake', return_value=fake_waypoints), \
             patch.object(self.node, '_validate_waypoints_safety', return_value=True):

            headers = {'X-Rover-Token': self.token}

            def send_sweep():
                return self.client.post('/api/sweep', headers=headers)

            with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
                f1 = executor.submit(send_sweep)
                f2 = executor.submit(send_sweep)
                res1 = f1.result()
                res2 = f2.result()

            status_codes = sorted([res1.status_code, res2.status_code])
            self.assertEqual(status_codes, [202, 409], f"Expected [202, 409], got {status_codes}")

    def test_estop_rejects_with_403(self):
        locs_mock = {
            'sweep': {'xmin': 0.0, 'xmax': 2.0, 'ymin': 0.0, 'ymax': 2.0, 'row_gap': 0.5, 'robot_radius': 0.48},
            'dock': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'pit': {'x': 1.0, 'y': 1.0, 'yaw': 0.0}
        }
        with patch('rover_app_bridge.rover_bridge_node.load_and_validate_locations', return_value=(locs_mock, None)):
            self.node.status['e_stop'] = True
            headers = {'X-Rover-Token': self.token}
            res = self.client.post('/api/dock', headers=headers)
            self.assertEqual(res.status_code, 403)
            self.assertEqual(res.get_json().get('error'), 'e_stop_active')

    def test_rollback_on_thread_start_failure(self):
        locs_mock = {
            'dock': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        }
        with patch('rover_app_bridge.rover_bridge_node.load_and_validate_locations', return_value=(locs_mock, None)), \
             patch('rover_app_bridge.rover_bridge_node.threading.Thread', side_effect=RuntimeError("Thread limit reached")):
            headers = {'X-Rover-Token': self.token}
            res = self.client.post('/api/dock', headers=headers)
            self.assertEqual(res.status_code, 500)
            # Ensure state was rolled back to IDLE
            self.assertEqual(self.node.state, "IDLE")
            self.assertIsNone(self.node.task)

    def test_navigating_to_pit_blocks_new_goals(self):
        with self.node.lock:
            self.node.state = "NAVIGATING_TO_PIT"
        headers = {'X-Rover-Token': self.token}
        locs_mock = {'dock': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}}
        with patch('rover_app_bridge.rover_bridge_node.load_and_validate_locations', return_value=(locs_mock, None)):
            res = self.client.post('/api/dock', headers=headers)
            self.assertEqual(res.status_code, 409)
            self.assertEqual(res.get_json().get('error'), 'busy')

if __name__ == '__main__':
    unittest.main()
