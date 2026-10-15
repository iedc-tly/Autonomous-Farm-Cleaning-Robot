import unittest
from unittest.mock import MagicMock, patch
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestA4AuthenticationAndHygiene(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROVER_TOKEN'] = 'secret-test-token-12345'
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

    def test_unauthenticated_requests_return_401(self):
        # Protected route without token
        res = self.client.get('/api/status')
        self.assertEqual(res.status_code, 401)
        data = res.get_json()
        self.assertEqual(data.get('error'), 'Unauthorized')

        # Protected route with wrong token
        res = self.client.get('/api/status', headers={'X-Rover-Token': 'wrong-token'})
        self.assertEqual(res.status_code, 401)

    def test_authenticated_requests_succeed(self):
        # With X-Rover-Token header
        res = self.client.get('/api/status', headers={'X-Rover-Token': self.token})
        self.assertEqual(res.status_code, 200)

        # With Authorization: Bearer header
        res = self.client.get('/api/status', headers={'Authorization': f'Bearer {self.token}'})
        self.assertEqual(res.status_code, 200)

    def test_public_routes_exempt_from_auth(self):
        res = self.client.get('/health')
        self.assertEqual(res.status_code, 200)

        res = self.client.get('/api/health')
        self.assertEqual(res.status_code, 200)

        res = self.client.get('/')
        self.assertEqual(res.status_code, 200)

    def test_method_hygiene_removes_get_from_state_changing_routes(self):
        # GET on /cmd_vel should be 405 Method Not Allowed
        res = self.client.get('/api/cmd_vel', headers={'X-Rover-Token': self.token})
        self.assertEqual(res.status_code, 405)

        res = self.client.get('/api/start_mapping', headers={'X-Rover-Token': self.token})
        self.assertEqual(res.status_code, 405)

        res = self.client.get('/api/save_map', headers={'X-Rover-Token': self.token})
        self.assertEqual(res.status_code, 405)

        res = self.client.get('/api/align_map', headers={'X-Rover-Token': self.token})
        self.assertEqual(res.status_code, 405)

    def test_map_name_validation(self):
        # Malicious / invalid map names rejected with 400
        res = self.client.post('/api/save_map', 
                               json={'map_name': 'test;rm -rf /'}, 
                               headers={'X-Rover-Token': self.token})
        self.assertEqual(res.status_code, 400)

        res = self.client.post('/api/save_map', 
                               json={'map_name': 'invalid map space'}, 
                               headers={'X-Rover-Token': self.token})
        self.assertEqual(res.status_code, 400)

        res = self.client.post('/api/save_map', 
                               json={'map_name': 'toolong' * 10}, 
                               headers={'X-Rover-Token': self.token})
        self.assertEqual(res.status_code, 400)

        # Valid map name format accepted (will proceed to handler)
        with patch.object(self.node, 'handle_save_map', return_value=({}, 200)) as mock_save:
            res = self.client.post('/api/save_map', 
                                   json={'map_name': 'my_valid_map_123'}, 
                                   headers={'X-Rover-Token': self.token})
            mock_save.assert_called_once_with(map_name='my_valid_map_123')

if __name__ == '__main__':
    unittest.main()
