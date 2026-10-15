import unittest
from unittest.mock import MagicMock, patch
import sys
import os
import time
import threading

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestA2EstopLockFree(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_trigger_e_stop_does_not_block_on_lock(self):
        # We want to test BridgeNode.trigger_e_stop()
        # Mock dependencies that BridgeNode requires
        with patch('rover_app_bridge.rover_bridge_node.Cleaner'), \
             patch('rover_app_bridge.rover_bridge_node.TransformListener'), \
             patch('rover_app_bridge.rover_bridge_node.Buffer'), \
             patch('rover_app_bridge.rover_bridge_node.BridgeNode.ensure_nav2_active'):

            from rover_app_bridge.rover_bridge_node import BridgeNode
            node = BridgeNode()

            # Mock publishers
            mock_estop_pub = MagicMock()
            node.pub_hw_estop = mock_estop_pub
            mock_cmd_pub = MagicMock()
            node.pub_cmd_vel = mock_cmd_pub

            # Hold node.lock in another thread for 200 ms
            lock_held_event = threading.Event()
            release_event = threading.Event()

            def lock_holder():
                with node.lock:
                    lock_held_event.set()
                    release_event.wait(timeout=1.0)

            t = threading.Thread(target=lock_holder, daemon=True)
            t.start()
            self.assertTrue(lock_held_event.wait(timeout=0.5))

            # While lock is actively held by background thread, call trigger_e_stop in a thread
            # and measure time until pub_hw_estop.publish is invoked
            estop_called_event = threading.Event()
            start_time = None
            elapsed = None

            def call_trigger():
                nonlocal start_time, elapsed
                start_time = time.perf_counter()
                node.trigger_e_stop()

            # Hook into publish to record time when /estop is published
            def on_publish(msg):
                nonlocal elapsed
                if elapsed is None and start_time is not None:
                    elapsed = time.perf_counter() - start_time
                    estop_called_event.set()

            mock_estop_pub.publish.side_effect = on_publish

            t_trigger = threading.Thread(target=call_trigger, daemon=True)
            t_trigger.start()

            # Assert that /estop publish happened within 50 ms (0.05 s) despite lock being held
            estop_called = estop_called_event.wait(timeout=0.1)
            self.assertTrue(estop_called, "trigger_e_stop failed to publish /estop while lock was held")
            self.assertIsNotNone(elapsed)
            self.assertLess(elapsed, 0.05, f"Publish took {elapsed:.4f}s, expected < 0.050s")

            # Clean up
            release_event.set()
            t.join(timeout=0.5)
            t_trigger.join(timeout=0.5)
            node.destroy_node()

    def test_get_status_response_pose_outside_lock(self):
        with patch('rover_app_bridge.rover_bridge_node.Cleaner'), \
             patch('rover_app_bridge.rover_bridge_node.TransformListener'), \
             patch('rover_app_bridge.rover_bridge_node.Buffer'), \
             patch('rover_app_bridge.rover_bridge_node.BridgeNode.ensure_nav2_active'):

            from rover_app_bridge.rover_bridge_node import BridgeNode
            node = BridgeNode()

            lock_held_during_pose = None
            def mock_get_pose():
                nonlocal lock_held_during_pose
                # Check whether node.lock is held when get_current_pose is called
                # In Python locks, acquire(blocking=False) returns False if already locked
                acquired = node.lock.acquire(blocking=False)
                if acquired:
                    lock_held_during_pose = False
                    node.lock.release()
                else:
                    lock_held_during_pose = True
                return {'x': 1.0, 'y': 2.0, 'yaw': 0.0}

            node.get_current_pose = mock_get_pose
            st = node.get_status_response()
            self.assertFalse(lock_held_during_pose, "get_current_pose was called while holding node.lock")
            self.assertEqual(st['pose']['x'], 1.0)
            node.destroy_node()

if __name__ == '__main__':
    unittest.main()
