import unittest
from unittest.mock import MagicMock, patch
import sys
import os

# Ensure local src/ is searched first
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/rover_app_bridge')))

import rclpy

class TestA1EstopLatch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Mock motor_node before importing
        cls.mock_mc = MagicMock()
        sys.modules['motor_control.motor_node'] = cls.mock_mc
        sys.modules['motor_node'] = cls.mock_mc
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.mock_mc.reset_mock()
        from motor_control.motor_ros_node import MotorNode
        self.node = MotorNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_estop_latch_and_clear(self):
        self.assertFalse(self.node.estopped)

        # 1. Trigger estop
        from std_msgs.msg import Empty
        self.node.estop_cb(Empty())
        self.assertTrue(self.node.estopped)
        self.mock_mc.stop_all.assert_called_with(immediate=True)

        # 2. cmd_vel during estop must NOT call mc.drive
        self.mock_mc.drive.reset_mock()
        from geometry_msgs.msg import Twist
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.node.cmd_vel_cb(twist)
        self.mock_mc.drive.assert_not_called()
        self.mock_mc.stop_all.assert_called()

        # 3. Clear estop
        self.node.estop_clear_cb(Empty())
        self.assertFalse(self.node.estopped)

        # 4. cmd_vel now drives
        self.mock_mc.drive.reset_mock()
        self.node.cmd_vel_cb(twist)
        self.mock_mc.drive.assert_called_once()

    def test_non_finite_rejection(self):
        self.mock_mc.drive.reset_mock()
        self.mock_mc.stop_all.reset_mock()
        from geometry_msgs.msg import Twist
        twist = Twist()
        twist.linear.x = float('nan')
        twist.angular.z = 0.0
        self.node.cmd_vel_cb(twist)
        self.mock_mc.drive.assert_not_called()
        self.mock_mc.stop_all.assert_called()

    def test_param_validation(self):
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rclpy.parameter import Parameter as RclParameter
        
        p_neg = RclParameter('max_linear', RclParameter.Type.DOUBLE, -0.1)
        res = self.node.param_cb([p_neg])
        self.assertFalse(res.successful)

        p_zero = RclParameter('max_angular', RclParameter.Type.DOUBLE, 0.0)
        res = self.node.param_cb([p_zero])
        self.assertFalse(res.successful)

        p_valid = RclParameter('max_linear', RclParameter.Type.DOUBLE, 0.6)
        res = self.node.param_cb([p_valid])
        self.assertTrue(res.successful)
        self.assertEqual(self.node.max_linear, 0.6)

if __name__ == '__main__':
    unittest.main()
