import unittest
from unittest.mock import patch
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from tools.odom_sign_check import quaternion_to_yaw, robot_home

class TestB2OdomSignCheck(unittest.TestCase):
    def test_robot_home_env(self):
        with patch.dict(os.environ, {"ROBOT_HOME": "/custom/path"}):
            self.assertEqual(robot_home(), "/custom/path")

    def test_quaternion_to_yaw(self):
        # 0 degrees: q = (0, 0, 0, 1)
        self.assertAlmostEqual(quaternion_to_yaw(0.0, 0.0, 0.0, 1.0), 0.0, places=4)

        # +90 degrees CCW (yaw = +pi/2): q = (0, 0, sin(pi/4), cos(pi/4)) = (0, 0, 0.7071068, 0.7071068)
        half_angle = math.pi / 4.0
        z = math.sin(half_angle)
        w = math.cos(half_angle)
        self.assertAlmostEqual(quaternion_to_yaw(0.0, 0.0, z, w), math.pi / 2.0, places=4)

        # -90 degrees CW (yaw = -pi/2): q = (0, 0, -sin(pi/4), cos(pi/4))
        self.assertAlmostEqual(quaternion_to_yaw(0.0, 0.0, -z, w), -math.pi / 2.0, places=4)

if __name__ == '__main__':
    unittest.main()
