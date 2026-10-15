import unittest
import yaml
import os

class TestC3AMCLRecoveryParams(unittest.TestCase):
    def test_nav2_params_amcl_recovery(self):
        params_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control/config/nav2_params.yaml'))
        with open(params_path, 'r') as f:
            cfg = yaml.safe_load(f)
        amcl_ros__parameters = cfg['amcl']['ros__parameters']
        self.assertAlmostEqual(amcl_ros__parameters.get('recovery_alpha_fast'), 0.1)
        self.assertAlmostEqual(amcl_ros__parameters.get('recovery_alpha_slow'), 0.001)

if __name__ == '__main__':
    unittest.main()
