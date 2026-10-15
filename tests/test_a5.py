import unittest
from unittest.mock import MagicMock, patch, call
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/motor_control')))

class TestA5ShellInjectionAndProcessHygiene(unittest.TestCase):
    def test_robot_home_env_var(self):
        from motor_control.farm_cleaner import robot_home
        with patch.dict(os.environ, {"ROBOT_HOME": "/custom/robot/home"}):
            self.assertEqual(robot_home(), "/custom/robot/home")

    def test_launch_uses_exec_list_and_start_new_session(self):
        from motor_control.farm_cleaner import launch
        with patch('subprocess.Popen') as mock_popen:
            # Test with string
            launch("ros2 launch my_pkg test.launch.py")
            self.assertTrue(mock_popen.called)
            args, kwargs = mock_popen.call_args
            self.assertIsInstance(args[0], list)
            self.assertEqual(args[0][0], "bash")
            self.assertEqual(args[0][1], "-c")
            self.assertIn("exec ros2 launch my_pkg test.launch.py", args[0][2])
            self.assertFalse(kwargs.get('shell', False))
            self.assertTrue(kwargs.get('start_new_session', False))

            mock_popen.reset_mock()
            # Test with argv list (including dangerous characters)
            cmd_list = ["ros2", "launch", "motor_control", "nav2.launch.py", "map:=/path; rm -rf /"]
            launch(cmd_list)
            args, kwargs = mock_popen.call_args
            self.assertIsInstance(args[0], list)
            # Quoting ensures the injected command cannot break out
            self.assertIn("'map:=/path; rm -rf /'", args[0][2])

    def test_save_map_safe_execution_and_no_wildcard_rm(self):
        from motor_control.farm_cleaner import Cleaner
        with patch('subprocess.run') as mock_run, \
             patch('os.path.exists', return_value=True), \
             patch('motor_control.farm_cleaner.kill_all_zombies'):
            cleaner = Cleaner(map_path="/tmp/test_map")
            mock_run.return_value = MagicMock(returncode=0)

            cleaner.save_map("injected_name; rm -rf /")

            # Check that subprocess.run was called
            self.assertTrue(mock_run.called)
            called_commands = [c[0][0] for c in mock_run.call_args_list if c[0]]
            for cmd in called_commands:
                if isinstance(cmd, list) and len(cmd) >= 3 and cmd[0] == 'bash':
                    # Ensure shlex quoting prevented shell injection
                    script = cmd[2]
                    self.assertIn("injected_name; rm -rf /", script)
                    self.assertIn("'", script)

            # Check that wildcard rm was never called
            for c in mock_run.call_args_list:
                args = c[0][0]
                if isinstance(args, str):
                    self.assertNotIn("rm -f *.posegraph", args)
                elif isinstance(args, list):
                    for a in args:
                        self.assertNotIn("rm -f *.posegraph", str(a))

    def test_kill_all_zombies_uses_exact_matching(self):
        from motor_control.farm_cleaner import kill_all_zombies
        with patch('subprocess.run') as mock_run, \
             patch('time.sleep'):
            kill_all_zombies()
            calls = mock_run.call_args_list
            for c in calls:
                args = c[0][0]
                # If using pkill with -f, ensure it never targets bare 'nav2'
                if isinstance(args, list):
                    if "-f" in args:
                        self.assertNotIn("nav2", args)
                        self.assertIn("ros2 launch motor_control", args)
                    if "-x" in args:
                        # Exact target
                        self.assertIn(args[1], ["-9", "-x"])
                elif isinstance(args, str):
                    self.assertNotIn("pkill -9 -f nav2", args)

if __name__ == '__main__':
    unittest.main()
