#!/usr/bin/env python3
"""
autostart_robot.py

Supervised Boot Manager for 3-Button Autonomous Operation.
Brings up system components in required order:
  1. pigpiod motor daemon
  2. Hardware drivers & Nav2 bringup (map:=/home/iedctly/my_map.yaml, autostart:=true)
  3. Verifies lifecycle nodes reach active state (retries up to 3 times)
  4. Publishes initial pose = dock pose from locations.yaml
  5. Launches rover_bridge_node (HTTP :8000)
"""

import os
import sys
import time
import subprocess
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState

LOCATIONS_FILE = "/home/iedctly/ros2_ws/src/motor_control/config/locations.yaml"
MAP_FILE = "/home/iedctly/my_map.yaml"


def log(msg):
    print(f"[Autostart] {msg}", flush=True)


def ensure_pigpiod():
    log("Checking pigpiod daemon...")
    res = subprocess.run(["pgrep", "-x", "pigpiod"], stdout=subprocess.DEVNULL)
    if res.returncode != 0:
        log("Starting pigpiod...")
        subprocess.run(["sudo", "systemctl", "start", "pigpiod"], stderr=subprocess.DEVNULL)
        time.sleep(1.0)
        res = subprocess.run(["pgrep", "-x", "pigpiod"], stdout=subprocess.DEVNULL)
        if res.returncode != 0:
            log("Launching standalone pigpiod process...")
            subprocess.Popen(["pigpiod"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)


def check_lifecycle_nodes(node):
    required = ['amcl', 'bt_navigator', 'planner_server', 'controller_server']
    clients = {}
    for n in required:
        clients[n] = node.create_client(GetState, f'{n}/get_state')

    start_time = time.time()
    while time.time() - start_time < 30.0:
        all_active = True
        for name, client in clients.items():
            if not client.service_is_ready():
                all_active = False
                break
            req = GetState.Request()
            future = client.call_async(req)
            st = time.time()
            while not future.done() and (time.time() - st) < 0.3:
                time.sleep(0.01)
            if not future.done() or future.result() is None or future.result().current_state.label != 'active':
                all_active = False
                break
        if all_active:
            return True
        time.sleep(1.0)
    return False


def yaw_to_quaternion(yaw: float) -> dict:
    import math
    return {
        'x': 0.0,
        'y': 0.0,
        'z': float(round(math.sin(yaw / 2.0), 4)),
        'w': float(round(math.cos(yaw / 2.0), 4))
    }


def publish_initial_pose(node):
    dock = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    if os.path.exists(LOCATIONS_FILE):
        try:
            with open(LOCATIONS_FILE, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'dock' in data:
                    dock = data['dock']
        except Exception as e:
            log(f"Warning: Failed to read dock pose from locations.yaml: {e}")

    log(f"Setting initial pose to dock ({dock.get('x', 0.0)}, {dock.get('y', 0.0)}, yaw={dock.get('yaw', 0.0)})...")
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    time.sleep(1.0)

    for i in range(5):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(dock.get('x', 0.0))
        msg.pose.pose.position.y = float(dock.get('y', 0.0))
        msg.pose.pose.position.z = 0.0
        q = yaw_to_quaternion(float(dock.get('yaw', 0.0)))
        msg.pose.pose.orientation.x = q['x']
        msg.pose.pose.orientation.y = q['y']
        msg.pose.pose.orientation.z = q['z']
        msg.pose.pose.orientation.w = q['w']

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        pub.publish(msg)
        log(f"Published initialpose attempt {i+1}/5")
        time.sleep(1.0)


def main():
    log("==================================================")
    log("  3-BUTTON AUTONOMOUS ROBOT SUPERVISED BOOT MANAGER ")
    log("==================================================")

    # Step 1: pigpiod
    ensure_pigpiod()

    # Step 2: Launch Nav2 & Hardware Bringup with retries
    nav2_proc = None
    rclpy.init()
    node = Node('autostart_supervisor')

    max_retries = 3
    success = False

    for attempt in range(1, max_retries + 1):
        log(f"Attempt {attempt}/{max_retries}: Launching hardware & Nav2 bringup...")
        cmd = f"ros2 launch motor_control nav2_bringup.launch.py map:={MAP_FILE}"
        nav2_proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)

        log("Waiting for Nav2 lifecycle nodes to become active...")
        if check_lifecycle_nodes(node):
            log("SUCCESS: Nav2 lifecycle nodes active!")
            success = True
            break

        log(f"WARNING: Attempt {attempt} failed to reach active lifecycle state! Terminating process group...")
        try:
            os.killpg(os.getpgid(nav2_proc.pid), 9)
        except Exception:
            pass
        subprocess.run("pkill -9 -f nav2; pkill -9 -f motor_node; pkill -9 -f rplidar", shell=True)
        time.sleep(3.0)

    if not success:
        log("ERROR: Nav2 bringup failed after maximum retries!")
        sys.exit(1)

    # Step 3: Publish Initial Pose
    publish_initial_pose(node)

    # Step 4: Launch Bridge Node
    log("Launching rover_bridge_node (HTTP :8000)...")
    bridge_script = "/home/iedctly/ros2_ws/src/rover_app_bridge/rover_app_bridge/rover_bridge_node.py"
    bridge_proc = subprocess.Popen([sys.executable, bridge_script])

    log("Autostart complete! System operational on port 8000.")

    try:
        bridge_proc.wait()
    except KeyboardInterrupt:
        log("Shutting down autostart supervisor...")
        if bridge_proc: bridge_proc.terminate()
        if nav2_proc:
            try: os.killpg(os.getpgid(nav2_proc.pid), 9)
            except Exception: pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
