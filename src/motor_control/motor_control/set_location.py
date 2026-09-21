#!/usr/bin/env python3
"""
set_location.py

Coordinate capture helper script.
Usage:
  ros2 run motor_control set_location <dock|pit|sweep_bl|sweep_tr>

Reads current map->base_link pose from TF and updates locations.yaml:
  - dock: {x, y, yaw}
  - pit: {x, y, yaw}
  - sweep_bl: updates sweep.xmin and sweep.ymin
  - sweep_tr: updates sweep.xmax and sweep.ymax
"""

import sys
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

LOCATIONS_FILE = "/home/iedctly/ros2_ws/src/motor_control/config/locations.yaml"


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('dock', 'pit', 'sweep_bl', 'sweep_tr'):
        print("Usage: ros2 run motor_control set_location <dock|pit|sweep_bl|sweep_tr>")
        sys.exit(1)

    target = sys.argv[1]
    rclpy.init()
    node = Node('set_location_helper')
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    print(f"[set_location] Reading current map->base_link pose for target: '{target}'...")
    pose = None
    start_t = node.get_clock().now().seconds_nanoseconds()[0]
    while (node.get_clock().now().seconds_nanoseconds()[0] - start_t) < 5:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            trans = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = trans.transform.translation
            q = trans.transform.rotation
            yaw = yaw_from_quaternion(q)
            pose = {
                'x': float(round(t.x, 3)),
                'y': float(round(t.y, 3)),
                'yaw': float(round(yaw, 3))
            }
            break
        except Exception:
            pass

    node.destroy_node()
    rclpy.shutdown()

    if pose is None:
        print("[set_location] ERROR: Failed to lookup map->base_link transform from TF within 5 seconds.")
        sys.exit(1)

    print(f"[set_location] Captured pose: x={pose['x']}, y={pose['y']}, yaw={pose['yaw']}")

    path = os.path.expanduser(LOCATIONS_FILE)
    if not os.path.exists(path):
        print(f"[set_location] ERROR: locations.yaml not found at {path}")
        sys.exit(1)

    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}

    if 'dock' not in data:
        data['dock'] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    if 'pit' not in data:
        data['pit'] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    if 'sweep' not in data:
        data['sweep'] = {'xmin': 0.0, 'xmax': 0.0, 'ymin': 0.0, 'ymax': 0.0, 'row_gap': 0.4, 'robot_radius': 0.3}

    if target == 'dock':
        data['dock'] = pose
    elif target == 'pit':
        data['pit'] = pose
    elif target == 'sweep_bl':
        data['sweep']['xmin'] = pose['x']
        data['sweep']['ymin'] = pose['y']
    elif target == 'sweep_tr':
        data['sweep']['xmax'] = pose['x']
        data['sweep']['ymax'] = pose['y']

    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print(f"[set_location] Successfully updated '{target}' in {path}:")
    print(yaml.dump(data, sort_keys=False))


if __name__ == '__main__':
    main()
