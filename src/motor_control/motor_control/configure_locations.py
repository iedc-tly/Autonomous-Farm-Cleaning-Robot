#!/usr/bin/env python3
"""
configure_locations.py

Unified configuration script for entering all robot position parameters in one place:
1. Docking Position (Initial Position: x, y, yaw)
2. Pit Position (x, y, yaw)
3. Snake Pattern Coverage Area (xmin, xmax, ymin, ymax, row_gap, step_x, robot_radius)

Saves coordinates directly to src/motor_control/config/locations.yaml and 
automatically generates snake pattern waypoints in snake_waypoints.yaml.

Supports:
- Interactive Guided Setup Wizard
- Manual command-line argument input
- Live TF robot position capture (map -> base_link)
- RViz /goal_pose click capture
"""

import sys
import os
import math
import time
import yaml
import argparse
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

LOCATIONS_FILE = "/home/iedctly/ros2_ws/src/motor_control/config/locations.yaml"
SNAKE_WAYPOINTS_FILE = "/home/iedctly/ros2_ws/snake_waypoints.yaml"


def yaw_to_quaternion(yaw: float) -> dict:
    """Convert yaw angle (radians) to quaternion dict."""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': float(round(math.sin(yaw / 2.0), 4)),
        'w': float(round(math.cos(yaw / 2.0), 4))
    }


def yaw_from_quaternion(q) -> float:
    """Calculate yaw angle from quaternion."""
    if hasattr(q, 'z') and hasattr(q, 'w'):
        qz, qw = q.z, q.w
        qx = getattr(q, 'x', 0.0)
        qy = getattr(q, 'y', 0.0)
    else:
        qx, qy, qz, qw = q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0), q.get('w', 1.0)
    
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def capture_tf_pose(timeout_sec=5.0):
    """Capture current robot pose from map -> base_link transform."""
    if not rclpy.ok():
        rclpy.init()

    node = Node('pose_tf_capturer')
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    print("   [TF] Looking up map -> base_link transform...")
    start_t = time.time()
    pose = None

    while (time.time() - start_t) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
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
    return pose


def capture_rviz_goal_pose(prompt_label="RViz Click", timeout_sec=None):
    """
    Capture pose clicked in RViz on /goal_pose topic.
    Waits indefinitely without exiting or timing out unless user presses Ctrl+C.
    """
    if not rclpy.ok():
        rclpy.init()

    node = Node('rviz_pose_capturer')
    captured = []

    def callback(msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)
        captured.append({
            'x': float(round(x, 3)),
            'y': float(round(y, 3)),
            'yaw': float(round(yaw, 3)),
            'orientation': {
                'x': float(round(msg.pose.orientation.x, 4)),
                'y': float(round(msg.pose.orientation.y, 4)),
                'z': float(round(msg.pose.orientation.z, 4)),
                'w': float(round(msg.pose.orientation.w, 4))
            }
        })

    sub = node.create_subscription(PoseStamped, '/goal_pose', callback, 10)
    print(f"\n   \033[1;36m[RViz]\033[0m Waiting for '2D Goal Pose' click in RViz for {prompt_label}...")
    print("   Listening on /goal_pose indefinitely (will NOT exit or time out. Press Ctrl+C to cancel)...")

    start_t = time.time()
    try:
        while not captured and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if timeout_sec is not None and (time.time() - start_t) >= timeout_sec:
                break
    except KeyboardInterrupt:
        print(f"\n   \033[1;33m[CANCELLED]\033[0m Waiting for {prompt_label} cancelled by user.")
        node.destroy_node()
        return None

    node.destroy_node()
    if captured:
        p = captured[0]
        print(f"   \033[1;32m[RECEIVED]\033[0m Goal Pose captured for {prompt_label}: X={p['x']:.3f} m, Y={p['y']:.3f} m, Yaw={math.degrees(p['yaw']):.1f}°")
        return p
    return None


def capture_chained_rviz_goal_poses(target_count: int = None, prompt_label: str = "Sweep Path") -> list:
    """
    Capture multiple goal poses clicked sequentially in RViz on /goal_pose topic.
    Waits indefinitely without timing out until the user finishes or cancels.
    
    Controls:
      • Click '2D Goal Pose' in RViz to add waypoints to the sweep.
      • Press [ENTER] in this terminal when finished clicking poses (free click mode).
      • Type 'u' + Enter to UNDO the last clicked pose.
      • Type 'q' + Enter or press Ctrl+C to cancel.
    """
    if not rclpy.ok():
        rclpy.init()

    node = Node('rviz_chained_pose_capturer')
    captured_poses = []
    stop_event = threading.Event()
    lock = threading.Lock()

    def callback(msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)
        pose_data = {
            'x': float(round(x, 3)),
            'y': float(round(y, 3)),
            'yaw': float(round(yaw, 3)),
            'orientation': {
                'x': float(round(msg.pose.orientation.x, 4)),
                'y': float(round(msg.pose.orientation.y, 4)),
                'z': float(round(msg.pose.orientation.z, 4)),
                'w': float(round(msg.pose.orientation.w, 4))
            }
        }
        with lock:
            captured_poses.append(pose_data)
            count = len(captured_poses)

        print(f"\n   \033[1;32m[Pose #{count} Captured]\033[0m X = {pose_data['x']:.3f} m, Y = {pose_data['y']:.3f} m, Yaw = {math.degrees(pose_data['yaw']):.1f}°")
        if target_count and count >= target_count:
            print(f"   \033[1;32m[TARGET REACHED]\033[0m Reached target count of {target_count} poses!")
            stop_event.set()
        else:
            target_str = f" ({count}/{target_count})" if target_count else ""
            print(f"   -> Chained Poses{target_str}: {count}. Click next '2D Goal Pose' in RViz, or press [ENTER] when finished.")
            print("   > ", end='', flush=True)

    sub = node.create_subscription(PoseStamped, '/goal_pose', callback, 10)

    def spin_worker():
        while not stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin_worker, daemon=True)
    spin_thread.start()

    print("\n   \033[1;36m============================================================\033[0m")
    print(f"   \033[1;36m       CHAINING GOAL POSES IN RVIZ FOR {prompt_label.upper()}           \033[0m")
    print("   \033[1;36m============================================================\033[0m")
    print("   • Click '2D Goal Pose' in RViz on your map to add waypoints to the sweep.")
    print("   • Listening on topic /goal_pose indefinitely (will NOT exit or timeout).")
    if target_count:
        print(f"   • Target Count: {target_count} poses. (Will automatically proceed once {target_count} poses are clicked).")
    else:
        print("   • When you are finished clicking poses, press \033[1;33m[ENTER]\033[0m in this terminal.")
    print("   • Type \033[1;33m'u'\033[0m + Enter to UNDO the last clicked pose.")
    print("   • Type \033[1;31m'q'\033[0m + Enter or press Ctrl+C to cancel.")
    print("   ------------------------------------------------------------\n")

    try:
        if target_count:
            while not stop_event.is_set():
                with lock:
                    if len(captured_poses) >= target_count:
                        break
                time.sleep(0.1)
        else:
            while not stop_event.is_set():
                user_input = input().strip()
                if user_input.lower() == 'q':
                    print("   \033[1;33m[CANCELLED]\033[0m Chaining cancelled by user.")
                    with lock:
                        captured_poses.clear()
                    break
                elif user_input.lower() == 'u':
                    with lock:
                        if captured_poses:
                            removed = captured_poses.pop()
                            print(f"   \033[1;33m[UNDO]\033[0m Removed Pose #{len(captured_poses)+1}: X={removed['x']}, Y={removed['y']}")
                            print(f"   -> Remaining poses in chain: {len(captured_poses)}")
                        else:
                            print("   \033[1;33m[Notice]\033[0m No poses to undo.")
                    continue

                with lock:
                    count = len(captured_poses)
                if count < 2:
                    print(f"   \033[1;33m[Notice]\033[0m Only {count} pose(s) recorded so far. Please click at least 2 poses in RViz to create a sweep chain, or 'q' to cancel.")
                else:
                    print(f"   \033[1;32m[DONE]\033[0m Finished chaining with {count} goal poses.")
                    break
    except KeyboardInterrupt:
        print("\n   \033[1;33m[Notice]\033[0m Chaining interrupted by user.")
    finally:
        stop_event.set()
        time.sleep(0.1)
        node.destroy_node()

    return captured_poses


def generate_chained_path_waypoints(poses: list, step_size: float = 0.4, robot_radius: float = 0.48) -> list:
    """
    Generate dense, smooth sweep waypoints chaining through a sequence of goal poses (P1 -> P2 -> ... -> Pn).
    Interpolates straight line segments between consecutive poses, computes continuous motion heading (yaw),
    and validates obstacle clearance if map is available.
    """
    if not poses or len(poses) < 2:
        return poses

    pts = []
    for i in range(len(poses) - 1):
        p_start = poses[i]
        p_end = poses[i + 1]

        x0, y0 = p_start['x'], p_start['y']
        x1, y1 = p_end['x'], p_end['y']

        dist = math.hypot(x1 - x0, y1 - y0)
        n_steps = max(1, int(math.ceil(dist / max(0.05, step_size))))

        for step in range(n_steps):
            frac = step / float(n_steps)
            curr_x = float(round(x0 + frac * (x1 - x0), 3))
            curr_y = float(round(y0 + frac * (y1 - y0), 3))

            if pts and math.hypot(curr_x - pts[-1][0], curr_y - pts[-1][1]) < 0.02:
                continue
            pts.append((curr_x, curr_y))

    # Append final endpoint
    last_pose = poses[-1]
    last_pt = (float(round(last_pose['x'], 3)), float(round(last_pose['y'], 3)))
    if not pts or math.hypot(last_pt[0] - pts[-1][0], last_pt[1] - pts[-1][1]) >= 0.02:
        pts.append(last_pt)

    # Optional obstacle grid safety check against /home/iedctly/my_map.yaml
    map_yaml = "/home/iedctly/my_map.yaml"
    if os.path.exists(map_yaml):
        try:
            with open(map_yaml, 'r') as f:
                meta = yaml.safe_load(f)
            from PIL import Image
            import numpy as np
            img_rel = meta['image']
            img_path = img_rel if img_rel.startswith('/') else os.path.join(os.path.dirname(map_yaml), img_rel)
            if os.path.exists(img_path):
                img = Image.open(img_path)
                grid = np.array(img)
                h, w = grid.shape
                res = float(meta['resolution'])
                ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
                rad_px = int(math.ceil(robot_radius / res))

                def _is_safe(px: float, py: float) -> bool:
                    c_col = int(math.floor((px - ox) / res))
                    c_row = int(math.floor(h - 1 - (py - oy) / res))
                    if c_col < 0 or c_col >= w or c_row < 0 or c_row >= h:
                        return False
                    for dr in range(-rad_px, rad_px + 1):
                        for dc in range(-rad_px, rad_px + 1):
                            if dr * dr + dc * dc > rad_px * rad_px:
                                continue
                            ri = c_row + dr
                            ci = c_col + dc
                            if ri < 0 or ri >= h or ci < 0 or ci >= w:
                                return False
                            if grid[ri, ci] < 100:
                                return False
                    return True

                unsafe_indices = [idx for idx, pt in enumerate(pts) if not _is_safe(pt[0], pt[1])]
                if unsafe_indices:
                    print(f"   \033[1;33m[Warning]\033[0m {len(unsafe_indices)} of {len(pts)} chained waypoints are close to obstacles in {map_yaml}!")
                else:
                    print(f"   \033[1;32m[Safety OK]\033[0m All {len(pts)} chained waypoints verified safe against map obstacles.")
        except Exception:
            pass

    # Compute headings
    waypoints = []
    for i in range(len(pts)):
        x, y = pts[i]
        if i < len(pts) - 1:
            seg_dx = pts[i + 1][0] - x
            seg_dy = pts[i + 1][1] - y
            if abs(seg_dx) > 1e-4 or abs(seg_dy) > 1e-4:
                yaw = math.atan2(seg_dy, seg_dx)
            else:
                yaw = waypoints[-1]['yaw'] if waypoints else 0.0
        else:
            yaw = last_pose.get('yaw', waypoints[-1]['yaw'] if waypoints else 0.0)

        yaw = float(round(yaw, 3))
        waypoints.append({
            'x': x,
            'y': y,
            'yaw': yaw,
            'orientation': yaw_to_quaternion(yaw)
        })

    return waypoints


def generate_snake_pattern(xmin: float, xmax: float, ymin: float, ymax: float, step_x: float = 0.4, row_gap: float = 0.4, robot_radius: float = 0.48):
    """Generate snake pattern waypoints with map occupancy checking and safe clearance."""
    map_file = "/home/iedctly/my_map.yaml"
    if os.path.exists(map_file):
        try:
            from motor_control.farm_cleaner import make_snake
            wps = make_snake(map_file, bounds=[xmin, xmax, ymin, ymax], row_gap=row_gap, step_x=step_x, robot_radius=robot_radius)
            if wps and len(wps) > 0:
                print(f"[configure_locations] Successfully created {len(wps)} map-filtered, obstacle-safe waypoints.")
                return wps
            else:
                print("\033[1;33m[configure_locations Warning] Map filter returned 0 points (area outside free space or blocked). Falling back to geometric grid.\033[0m")
        except Exception as e:
            print(f"[configure_locations] Notice: make_snake fallback ({e})")

    from motor_control.snake_pattern_navigator import generate_snake_pattern as _gen
    return _gen(xmin, xmax, ymin, ymax, step_x=step_x, step_y=row_gap, axis='auto')


def save_locations_yaml(dock: dict, pit: dict, sweep: dict, filepath: str = LOCATIONS_FILE):
    """Save dock, pit, and sweep settings to locations.yaml."""
    data = {
        'dock': {
            'x': float(round(dock['x'], 3)),
            'y': float(round(dock['y'], 3)),
            'yaw': float(round(dock['yaw'], 3))
        },
        'pit': {
            'x': float(round(pit['x'], 3)),
            'y': float(round(pit['y'], 3)),
            'yaw': float(round(pit['yaw'], 3))
        },
        'sweep': {
            'xmin': float(round(sweep['xmin'], 3)),
            'xmax': float(round(sweep['xmax'], 3)),
            'ymin': float(round(sweep['ymin'], 3)),
            'ymax': float(round(sweep['ymax'], 3)),
            'row_gap': float(round(sweep.get('row_gap', 0.5), 3)),
            'robot_radius': float(round(sweep.get('robot_radius', 0.48), 3))
        }
    }
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    with open(filepath, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"\033[1;32m[SAVED]\033[0m Locations updated in: {filepath}")
    return data


def save_snake_waypoints_yaml(waypoints: list, filepath: str = SNAKE_WAYPOINTS_FILE):
    """Save generated snake waypoints to YAML."""
    data = {
        'num_waypoints': len(waypoints),
        'waypoints': waypoints
    }
    with open(filepath, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"\033[1;32m[SAVED]\033[0m {len(waypoints)} waypoints generated & saved to: {filepath}")


def prompt_pose(label: str, default_pose: dict = None):
    """Interactive prompt to get a 3D pose (x, y, yaw)."""
    default_pose = default_pose or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    print(f"\n--- \033[1;33mConfigure {label} Position\033[0m ---")
    print("Choose entry method:")
    print("  [1] Type X, Y, Yaw manually")
    print("  [2] Capture current robot position live from ROS TF (map -> base_link)")
    print("  [3] Click '2D Goal Pose' point in RViz")
    
    choice = input("Select [1-3] (default 1): ").strip() or "1"

    if choice == "2":
        pose = capture_tf_pose()
        if pose:
            print(f" -> Captured TF Pose for {label}: X={pose['x']}m, Y={pose['y']}m, Yaw={math.degrees(pose['yaw']):.1f}°")
            return pose
        print(" ! Could not capture TF pose. Falling back to manual entry.")
    elif choice == "3":
        pose = capture_rviz_goal_pose(prompt_label=label, timeout_sec=None)
        if pose:
            print(f" -> Captured RViz Pose for {label}: X={pose['x']}m, Y={pose['y']}m, Yaw={math.degrees(pose['yaw']):.1f}°")
            return pose
        print(" ! RViz capture cancelled. Falling back to manual entry.")

    # Manual entry fallback / option 1
    x_val = input(f"Enter {label} X (meters) [default: {default_pose['x']}]: ").strip()
    y_val = input(f"Enter {label} Y (meters) [default: {default_pose['y']}]: ").strip()
    yaw_val = input(f"Enter {label} Yaw (degrees) [default: {math.degrees(default_pose['yaw']):.1f}°]: ").strip()

    x = float(x_val) if x_val else default_pose['x']
    y = float(y_val) if y_val else default_pose['y']
    yaw_deg = float(yaw_val) if yaw_val else math.degrees(default_pose['yaw'])
    yaw_rad = math.radians(yaw_deg)

    return {'x': x, 'y': y, 'yaw': yaw_rad}


def prompt_snake_area(default_sweep: dict = None):
    """Interactive prompt to define Snake Pattern Bounds and parameters."""
    def_sw = default_sweep or {'xmin': -3.0, 'xmax': 1.5, 'ymin': -3.0, 'ymax': 0.0, 'row_gap': 0.5, 'robot_radius': 0.48}
    print(f"\n--- \033[1;33mConfigure Snake Pattern Coverage Area\033[0m ---")

    # Read map limits if my_map.yaml exists
    map_file = "/home/iedctly/my_map.yaml"
    map_limits = None
    if os.path.exists(map_file):
        try:
            with open(map_file, 'r') as f:
                meta = yaml.safe_load(f)
            from PIL import Image
            import numpy as np
            img_rel = meta['image']
            img_path = img_rel if img_rel.startswith('/') else os.path.join(os.path.dirname(map_file), img_rel)
            if os.path.exists(img_path):
                img = Image.open(img_path)
                grid = np.array(img)
                h, w = grid.shape
                res = float(meta['resolution'])
                ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
                map_limits = {
                    'xmin': float(round(ox, 2)),
                    'xmax': float(round(ox + w * res, 2)),
                    'ymin': float(round(oy, 2)),
                    'ymax': float(round(oy + h * res, 2)),
                }
                free_r, free_c = np.where(grid >= 250)
                if len(free_r) > 0:
                    map_limits['free_xmin'] = float(round(ox + free_c.min() * res + 0.48, 2))
                    map_limits['free_xmax'] = float(round(ox + free_c.max() * res - 0.48, 2))
                    map_limits['free_ymin'] = float(round(oy + (h - 1 - free_r.max()) * res + 0.48, 2))
                    map_limits['free_ymax'] = float(round(oy + (h - 1 - free_r.min()) * res - 0.48, 2))
                print(f"  \033[1;36m[MAP DETECTED]\033[0m {map_file}")
                print(f"  Map Range:       X: [{map_limits['xmin']}, {map_limits['xmax']}] m, Y: [{map_limits['ymin']}, {map_limits['ymax']}] m")
                if 'free_xmin' in map_limits:
                    print(f"  Suggested Room:  X: [{map_limits['free_xmin']}, {map_limits['free_xmax']}] m, Y: [{map_limits['free_ymin']}, {map_limits['free_ymax']}] m")
        except Exception:
            pass

    print("\nChoose area / sweep entry method:")
    print("  [1] Chain multiple Goal Poses in RViz to create Sweep \033[1;32m[Recommended]\033[0m")
    print("  [2] Click 2 opposite corners in RViz (/goal_pose)")
    print("  [3] Sweep Entire Room (Auto-detected from map)")
    print("  [4] Capture corners from live Robot position (TF)")
    print("  [5] Type Bounding Box coordinates manually (Min X, Max X, Min Y, Max Y)")

    choice = input("Select [1-5] (default 1): ").strip() or "1"

    xmin, xmax, ymin, ymax = def_sw['xmin'], def_sw['xmax'], def_sw['ymin'], def_sw['ymax']

    if choice == "1":
        print("\nHow would you like to finish chaining?")
        print("  [Enter] Free click mode (Click poses in RViz, then press [ENTER] in terminal)")
        print("  Or enter a number (e.g. 4) to automatically finish after N poses.")
        target_inp = input("Target poses count [default: free click]: ").strip()
        target_count = int(target_inp) if target_inp.isdigit() and int(target_inp) > 0 else None

        poses = capture_chained_rviz_goal_poses(target_count=target_count, prompt_label="Sweep Path")
        if not poses or len(poses) < 2:
            print("\n\033[1;33m[Notice]\033[0m Fewer than 2 goal poses captured. Falling back to default bounding box.")
        else:
            print(f"\n\033[1;32m[CHAIN COMPLETE]\033[0m Captured {len(poses)} chained goal poses:")
            for idx, p in enumerate(poses):
                print(f"   #{idx + 1}: X = {p['x']:.3f} m, Y = {p['y']:.3f} m, Yaw = {math.degrees(p['yaw']):.1f}°")

            xs = [p['x'] for p in poses]
            ys = [p['y'] for p in poses]
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)
            if math.isclose(xmin, xmax):
                xmin -= 0.5
                xmax += 0.5
            if math.isclose(ymin, ymax):
                ymin -= 0.5
                ymax += 0.5

            print(f"\nArea span: X=[{xmin:.2f}, {xmax:.2f}] (dx={xmax-xmin:.2f}m), Y=[{ymin:.2f}, {ymax:.2f}] (dy={ymax-ymin:.2f}m)")
            print("\nChoose sweep creation strategy:")
            print("  [1] Direct Chained Sweep Path (Interpolate smooth waypoints following poses 1 -> 2 -> ... -> N) \033[1;32m[Recommended]\033[0m")
            print("  [2] Snake Lawnmower Grid (Generate full back-and-forth grid covering the bounding box of these poses)")
            strat_choice = input("Select [1-2] (default 1): ").strip() or "1"

            if strat_choice == "1":
                step_inp = input(f"Enter step size along path (meters) [default: {def_sw.get('row_gap', 0.4)}]: ").strip()
                step_size = float(step_inp) if step_inp else def_sw.get('row_gap', 0.4)
                radius_inp = input(f"Enter robot clearance radius (meters) [default: {def_sw.get('robot_radius', 0.48)}]: ").strip()
                robot_radius = float(radius_inp) if radius_inp else def_sw.get('robot_radius', 0.48)

                waypoints = generate_chained_path_waypoints(poses, step_size=step_size, robot_radius=robot_radius)
                return {
                    'xmin': xmin,
                    'xmax': xmax,
                    'ymin': ymin,
                    'ymax': ymax,
                    'row_gap': step_size,
                    'robot_radius': robot_radius,
                    'custom_waypoints': waypoints,
                    'pattern_type': 'chained_poses'
                }
            else:
                row_gap_inp = input(f"Enter row gap / step size (meters) [default: {def_sw.get('row_gap', 0.5)}]: ").strip()
                radius_inp = input(f"Enter robot clearance radius (meters) [default: {def_sw.get('robot_radius', 0.48)}]: ").strip()
                row_gap = float(row_gap_inp) if row_gap_inp else def_sw.get('row_gap', 0.5)
                robot_radius = float(radius_inp) if radius_inp else def_sw.get('robot_radius', 0.48)

                waypoints = generate_snake_pattern(xmin, xmax, ymin, ymax, step_x=row_gap, row_gap=row_gap, robot_radius=robot_radius)
                return {
                    'xmin': xmin,
                    'xmax': xmax,
                    'ymin': ymin,
                    'ymax': ymax,
                    'row_gap': row_gap,
                    'robot_radius': robot_radius,
                    'custom_waypoints': waypoints,
                    'pattern_type': 'snake_grid'
                }

    elif choice == "2":
        print("\n\033[1;36mIMPORTANT:\033[0m Click TWO OPPOSITE DIAGONAL CORNERS spanning the ENTIRE cleaning area!")
        print("  - 1st Click = Bottom-Left (Min X, Min Y)")
        print("  - 2nd Click = Top-Right (Max X, Max Y)")
        print("  Listening indefinitely without timeout - take your time!\n")

        print("Click Bottom-Left corner in RViz...")
        p_bl = capture_rviz_goal_pose(prompt_label="Bottom-Left corner", timeout_sec=None)
        if p_bl:
            xmin, ymin = p_bl['x'], p_bl['y']
            print(f" -> Bottom-Left set to: X={xmin:.2f}, Y={ymin:.2f}")

        print("Click Top-Right corner in RViz...")
        p_tr = capture_rviz_goal_pose(prompt_label="Top-Right corner", timeout_sec=None)
        if p_tr:
            xmax, ymax = p_tr['x'], p_tr['y']
            print(f" -> Top-Right set to: X={xmax:.2f}, Y={ymax:.2f}")

    elif choice == "3":
        from motor_control.farm_cleaner import get_entire_room_bounds
        bounds = get_entire_room_bounds(map_file, robot_radius=0.48)
        xmin, xmax, ymin, ymax = bounds[0], bounds[1], bounds[2], bounds[3]
        print(f" -> \033[1;32m[Auto-Selected Entire Room]\033[0m Bounds: X=[{xmin:.2f}, {xmax:.2f}], Y=[{ymin:.2f}, {ymax:.2f}]")

    elif choice == "4":
        print("\nStep 1: Move robot to Bottom-Left corner (Min X, Min Y) of entire cleaning area...")
        input("Press Enter when robot is at Bottom-Left corner...")
        p_bl = capture_tf_pose()
        if p_bl:
            xmin, ymin = p_bl['x'], p_bl['y']
            print(f" -> Bottom-Left set to: X={xmin:.2f}, Y={ymin:.2f}")

        print("\nStep 2: Move robot to Top-Right corner (Max X, Max Y) of entire cleaning area...")
        input("Press Enter when robot is at Top-Right corner...")
        p_tr = capture_tf_pose()
        if p_tr:
            xmax, ymax = p_tr['x'], p_tr['y']
            print(f" -> Top-Right set to: X={xmax:.2f}, Y={ymax:.2f}")

    else:
        # Manual entry
        xmin_inp = input(f"Enter Min X (meters) [default: {xmin}]: ").strip()
        xmax_inp = input(f"Enter Max X (meters) [default: {xmax}]: ").strip()
        ymin_inp = input(f"Enter Min Y (meters) [default: {ymin}]: ").strip()
        ymax_inp = input(f"Enter Max Y (meters) [default: {ymax}]: ").strip()

        xmin = float(xmin_inp) if xmin_inp else xmin
        xmax = float(xmax_inp) if xmax_inp else xmax
        ymin = float(ymin_inp) if ymin_inp else ymin
        ymax = float(ymax_inp) if ymax_inp else ymax

    row_gap_inp = input(f"Enter row gap / step size (meters) [default: {def_sw.get('row_gap', 0.5)}]: ").strip()
    radius_inp = input(f"Enter robot clearance radius (meters) [default: {def_sw.get('robot_radius', 0.48)}]: ").strip()

    row_gap = float(row_gap_inp) if row_gap_inp else def_sw.get('row_gap', 0.5)
    robot_radius = float(radius_inp) if radius_inp else def_sw.get('robot_radius', 0.48)

    # Swap bounds if user entered backwards
    xmin, xmax = min(xmin, xmax), max(xmin, xmax)
    ymin, ymax = min(ymin, ymax), max(ymin, ymax)

    # Validate against map limits to prevent "out of map range" errors
    if map_limits:
        if xmin < map_limits['xmin'] or xmax > map_limits['xmax'] or ymin < map_limits['ymin'] or ymax > map_limits['ymax']:
            print(f"\n\033[1;31m[ERROR: OUT OF MAP RANGE]\033[0m Specified coordinates X:[{xmin:.2f}, {xmax:.2f}], Y:[{ymin:.2f}, {ymax:.2f}] "
                  f"exceed map range X:[{map_limits['xmin']}, {map_limits['xmax']}], Y:[{map_limits['ymin']}, {map_limits['ymax']}].")
            print("  -> Automatically clamping coordinates to safe map boundary...")
            xmin = max(xmin, map_limits['xmin'] + robot_radius)
            xmax = min(xmax, map_limits['xmax'] - robot_radius)
            ymin = max(ymin, map_limits['ymin'] + robot_radius)
            ymax = min(ymax, map_limits['ymax'] - robot_radius)

    dx = xmax - xmin
    dy = ymax - ymin
    sweep_axis = 'y' if dy > dx else 'x'
    sweep_span = dy if sweep_axis == 'x' else dx
    est_sweeps = max(1, int(math.ceil(sweep_span / max(0.05, row_gap)))) + 1

    if est_sweeps <= 2:
        print("\n\033[1;33m" + "!" * 70 + "\033[0m")
        print(f"\033[1;33m[WARNING] ONLY {est_sweeps} SWEEPS WILL BE GENERATED!\033[0m")
        print(f"  The span across rows is only {sweep_span:.2f} meters with a row gap of {row_gap:.2f} meters.")
        print(f"  If you want to clean a whole area (e.g. 5 to 10 sweeps), your Bottom-Left and Top-Right")
        print(f"  corners must span the full height/width of the area (e.g., Y from -3.0m to 0.0m).")
        print("\033[1;33m" + "!" * 70 + "\033[0m\n")

    return {
        'xmin': xmin,
        'xmax': xmax,
        'ymin': ymin,
        'ymax': ymax,
        'row_gap': row_gap,
        'robot_radius': robot_radius
    }


def main():
    parser = argparse.ArgumentParser(description="Single Script Configurator for Docking, Pit & Snake Pattern Positions")
    parser.add_argument('--interactive', '-i', action='store_true', help="Run interactive guided setup wizard")
    
    # Dock arguments
    parser.add_argument('--dock-x', type=float, help="Docking position X (m)")
    parser.add_argument('--dock-y', type=float, help="Docking position Y (m)")
    parser.add_argument('--dock-yaw', type=float, help="Docking orientation Yaw (rad)")

    # Pit arguments
    parser.add_argument('--pit-x', type=float, help="Pit position X (m)")
    parser.add_argument('--pit-y', type=float, help="Pit position Y (m)")
    parser.add_argument('--pit-yaw', type=float, help="Pit orientation Yaw (rad)")

    # Snake area arguments
    parser.add_argument('--xmin', type=float, help="Snake area Min X (m)")
    parser.add_argument('--xmax', type=float, help="Snake area Max X (m)")
    parser.add_argument('--ymin', type=float, help="Snake area Min Y (m)")
    parser.add_argument('--ymax', type=float, help="Snake area Max Y (m)")
    parser.add_argument('--row-gap', type=float, default=0.5, help="Snake pattern row gap (m)")
    parser.add_argument('--step-x', type=float, default=0.5, help="Snake pattern step size along rows (m)")
    parser.add_argument('--robot-radius', type=float, default=0.48, help="Robot clearance radius (m)")

    # Chained Goal Pose arguments
    parser.add_argument('--chain-poses', action='store_true', help="Record chained goal poses from RViz for sweep")
    parser.add_argument('--num-poses', type=int, default=None, help="Target number of goal poses for chaining")
    parser.add_argument('--chain-mode', type=str, choices=['direct', 'grid'], default='direct', help="Sweep strategy for chained poses ('direct' path or 'grid')")

    args = parser.parse_args()

    # Read current locations if file exists
    curr_dock = {'x': -0.19, 'y': -3.041, 'yaw': 1.542}
    curr_pit = {'x': 2.682, 'y': 0.237, 'yaw': -0.309}
    curr_sweep = {'xmin': -3.0, 'xmax': 1.5, 'ymin': -3.0, 'ymax': 0.0, 'row_gap': 0.5, 'robot_radius': 0.48}

    if os.path.exists(LOCATIONS_FILE):
        try:
            with open(LOCATIONS_FILE, 'r') as f:
                d = yaml.safe_load(f) or {}
                if 'dock' in d: curr_dock = d['dock']
                if 'pit' in d: curr_pit = d['pit']
                if 'sweep' in d: curr_sweep = d['sweep']
        except Exception:
            pass

    print("\033[1;34m====================================================================\033[0m")
    print("\033[1;32m    UNIFIED ROBOT POSITIONS & SNAKE PATTERN CONFIGURATOR        \033[0m")
    print("\033[1;34m====================================================================\033[0m")

    # Determine mode: CLI arguments passed vs Interactive Wizard
    cli_mode = any([
        args.dock_x is not None, args.dock_y is not None,
        args.pit_x is not None, args.pit_y is not None,
        args.xmin is not None, args.xmax is not None,
        args.ymin is not None, args.ymax is not None,
        args.chain_poses
    ])

    custom_waypoints = None
    pattern_type = 'snake_grid'

    if cli_mode:
        dock = {
            'x': args.dock_x if args.dock_x is not None else curr_dock['x'],
            'y': args.dock_y if args.dock_y is not None else curr_dock['y'],
            'yaw': args.dock_yaw if args.dock_yaw is not None else curr_dock['yaw']
        }
        pit = {
            'x': args.pit_x if args.pit_x is not None else curr_pit['x'],
            'y': args.pit_y if args.pit_y is not None else curr_pit['y'],
            'yaw': args.pit_yaw if args.pit_yaw is not None else curr_pit['yaw']
        }
        sweep = {
            'xmin': args.xmin if args.xmin is not None else curr_sweep['xmin'],
            'xmax': args.xmax if args.xmax is not None else curr_sweep['xmax'],
            'ymin': args.ymin if args.ymin is not None else curr_sweep['ymin'],
            'ymax': args.ymax if args.ymax is not None else curr_sweep['ymax'],
            'row_gap': args.row_gap,
            'robot_radius': args.robot_radius
        }
        step_x = args.step_x

        if args.chain_poses:
            poses = capture_chained_rviz_goal_poses(target_count=args.num_poses, prompt_label="CLI Sweep Path")
            if poses and len(poses) >= 2:
                xs = [p['x'] for p in poses]
                ys = [p['y'] for p in poses]
                xmin, xmax = min(xs), max(xs)
                ymin, ymax = min(ys), max(ys)
                if math.isclose(xmin, xmax):
                    xmin -= 0.5
                    xmax += 0.5
                if math.isclose(ymin, ymax):
                    ymin -= 0.5
                    ymax += 0.5
                sweep['xmin'] = xmin
                sweep['xmax'] = xmax
                sweep['ymin'] = ymin
                sweep['ymax'] = ymax

                if args.chain_mode == 'grid':
                    pattern_type = 'snake_grid'
                    custom_waypoints = generate_snake_pattern(xmin, xmax, ymin, ymax, step_x=args.step_x, row_gap=args.row_gap, robot_radius=args.robot_radius)
                else:
                    pattern_type = 'chained_poses'
                    custom_waypoints = generate_chained_path_waypoints(poses, step_size=args.row_gap, robot_radius=args.robot_radius)
    else:
        dock = prompt_pose("Docking (Initial Pose)", curr_dock)
        pit = prompt_pose("Pit Location", curr_pit)
        sweep = prompt_snake_area(curr_sweep)
        step_x = sweep.get('row_gap', 0.5)
        custom_waypoints = sweep.pop('custom_waypoints', None)
        pattern_type = sweep.pop('pattern_type', 'snake_grid')

    # 1. Save locations.yaml
    saved_data = save_locations_yaml(dock, pit, sweep, LOCATIONS_FILE)

    # 2. Generate and save Snake Waypoints
    if custom_waypoints:
        waypoints = custom_waypoints
    else:
        waypoints = generate_snake_pattern(
            xmin=sweep['xmin'],
            xmax=sweep['xmax'],
            ymin=sweep['ymin'],
            ymax=sweep['ymax'],
            step_x=step_x,
            row_gap=sweep['row_gap'],
            robot_radius=sweep.get('robot_radius', 0.48)
        )
    save_snake_waypoints_yaml(waypoints, SNAKE_WAYPOINTS_FILE)

    # 3. Print complete configuration summary
    strategy_label = "Direct Chained Path (Waypoints Sequence)" if pattern_type == 'chained_poses' else "Snake Lawnmower Grid"
    print("\n\033[1;36m====================================================================\033[0m")
    print("\033[1;36m               CONFIGURED POSITIONS SUMMARY TABLE                   \033[0m")
    print("\033[1;36m====================================================================\033[0m")
    print(f"  \033[1;33mDocking (Initial Position):\033[0m X = {dock['x']:.3f} m, Y = {dock['y']:.3f} m, Yaw = {math.degrees(dock['yaw']):.1f}°")
    print(f"  \033[1;33mPit Location:\033[0m               X = {pit['x']:.3f} m, Y = {pit['y']:.3f} m, Yaw = {math.degrees(pit['yaw']):.1f}°")
    print(f"  \033[1;33mSnake Pattern Area Bounds:\033[0m   X: [{sweep['xmin']:.2f}, {sweep['xmax']:.2f}] m, Y: [{sweep['ymin']:.2f}, {sweep['ymax']:.2f}] m")
    print(f"  \033[1;33mSweep Creation Strategy:\033[0m     {strategy_label}")
    print(f"  \033[1;33mGrid / Step Resolution:\033[0m     Row Gap = {sweep['row_gap']:.2f} m, Step = {step_x:.2f} m, Clearance = {sweep['robot_radius']:.2f} m")
    print(f"  \033[1;32mTotal Waypoints Generated:\033[0m  {len(waypoints)} waypoints")
    print("\033[1;36m====================================================================\033[0m\n")


if __name__ == '__main__':
    main()
