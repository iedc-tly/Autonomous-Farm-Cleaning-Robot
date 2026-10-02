#!/usr/bin/env python3
"""
set_speed.py

Single-command tool to inspect and adjust linear & angular speeds across the
entire Autonomous Farm Cleaner ROS 2 stack:
  1. Nav2 Parameter Config (nav2_params.yaml)
  2. Master Orchestrator Pipeline (allrun.py)
  3. Keyboard Teleoperation (teleop_node.py)
  4. App Bridge & WebSockets (rover_bridge_node.py)
  5. Auto Global Localizer (auto_global_localizer.py)
  6. Live ROS 2 Parameters (if nodes are running)
"""

import os
import sys
import re
import argparse
import subprocess

WS_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if not os.path.exists(os.path.join(WS_DIR, "src")):
    # Fallback to current working directory or /home/iedctly/ros2_ws
    WS_DIR = "/home/iedctly/ros2_ws"

NAV2_PARAMS_FILE = os.path.join(WS_DIR, "src/motor_control/config/nav2_params.yaml")
ALLRUN_FILE = os.path.join(WS_DIR, "allrun.py")
TELEOP_FILE = os.path.join(WS_DIR, "src/motor_control/motor_control/teleop_node.py")
BRIDGE_FILE = os.path.join(WS_DIR, "src/rover_app_bridge/rover_app_bridge/rover_bridge_node.py")
LOCALIZER_FILE = os.path.join(WS_DIR, "src/motor_control/motor_control/auto_global_localizer.py")


def get_current_speeds():
    speeds = {
        "nav2_linear": None,
        "nav2_angular": None,
        "allrun_linear": None,
        "allrun_angular": None,
        "teleop_linear": None,
        "teleop_angular": None,
    }

    if os.path.exists(NAV2_PARAMS_FILE):
        with open(NAV2_PARAMS_FILE, "r") as f:
            content = f.read()
            m_lin = re.search(r"desired_linear_vel:\s*([0-9.]+)", content)
            m_ang = re.search(r"rotate_to_heading_angular_vel:\s*([0-9.]+)", content)
            if m_lin:
                speeds["nav2_linear"] = float(m_lin.group(1))
            if m_ang:
                speeds["nav2_angular"] = float(m_ang.group(1))

    if os.path.exists(ALLRUN_FILE):
        with open(ALLRUN_FILE, "r") as f:
            content = f.read()
            m_lin = re.search(r"self\.linear_speed\s*=\s*([0-9.]+)", content)
            m_ang = re.search(r"self\.angular_speed\s*=\s*([0-9.]+)", content)
            if m_lin:
                speeds["allrun_linear"] = float(m_lin.group(1))
            if m_ang:
                speeds["allrun_angular"] = float(m_ang.group(1))

    if os.path.exists(TELEOP_FILE):
        with open(TELEOP_FILE, "r") as f:
            content = f.read()
            m_lin = re.search(r"self\.linear_speed\s*=\s*([0-9.]+)", content)
            m_ang = re.search(r"self\.angular_speed\s*=\s*([0-9.]+)", content)
            if m_lin:
                speeds["teleop_linear"] = float(m_lin.group(1))
            if m_ang:
                speeds["teleop_angular"] = float(m_ang.group(1))

    return speeds


def display_current_speeds(speeds):
    print("\n\033[1;34m===================================================================\033[0m")
    print("\033[1;32m           CURRENT CONFIGURED ROBOT SPEEDS                        \033[0m")
    print("\033[1;34m===================================================================\033[0m")
    print(f"  • Nav2 Controller (RPP) : Linear = \033[1;33m{speeds['nav2_linear']} m/s\033[0m  | Angular = \033[1;33m{speeds['nav2_angular']} rad/s\033[0m")
    print(f"  • Master Allrun Script  : Linear = \033[1;33m{speeds['allrun_linear']} m/s\033[0m  | Angular = \033[1;33m{speeds['allrun_angular']} rad/s\033[0m")
    print(f"  • Keyboard Teleop Node  : Linear = \033[1;33m{speeds['teleop_linear']} m/s\033[0m  | Angular = \033[1;33m{speeds['teleop_angular']} rad/s\033[0m")
    print("\033[1;34m===================================================================\033[0m\n")


def update_nav2_params(linear_speed: float, angular_speed: float):
    if not os.path.exists(NAV2_PARAMS_FILE):
        print(f"\033[1;31m[ERROR]\033[0m File not found: {NAV2_PARAMS_FILE}")
        return False

    with open(NAV2_PARAMS_FILE, "r") as f:
        content = f.read()

    # Derived values
    approach_speed = round(min(0.02, linear_speed * 0.5), 3)
    min_angular = round(max(0.02, angular_speed * 0.33), 3)
    # Give enough time for a 180° rotation at slow angular speed plus buffer
    time_allowance = max(20.0, round((3.14159 / max(0.05, angular_speed)) + 10.0, 1))

    # Regulated Pure Pursuit Controller
    content = re.sub(r"desired_linear_vel:\s*[0-9.]+", f"desired_linear_vel: {linear_speed}", content)
    content = re.sub(r"rotate_to_heading_angular_vel:\s*[0-9.]+", f"rotate_to_heading_angular_vel: {angular_speed}", content)
    content = re.sub(r"min_approach_linear_velocity:\s*[0-9.]+", f"min_approach_linear_velocity: {approach_speed}", content)
    content = re.sub(r"regulated_linear_scaling_min_speed:\s*[0-9.]+", f"regulated_linear_scaling_min_speed: {approach_speed}", content)
    content = re.sub(r"movement_time_allowance:\s*[0-9.]+", f"movement_time_allowance: {time_allowance}", content)

    # Behavior server
    content = re.sub(r"max_rotational_vel:\s*[0-9.]+", f"max_rotational_vel: {angular_speed}", content)
    content = re.sub(r"min_rotational_vel:\s*[0-9.]+", f"min_rotational_vel: {min_angular}", content)

    # Velocity smoother limits
    content = re.sub(r"max_velocity:\s*\[[0-9., -]+\]", f"max_velocity: [{linear_speed}, 0.0, {angular_speed}]", content)
    content = re.sub(r"min_velocity:\s*\[[0-9., -]+\]", f"min_velocity: [-{linear_speed}, 0.0, -{angular_speed}]", content)

    with open(NAV2_PARAMS_FILE, "w") as f:
        f.write(content)
    print(f"  \033[1;32m✓\033[0m Updated {os.path.basename(NAV2_PARAMS_FILE)}")
    return True


def update_allrun_file(linear_speed: float, angular_speed: float):
    if not os.path.exists(ALLRUN_FILE):
        return False

    with open(ALLRUN_FILE, "r") as f:
        content = f.read()

    content = re.sub(r"self\.linear_speed\s*=\s*[0-9.]+", f"self.linear_speed = {linear_speed}", content, count=1)
    content = re.sub(r"self\.angular_speed\s*=\s*[0-9.]+", f"self.angular_speed = {angular_speed}", content, count=1)

    with open(ALLRUN_FILE, "w") as f:
        f.write(content)
    print(f"  \033[1;32m✓\033[0m Updated {os.path.basename(ALLRUN_FILE)}")
    return True


def update_teleop_file(linear_speed: float, angular_speed: float):
    if not os.path.exists(TELEOP_FILE):
        return False

    with open(TELEOP_FILE, "r") as f:
        content = f.read()

    content = re.sub(r"self\.linear_speed\s*=\s*[0-9.]+", f"self.linear_speed = {linear_speed}", content, count=1)
    content = re.sub(r"self\.angular_speed\s*=\s*[0-9.]+", f"self.angular_speed = {angular_speed}", content, count=1)

    with open(TELEOP_FILE, "w") as f:
        f.write(content)
    print(f"  \033[1;32m✓\033[0m Updated {os.path.basename(TELEOP_FILE)}")
    return True


def update_bridge_file(linear_speed: float, angular_speed: float):
    if not os.path.exists(BRIDGE_FILE):
        return False

    with open(BRIDGE_FILE, "r") as f:
        content = f.read()

    content = re.sub(
        r"mode in \('forward', 'up', 'w'\):\n\s*return\s*[0-9.]+\s*\*\s*speed_scale,\s*0\.0",
        f"mode in ('forward', 'up', 'w'):\n        return {linear_speed} * speed_scale, 0.0",
        content
    )
    content = re.sub(
        r"mode in \('backward', 'back', 'down', 's'\):\n\s*return\s*-[0-9.]+\s*\*\s*speed_scale,\s*0\.0",
        f"mode in ('backward', 'back', 'down', 's'):\n        return -{linear_speed} * speed_scale, 0.0",
        content
    )
    content = re.sub(
        r"mode in \('left', 'a'\):\n\s*return\s*0\.0,\s*[0-9.]+\s*\*\s*speed_scale",
        f"mode in ('left', 'a'):\n        return 0.0, {angular_speed} * speed_scale",
        content
    )
    content = re.sub(
        r"mode in \('right', 'd'\):\n\s*return\s*0\.0,\s*-[0-9.]+\s*\*\s*speed_scale",
        f"mode in ('right', 'd'):\n        return 0.0, -{angular_speed} * speed_scale",
        content
    )
    content = re.sub(
        r"_clamp\(linear_x \* speed_scale, -[0-9.]+, [0-9.]+\)",
        f"_clamp(linear_x * speed_scale, -{linear_speed}, {linear_speed})",
        content
    )
    content = re.sub(
        r"_clamp\(angular_z \* speed_scale, -[0-9.]+, [0-9.]+\)",
        f"_clamp(angular_z * speed_scale, -{angular_speed}, {angular_speed})",
        content
    )

    with open(BRIDGE_FILE, "w") as f:
        f.write(content)
    print(f"  \033[1;32m✓\033[0m Updated {os.path.basename(BRIDGE_FILE)}")
    return True


def update_localizer_file(angular_speed: float):
    if not os.path.exists(LOCALIZER_FILE):
        return False

    with open(LOCALIZER_FILE, "r") as f:
        content = f.read()

    content = re.sub(
        r"angular_speed:\s*float\s*=\s*[0-9.]+",
        f"angular_speed: float = {angular_speed}",
        content
    )
    content = re.sub(
        r"parser\.add_argument\('--speed',\s*'-s',\s*type=float,\s*default=[0-9.]+",
        f"parser.add_argument('--speed', '-s', type=float, default={angular_speed}",
        content
    )

    with open(LOCALIZER_FILE, "w") as f:
        f.write(content)
    print(f"  \033[1;32m✓\033[0m Updated {os.path.basename(LOCALIZER_FILE)}")
    return True


def update_live_ros2_params(linear_speed: float, angular_speed: float):
    print("\n\033[1;36m[Live ROS 2]\033[0m Checking for active running ROS 2 nodes...")
    res = subprocess.run("ros2 node list 2>/dev/null", shell=True, capture_output=True, text=True)
    active_nodes = res.stdout.splitlines()

    has_controller = any("controller_server" in n for n in active_nodes)
    has_smoother = any("velocity_smoother" in n for n in active_nodes)
    has_motor = any("motor_node" in n for n in active_nodes)

    if not (has_controller or has_smoother or has_motor):
        print("  \033[1;33mℹ\033[0m No active Nav2 / motor nodes found. Speeds will apply on next launch.")
        return

    approach_speed = round(min(0.02, linear_speed * 0.5), 3)

    if has_controller:
        print("  -> Updating live /controller_server parameters...")
        subprocess.run(f"ros2 param set /controller_server FollowPath.desired_linear_vel {linear_speed} >/dev/null 2>&1", shell=True)
        subprocess.run(f"ros2 param set /controller_server FollowPath.rotate_to_heading_angular_vel {angular_speed} >/dev/null 2>&1", shell=True)
        subprocess.run(f"ros2 param set /controller_server FollowPath.min_approach_linear_velocity {approach_speed} >/dev/null 2>&1", shell=True)
        subprocess.run(f"ros2 param set /controller_server FollowPath.regulated_linear_scaling_min_speed {approach_speed} >/dev/null 2>&1", shell=True)
        print("     \033[1;32m✓\033[0m /controller_server parameters applied live.")

    if has_smoother:
        print("  -> Updating live /velocity_smoother parameters...")
        subprocess.run(f"ros2 param set /velocity_smoother max_velocity '[{linear_speed}, 0.0, {angular_speed}]' >/dev/null 2>&1", shell=True)
        subprocess.run(f"ros2 param set /velocity_smoother min_velocity '[{-linear_speed}, 0.0, {-angular_speed}]' >/dev/null 2>&1", shell=True)
        print("     \033[1;32m✓\033[0m /velocity_smoother parameters applied live.")

    if has_motor:
        print("  -> Updating live /motor_node ceiling parameters...")
        subprocess.run(f"ros2 param set /motor_node max_linear {max(0.40, linear_speed + 0.05)} >/dev/null 2>&1", shell=True)
        subprocess.run(f"ros2 param set /motor_node max_angular {max(0.60, angular_speed + 0.10)} >/dev/null 2>&1", shell=True)
        print("     \033[1;32m✓\033[0m /motor_node parameters applied live.")


def build_workspace():
    print("\n\033[1;36m[Build]\033[0m Running colcon build --symlink-install...")
    cmd = "colcon build --symlink-install --packages-select motor_control rover_app_bridge"
    proc = subprocess.run(cmd, shell=True, cwd=WS_DIR)
    if proc.returncode == 0:
        print("  \033[1;32m✓\033[0m Workspace packages built and synchronized successfully.")
    else:
        print("  \033[1;31m✗\033[0m colcon build encountered an error.")


def main():
    parser = argparse.ArgumentParser(description="View and easily adjust robot linear and angular speeds across ROS 2.")
    parser.add_argument("linear", nargs="?", type=float, help="Desired linear speed in m/s (e.g. 0.05)")
    parser.add_argument("angular", nargs="?", type=float, help="Desired angular speed in rad/s (e.g. 0.15)")
    parser.add_argument("--linear", "-l", dest="opt_linear", type=float, help="Desired linear speed (m/s)")
    parser.add_argument("--angular", "-a", dest="opt_angular", type=float, help="Desired angular speed (rad/s)")
    parser.add_argument("--show", "-s", action="store_true", help="Display current speeds and exit")
    parser.add_argument("--no-build", action="store_true", help="Skip colcon build")
    args = parser.parse_args()

    current = get_current_speeds()

    if args.show:
        display_current_speeds(current)
        return

    lin = args.opt_linear if args.opt_linear is not None else args.linear
    ang = args.opt_angular if args.opt_angular is not None else args.angular

    # If neither argument provided, prompt interactively
    if lin is None and ang is None:
        display_current_speeds(current)
        default_lin = current.get("nav2_linear") or 0.05
        default_ang = current.get("nav2_angular") or 0.15

        try:
            inp_lin = input(f"Enter new linear speed in m/s [current: {default_lin}]: ").strip()
            lin = float(inp_lin) if inp_lin else default_lin

            inp_ang = input(f"Enter new angular speed in rad/s [current: {default_ang}]: ").strip()
            ang = float(inp_ang) if inp_ang else default_ang
        except (ValueError, KeyboardInterrupt):
            print("\nCancelled.")
            sys.exit(0)
    elif lin is None:
        lin = current.get("nav2_linear") or 0.05
    elif ang is None:
        ang = current.get("nav2_angular") or 0.15

    # Validation
    if lin <= 0.0 or lin > 2.0:
        print(f"\033[1;31m[ERROR]\033[0m Linear speed {lin} m/s out of sensible range (0.01 - 2.0 m/s).")
        sys.exit(1)
    if ang <= 0.0 or ang > 3.5:
        print(f"\033[1;31m[ERROR]\033[0m Angular speed {ang} rad/s out of sensible range (0.02 - 3.5 rad/s).")
        sys.exit(1)

    lin = round(lin, 3)
    ang = round(ang, 3)

    print(f"\n\033[1;36m[APPLYING NEW SPEEDS]\033[0m Linear: \033[1;32m{lin} m/s\033[0m | Angular: \033[1;32m{ang} rad/s\033[0m ({ang * 57.2958:.1f}°/s)")

    print("\n\033[1;36m[Updating Configuration Files]\033[0m")
    update_nav2_params(lin, ang)
    update_allrun_file(lin, ang)
    update_teleop_file(lin, ang)
    update_bridge_file(lin, ang)
    update_localizer_file(ang)

    update_live_ros2_params(lin, ang)

    if not args.no_build:
        build_workspace()

    print("\n\033[1;32m===================================================================\033[0m")
    print(f" Speed configuration updated successfully: {lin} m/s linear, {ang} rad/s angular.")
    print("\033[1;32m===================================================================\033[0m\n")


if __name__ == "__main__":
    main()
