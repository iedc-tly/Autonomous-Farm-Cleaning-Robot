#!/usr/bin/env bash
# ==============================================================================
# run_teleop.sh
# 
# Standalone Keyboard Teleoperation Bringup Script
# Launches motor_node hardware driver if not already running, then starts teleop.
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_PATH="/opt/ros/humble"

# Source ROS 2 and Workspace
if [ -f "$ROS_DISTRO_PATH/setup.bash" ]; then
    source "$ROS_DISTRO_PATH/setup.bash"
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

echo -e "\033[1;34m====================================================================\033[0m"
echo -e "\033[1;32m                 KEYBOARD TELEOPERATION CONTROL                     \033[0m"
echo -e "\033[1;34m====================================================================\033[0m"

HW_PID=""

cleanup() {
    if [ -n "$HW_PID" ]; then
        echo -e "\n\033[1;33m[SHUTDOWN]\033[0m Stopping hardware driver nodes..."
        ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 || true
        kill -SIGINT "$HW_PID" 2>/dev/null || true
        sleep 1
        pkill -9 -f "motor_node" 2>/dev/null || true
        pkill -9 -f "encoder_odom_node" 2>/dev/null || true
    fi
}

trap cleanup SIGINT SIGTERM EXIT

# Kill any lingering background hardware nodes to load updated build
pkill -9 -f "hardware_bringup.launch.py" 2>/dev/null || true
pkill -9 -f "motor_node" 2>/dev/null || true
pkill -9 -f "encoder_odom_node" 2>/dev/null || true
sleep 1

echo -e "\033[1;36m[INFO]\033[0m Starting hardware driver nodes..."
ros2 launch motor_control hardware_bringup.launch.py > /tmp/hardware_bringup.log 2>&1 &
HW_PID=$!
echo "       -> Hardware bringup started (PID: $HW_PID, log: /tmp/hardware_bringup.log)"
sleep 3

echo -e "\033[1;32m  Use WASD / Arrow Keys to drive. Press [ Ctrl+C ] to exit safely.  \033[0m"
echo -e "\033[1;32m--------------------------------------------------------------------\033[0m"

ros2 run motor_control teleop
