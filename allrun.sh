#!/usr/bin/env bash
# ==============================================================================
# allrun.sh - Autonomous Farm Cleaning Robot Master All-In-One Launcher
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_PATH="/opt/ros/humble"

# Source ROS 2 Humble
if [ -f "$ROS_DISTRO_PATH/setup.bash" ]; then
    source "$ROS_DISTRO_PATH/setup.bash"
else
    echo -e "\033[1;31m[ERROR]\033[0m ROS 2 Humble not found at $ROS_DISTRO_PATH"
    exit 1
fi

# Source workspace install
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# Ensure pigpiod daemon is running for hardware PWM
if ! pgrep -x pigpiod >/dev/null 2>&1; then
    echo -e "\033[1;36m[Hardware]\033[0m Starting pigpio daemon..."
    sudo systemctl start pigpiod 2>/dev/null || sudo pigpiod 2>/dev/null || true
    sleep 0.5
fi

# Execute master allrun pipeline python script passing any CLI arguments
exec python3 "$WS_DIR/allrun.py" "$@"
