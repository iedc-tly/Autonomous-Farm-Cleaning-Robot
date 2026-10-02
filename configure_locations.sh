#!/usr/bin/env bash
# ==============================================================================
# configure_locations.sh
# 
# Unified Single-Script Tool to configure:
#   1. Docking Position (Initial Position)
#   2. Pit Position
#   3. Snake Pattern Coverage Area & Grid Waypoints
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_PATH="/opt/ros/humble"

# Source ROS 2 base and workspace
source "$ROS_DISTRO_PATH/setup.bash"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

python3 "$WS_DIR/src/motor_control/motor_control/configure_locations.py" "$@"
