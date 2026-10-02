#!/usr/bin/env bash
# ==============================================================================
# set_speed.sh
# 
# Fast Single-Command Utility to view & edit robot speeds:
#   Usage:
#     ./set_speed.sh                        # Interactive prompt
#     ./set_speed.sh --show                 # View current speeds
#     ./set_speed.sh 0.05 0.15              # Set linear=0.05 m/s, angular=0.15 rad/s
#     ./set_speed.sh -l 0.10 -a 0.25        # Set with flags
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_PATH="/opt/ros/humble"

# Source ROS 2 base and workspace if available
if [ -f "$ROS_DISTRO_PATH/setup.bash" ]; then
    source "$ROS_DISTRO_PATH/setup.bash"
fi
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

exec python3 "$WS_DIR/src/motor_control/motor_control/set_speed.py" "$@"
