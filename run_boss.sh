#!/usr/bin/env bash
# ==============================================================================
# run_boss.sh
# 
# Bringup script for Autonomous Farm Cleaner Boss Orchestrator & App Bridge
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Source ROS 2 Humble base
source /opt/ros/humble/setup.bash

# 2. Build workspace if needed
if [ ! -d "$WS_DIR/install" ] || [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo "[INFO] Workspace install directory missing. Building workspace..."
    (cd "$WS_DIR" && colcon build --symlink-install)
fi

source "$WS_DIR/install/setup.bash"
if [ -f "$WS_DIR/install/local_setup.bash" ]; then
    source "$WS_DIR/install/local_setup.bash"
fi

# 3. Print Header
echo -e "\033[1;34m====================================================================\033[0m"
echo -e "\033[1;32m   AUTONOMOUS FARM CLEANER - BOSS ORCHESTRATOR & APP BRIDGE SERVER   \033[0m"
echo -e "\033[1;34m====================================================================\033[0m"

# 4. Launch Boss Server (runs Python node directly to bypass shell index cache issues)
NODE_SCRIPT="$WS_DIR/install/rover_app_bridge/lib/python3.10/site-packages/rover_app_bridge/rover_bridge_node.py"

if [ -f "$NODE_SCRIPT" ]; then
    python3 "$NODE_SCRIPT"
else
    ros2 run rover_app_bridge rover_bridge
fi
