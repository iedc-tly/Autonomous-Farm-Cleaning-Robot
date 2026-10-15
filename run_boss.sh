#!/usr/bin/env bash
# ==============================================================================
# run_boss.sh
# 
# Master Boot & Bringup Script for Autonomous Farm Cleaner
# Boss Orchestrator, App Bridge (HTTP :8000), and Rosbridge (:9090)
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Ensure standard environment
ROBOT_HOME="${ROBOT_HOME:-${HOME:-$HOME}}"
export HOME="${ROBOT_HOME:-~}"
export ROS_LOG_DIR="/tmp"
export PYTHONUNBUFFERED=1

# 1. Source ROS 2 Humble base
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
else
    echo "[ERROR] ROS 2 Humble not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# 2. Source workspace install if available
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    if [ -f "$WS_DIR/install/local_setup.bash" ]; then
        source "$WS_DIR/install/local_setup.bash"
    fi
fi

# 3. Ensure FarmRover-Hotspot is active on boot if configured
if command -v nmcli >/dev/null 2>&1 && nmcli connection show "FarmRover-Hotspot" >/dev/null 2>&1; then
    if ! nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "FarmRover-Hotspot"; then
        echo "[Network] Activating FarmRover-Hotspot on boot..."
        sudo -n nmcli connection up "FarmRover-Hotspot" 2>/dev/null || nmcli connection up "FarmRover-Hotspot" 2>/dev/null || true
    fi
fi

# 4. Ensure pigpio daemon is running for hardware PWM / motors
if ! pgrep -x pigpiod >/dev/null 2>&1; then
    echo "[Hardware] Starting pigpio daemon..."
    sudo -n systemctl start pigpiod 2>/dev/null || sudo -n /usr/local/bin/pigpiod -l 2>/dev/null || pigpiod 2>/dev/null || true
    sleep 0.5
fi

# 5. Launch rosbridge_websocket (:9090) bound to localhost (127.0.0.1) if not already running
if ! pgrep -f "rosbridge_websocket" >/dev/null 2>&1; then
    echo "[App Bridge] Launching rosbridge_websocket (127.0.0.1:9090)..."
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1 >/tmp/rosbridge.log 2>&1 &
fi

# Check if an instance of rover_bridge_node is already running
if pgrep -f "rover_bridge_node.py" >/dev/null 2>&1; then
    echo "[INFO] rover_bridge_node is already running. Exiting redundant launch."
    exit 0
fi

# 6. Print Header
ROBOT_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "===================================================================="
echo "   AUTONOMOUS FARM CLEANER - BOSS ORCHESTRATOR & APP BRIDGE SERVER   "
echo "===================================================================="
echo "  • Mobile Web App  : http://${ROBOT_IP:-10.42.0.1}:8000"
echo "  • REST / WS API   : http://${ROBOT_IP:-10.42.0.1}:8000/api"
echo "  • Rosbridge Port  : ws://127.0.0.1:9090 (localhost only)"
echo "===================================================================="

# 7. Launch Boss Server via exec so signals (SIGTERM/SIGINT) pass cleanly
NODE_SCRIPT="$WS_DIR/src/rover_app_bridge/rover_app_bridge/rover_bridge_node.py"

if [ -f "$NODE_SCRIPT" ]; then
    exec python3 "$NODE_SCRIPT"
else
    exec ros2 run rover_app_bridge rover_bridge
fi
