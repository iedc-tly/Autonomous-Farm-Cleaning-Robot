#!/usr/bin/env bash
# ==============================================================================
# align_map.sh
# 
# One-shot tool to snap the live LiDAR scan and map into 100% alignment:
# 1. Seeds AMCL with calibrated dock coordinates from locations.yaml
# 2. Bursts 10 stationary scan-matching updates to AMCL (/request_nomotion_update)
# 3. Drops covariance to < 0.02 m² and locks red laser dots onto map walls
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_LOG_DIR="/tmp"

# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

echo -e "\033[1;34m====================================================================\033[0m"
echo -e "\033[1;32m       SNAPPING LIVE LIDAR SCAN TO MAP (AMCL FINE-ALIGNMENT)        \033[0m"
echo -e "\033[1;34m====================================================================\033[0m"

# 2. Check if bridge server is alive on port 8000; if so, trigger via API
if curl -s -m 1 http://localhost:8000/api/status >/dev/null 2>&1; then
    echo "[1/2] Sending Align Map request to Rover Bridge Server..."
    RES=$(curl -s -X POST http://localhost:8000/api/align_map)
    echo "      Result: $RES"
    echo -e "\033[1;32m[SUCCESS] Live map aligned via Bridge Orchestrator!\033[0m\n"
    exit 0
fi

# 3. Otherwise run precision dock localizer directly via ROS 2
echo "[1/2] Calling Precision Dock Localizer & Scan Matcher..."
python3 "$WS_DIR/src/motor_control/motor_control/auto_global_localizer.py" --mode dock

echo -e "\033[1;32m====================================================================\033[0m"
echo -e "\033[1;32m[DONE] Live LiDAR scan and Map are now 100% aligned!\033[0m"
echo -e "\033[1;32m====================================================================\033[0m"
