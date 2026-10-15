#!/usr/bin/env bash
# ==============================================================================
# start_hotspot.sh
# 
# Activates the FarmRover Wi-Fi Hotspot on wlan0.
# ==============================================================================

set -e

CON_NAME="FarmRover-Hotspot"

# If profile doesn't exist yet, run configure_hotspot.sh first
if ! nmcli connection show "$CON_NAME" >/dev/null 2>&1; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    bash "$SCRIPT_DIR/configure_hotspot.sh"
fi

echo "[Hotspot] Bringing up $CON_NAME on wlan0..."
sudo nmcli connection up "$CON_NAME"

echo ""
echo "=================================================="
echo " FarmRover Hotspot is ACTIVE!                     "
echo "  • Connect phone/laptop Wi-Fi to: FarmRover      "
echo "  • Password: farmrover123                        "
echo "  • Open Web Controller: http://10.42.0.1:8000    "
echo "=================================================="
