#!/usr/bin/env bash
# ==============================================================================
# stop_hotspot.sh
# 
# Deactivates the Hotspot and reconnects wlan0 to the default lab/home Wi-Fi.
# ==============================================================================

set -e

CON_NAME="FarmRover-Hotspot"
WIFI_NAME="${1:-IEDC CETLY}"

echo "[Hotspot] Bringing down $CON_NAME..."
sudo nmcli connection down "$CON_NAME" 2>/dev/null || true

if nmcli connection show "$WIFI_NAME" >/dev/null 2>&1; then
    echo "[Wi-Fi] Reconnecting to '$WIFI_NAME'..."
    sudo nmcli connection up "$WIFI_NAME"
fi

echo ""
echo "=================================================="
echo " Hotspot deactivated. Reconnected to '$WIFI_NAME'."
echo " Current IP: $(hostname -I | awk '{print $1}')"
echo "=================================================="
