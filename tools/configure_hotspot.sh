#!/usr/bin/env bash
# ==============================================================================
# configure_hotspot.sh
# 
# Configures a persistent Wi-Fi Hotspot profile ("FarmRover-Hotspot") using
# NetworkManager (nmcli) that starts AUTOMATICALLY when the Raspberry Pi boots.
# ==============================================================================

set -e

HOTSPOT_SSID="${1:-FarmRover}"
HOTSPOT_PASS="${2:-farmrover123}"
CON_NAME="FarmRover-Hotspot"
IFACE="wlan0"

echo "=================================================="
echo " CONFIGURING AUTOSTART WI-FI HOTSPOT (BOOT AP)   "
echo "=================================================="
echo "  • Interface : $IFACE"
echo "  • SSID      : $HOTSPOT_SSID"
echo "  • Password  : $HOTSPOT_PASS"
echo "  • Autostart : YES (Top Priority on switch-on)"
echo "  • IP Subnet : 10.42.0.1 / 24"
echo "  • Web App   : http://10.42.0.1:8000"
echo "=================================================="

# Check if connection already exists; remove if so
if nmcli connection show "$CON_NAME" >/dev/null 2>&1; then
    echo "[1/4] Replacing previous '$CON_NAME' profile..."
    sudo nmcli connection delete "$CON_NAME"
fi

# Add new Wi-Fi AP profile with autoconnect enabled
echo "[2/4] Creating Hotspot connection '$CON_NAME'..."
sudo nmcli connection add \
    type wifi \
    ifname "$IFACE" \
    con-name "$CON_NAME" \
    autoconnect yes \
    ssid "$HOTSPOT_SSID"

echo "[3/4] Setting WPA2-PSK security & DHCP server (shared)..."
sudo nmcli connection modify "$CON_NAME" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$HOTSPOT_PASS" \
    ipv4.method shared \
    ipv4.addresses 10.42.0.1/24

echo "[4/4] Setting Top Priority (100) so hotspot autostarts on switch-on..."
sudo nmcli connection modify "$CON_NAME" connection.autoconnect yes
sudo nmcli connection modify "$CON_NAME" connection.autoconnect-priority 100

# Set other Wi-Fi networks to lower autoconnect priority (0)
for con in "IEDC CETLY" "BSNLFTTH-3468" "BigChungus" "netplan-wlan0-moto"; do
    if nmcli connection show "$con" >/dev/null 2>&1; then
        sudo nmcli connection modify "$con" connection.autoconnect-priority 0 2>/dev/null || true
    fi
done

echo ""
echo "=================================================="
echo " SUCCESS: '$CON_NAME' configured to autostart on boot!"
echo ""
echo " Whenever the robot is switched on, it will automatically"
echo " broadcast the '$HOTSPOT_SSID' Wi-Fi hotspot."
echo ""
echo " You can connect your phone to '$HOTSPOT_SSID' and open:"
echo "   http://10.42.0.1:8000"
echo ""
echo " To activate it immediately right now without rebooting:"
echo "   sudo nmcli connection up $CON_NAME"
echo "=================================================="
