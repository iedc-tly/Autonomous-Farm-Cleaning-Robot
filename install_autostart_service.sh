#!/usr/bin/env bash
# ==============================================================================
# install_autostart_service.sh
# 
# Installs and enables the Autonomous Farm Cleaner systemd service
# so all robot nodes (App Bridge, Rosbridge, Nav2, Hardware) start on boot.
# ==============================================================================

set -e

SERVICE_SRC="/home/iedctly/ros2_ws/farm_robot.service"
SERVICE_DST="/etc/systemd/system/farm_robot.service"

echo "=================================================="
echo " INSTALLING AUTONOMOUS FARM CLEANER SYSTEMD SERVICE "
echo "=================================================="

# 1. Copy unit file
echo "[1/4] Copying $SERVICE_SRC -> $SERVICE_DST..."
sudo cp "$SERVICE_SRC" "$SERVICE_DST"

# 2. Daemon reload
echo "[2/4] Reloading systemd daemon..."
sudo systemctl daemon-reload

# 3. Enable service on boot
echo "[3/4] Enabling farm_robot.service on system boot..."
sudo systemctl enable farm_robot.service

# 4. Restart service now
echo "[4/4] Restarting farm_robot.service..."
sudo systemctl restart farm_robot.service

echo ""
echo "=================================================="
echo " SUCCESS: Service installed & running on boot!    "
echo " Web App & API available at: http://<robot-ip>:8000"
echo " Rosbridge available at: ws://<robot-ip>:9090     "
echo "=================================================="
echo "Useful commands:"
echo "  • Check status : sudo systemctl status farm_robot.service"
echo "  • View logs    : journalctl -u farm_robot.service -f"
echo "  • Restart      : sudo systemctl restart farm_robot.service"
echo "  • Stop         : sudo systemctl stop farm_robot.service"
echo "=================================================="
