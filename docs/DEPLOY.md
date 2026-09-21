# Autonomous Farm Cleaner Deployment Guide

This document describes how to deploy and run the 3-Button Autonomous Operation system on the Raspberry Pi 4 (Ubuntu 22.04 + ROS 2 Humble) without requiring a laptop at runtime.

---

## Assumptions & Requirements

1. **Physical Placement at Power-on**: The robot **must** be physically placed at the saved **Dock** pose `(0, 0, yaw=0)` prior to powering on the Pi.
2. **Network Connection**: The phone/app and Raspberry Pi must be connected to the same Wi-Fi network (or the Raspberry Pi Wi-Fi Hotspot).
3. **HTTP Server Port**: The REST API and status bridge runs on port **8000** (`http://<robot-ip>:8000`).

---

## 1. Fast Build Instructions

To build the ROS 2 workspace packages:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select motor_control rover_app_bridge
source install/setup.bash
```

---

## 2. Setting Locations via CLI Helper

Before using SWEEP or PIT, configure the location coordinates using the `set_location` CLI tool while teleoperating the robot to each target pose:

```bash
# 1. Teleop robot to Dock position, then capture:
ros2 run motor_control set_location dock

# 2. Teleop robot to Pit staging location, then capture:
ros2 run motor_control set_location pit

# 3. Teleop robot to Bottom-Left corner of sweeping rectangle:
ros2 run motor_control set_location sweep_bl

# 4. Teleop robot to Top-Right corner of sweeping rectangle:
ros2 run motor_control set_location sweep_tr
```

Coordinates are saved directly to `~/ros2_ws/src/motor_control/config/locations.yaml`.

---

## 3. Enable Boot Automation (No Laptop Needed)

To automatically launch pigpiod, motor drivers, RPLiDAR, Nav2, initial pose publisher, and the HTTP bridge on power-up:

```bash
# Enable pigpiod system daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Install and enable the farm robot systemd service
sudo cp ~/ros2_ws/farm_robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable farm_robot.service
sudo systemctl start farm_robot.service
```

Check autostart status:

```bash
sudo systemctl status farm_robot.service
```

---

## 4. HTTP API Endpoints (`http://<robot-ip>:8000`)

| Method + Path | Purpose | Success | Error Response |
| :--- | :--- | :--- | :--- |
| `GET /health` | Liveness check | `200 {"ok": true}` | - |
| `GET /locations` | Returns `locations.yaml` | `200 {dock, pit, sweep}` | `412 {"error": "..."}` |
| `GET /status` | System & Nav2 state | `200` JSON | - |
| `POST /sweep` | Start snake sweep | `202 {"accepted": true, "waypoints": N}` | `409 busy`, `412 nav2 not active / bad locations / 0 waypoints` |
| `POST /dock` | Navigate to Dock | `202 {"accepted": true}` | `409 busy`, `412 nav2 not active / bad locations` |
| `POST /pit` | Navigate to Pit | `202 {"accepted": true}` | `409 busy`, `412 nav2 not active / bad locations` |
| `POST /e_stop` | Emergency Stop | `200 {"ok": true}` | Never errors |
