# Autonomous Farm Cleaning Robot — Odometry Calibration Guide

This guide describes the odometry calibration procedure for the 4-wheel skid-steer farm cleaning robot operating without an IMU (`use_imu:=false`).

---

## 1. Kinematics & Skid-Steer Scrubbing Overview

Unlike Ackermann or differential-drive robots with unconstrained casters, a 4-wheel skid-steer robot must slip/scrub its tires sideways during turning. Consequently:
- **Physical track width**: The geometric distance between the left and right wheel centerlines (~0.80 m).
- **Effective track width (`TRACK_WIDTH_M = 1.0685756 m`)**: An empirical value accounting for sideways wheel scrub resistance.
- **Surface dependence**: Wheel scrubbing depends directly on friction, moisture, and surface compliance. The calibrated `TRACK_WIDTH_M = 1.0686` was measured on a dry lab surface. When moving to the actual barn floor (smooth or grooved concrete, slatted flooring, damp/wet manure, or bedding), the effective track width **must be recalibrated**.

---

## 2. In-Place Rotation Calibration Procedure

Run this test whenever deploying on a new floor surface or after tire changes.

### Pre-requisites
1. Bring up the robot stack with `use_imu:=false`:
   ```bash
   ros2 launch motor_control hardware_bringup.launch.py
   ```
2. Verify odometry signs using the diagnostic tool:
   ```bash
   python3 tools/odom_sign_check.py
   ```
   Confirm that a counter-clockwise (CCW) rotation increases yaw toward $+1.57$ rad ($+90^\circ$).

### Execution
1. Mark the starting heading of the robot on the floor with tape or laser reference.
2. Zero the odometry reading or record initial yaw $\theta_0$ from `/odom`.
3. Command the robot to perform $N = 10$ full in-place rotations (e.g., CCW at slow speed $\approx 0.3$ rad/s):
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.3}}"
   ```
4. Stop when the physical robot completes exactly 10 full revolutions ($3600^\circ$ or $20\pi \approx 62.83$ rad).
5. Read the total yaw accumulated on `/odom` ($\theta_{\text{odom}}$).
6. Calculate the correction factor:
   $$\text{scale} = \frac{\theta_{\text{odom}}}{20\pi}$$
7. Adjust `TRACK_WIDTH_M` in `src/motor_control/motor_control/encoder_odom_node.py`:
   $$\text{TRACK\_WIDTH}_{\text{new}} = \text{TRACK\_WIDTH}_{\text{current}} \times \text{scale}$$
   - If the robot turns physically more than the odometry reports, `TRACK_WIDTH` is too large.
   - If the robot turns physically less than the odometry reports, `TRACK_WIDTH` is too small.

---

## 3. UMBmark Procedure (Bi-Directional Square Path)

The University of Michigan Benchmark (UMBmark) decouples systematic wheel diameter discrepancies from effective track width errors.

1. Set up a $4\text{ m} \times 4\text{ m}$ square path on the barn floor.
2. Run the robot through 5 clockwise (CW) loops, recording the endpoint error $(x_{\text{cw}}, y_{\text{cw}})$.
3. Run the robot through 5 counter-clockwise (CCW) loops, recording the endpoint error $(x_{\text{ccw}}, y_{\text{ccw}})$.
4. Compute center of gravity of errors for CW and CCW runs:
   $$\alpha = \frac{(x_{\text{cw}} + x_{\text{ccw}})}{-4L}, \quad \beta = \frac{(x_{\text{cw}} - x_{\text{ccw}})}{-4L}$$
5. Adjust left/right wheel diameter ratios and effective track width until average return error is minimized ($< 0.1\text{ m}$).
