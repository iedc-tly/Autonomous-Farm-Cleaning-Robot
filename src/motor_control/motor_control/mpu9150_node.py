#!/usr/bin/env python3
"""
mpu9150_node.py

ROS 2 Humble Node for the InvenSense MPU-9150 9-DOF IMU.
Publishes:
  - /imu/data       (sensor_msgs/msg/Imu) with orientation, angular velocity, and linear acceleration
  - /imu/yaw_deg    (std_msgs/msg/Float32) for quick debugging and telemetry
  - /imu/mag        (sensor_msgs/msg/MagneticField) when magnetometer is available
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

from motor_control.mpu9150_driver import MPU9150


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MPU9150Node(Node):
    def __init__(self):
        super().__init__("mpu9150_node")

        # Declare parameters
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("device_address", 0x68)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("calibrate_samples", 200)
        self.declare_parameter("filter_alpha", 0.98)
        self.declare_parameter("invert_gyro_z", False)
        self.declare_parameter("accel_scale_g", 2)
        self.declare_parameter("gyro_scale_dps", 500)

        self.i2c_bus = self.get_parameter("i2c_bus").value
        self.device_addr = self.get_parameter("device_address").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.filter_alpha = float(self.get_parameter("filter_alpha").value)
        self.invert_gz = bool(self.get_parameter("invert_gyro_z").value)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.yaw_pub = self.create_publisher(Float32, "/imu/yaw_deg", 10)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 10)

        # Orientation state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.driver: Optional[MPU9150] = None
        self.last_update_time = None
        self.is_connected = False
        self.consecutive_errors = 0

        self.get_logger().info(
            f"Initializing MPU-9150 on I2C bus {self.i2c_bus}, address 0x{self.device_addr:02X}..."
        )

        self._init_driver()

        # Update timer
        timer_period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.02
        self.timer = self.create_timer(timer_period, self.update_callback)

    def _init_driver(self):
        try:
            self.driver = MPU9150(
                bus_id=self.i2c_bus,
                address=self.device_addr,
                accel_scale_g=self.get_parameter("accel_scale_g").value,
                gyro_scale_dps=self.get_parameter("gyro_scale_dps").value
            )

            if not self.driver.connect():
                # Attempt alternative address 0x69 if 0x68 failed
                alt_addr = 0x69 if self.device_addr == 0x68 else 0x68
                self.driver.address = alt_addr
                if self.driver.connect():
                    self.device_addr = alt_addr
                    self.get_logger().info(f"Connected to MPU-9150 on alternative address 0x{alt_addr:02X}")
                else:
                    self.get_logger().warn(
                        f"MPU-9150 not detected on I2C bus {self.i2c_bus} (checked 0x68, 0x69). "
                        "Will retry in background when sensor is connected."
                    )
                    self.is_connected = False
                    return

            self.is_connected = True
            calib_samples = int(self.get_parameter("calibrate_samples").value)
            if calib_samples > 0:
                self.get_logger().info(
                    f"Calibrating gyro biases ({calib_samples} samples)... PLEASE KEEP ROBOT STILL."
                )
                bx, by, bz = self.driver.calibrate_gyro(samples=calib_samples)
                self.get_logger().info(
                    f"Calibration complete! Gyro biases: X={bx:.4f}, Y={by:.4f}, Z={bz:.4f} rad/s"
                )

            # Seed initial roll and pitch from accelerometer
            try:
                m = self.driver.read_motion()
                self.roll = math.atan2(m["ay"], m["az"])
                self.pitch = math.atan2(-m["ax"], math.sqrt(m["ay"]**2 + m["az"]**2))
                self.yaw = 0.0
                self.get_logger().info(
                    f"Initial attitude seeded: roll={math.degrees(self.roll):.1f}°, pitch={math.degrees(self.pitch):.1f}°"
                )
            except Exception:
                pass

        except Exception as e:
            self.get_logger().error(f"Error initializing MPU-9150 driver: {e}")
            self.is_connected = False

    def update_callback(self):
        now = self.get_clock().now()

        if not self.is_connected or self.driver is None:
            # Periodic retry connection every 2 seconds
            if self.consecutive_errors % int(self.publish_rate * 2) == 0:
                self.get_logger().info("Checking for MPU-9150 sensor connection...")
                self._init_driver()
            self.consecutive_errors += 1
            return

        if self.last_update_time is None:
            self.last_update_time = now
            return

        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        if dt <= 0.0 or dt > 0.5:
            # Skip invalid dt jump
            return

        try:
            m = self.driver.read_motion()
            self.consecutive_errors = 0
        except Exception as e:
            self.consecutive_errors += 1
            if self.consecutive_errors % int(self.publish_rate) == 0:
                self.get_logger().warn(f"I2C read glitch/error ({self.consecutive_errors} ticks): {e}")
            if self.consecutive_errors > int(self.publish_rate * 5):
                self.get_logger().error("Lost communication with MPU-9150. Flagging disconnected.")
                self.is_connected = False
            return

        ax, ay, az = m["ax"], m["ay"], m["az"]
        gx, gy, gz = m["gx"], m["gy"], m["gz"]

        if self.invert_gz:
            gz = -gz

        # -------------------------------------------------------------
        # Complementary Filter:
        # Fuses accelerometer gravity vector with gyro integration
        # -------------------------------------------------------------
        # Accelerometer roll and pitch
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Euler angle integration
        alpha = self.filter_alpha
        self.roll = alpha * (self.roll + gx * dt) + (1.0 - alpha) * roll_acc
        self.pitch = alpha * (self.pitch + gy * dt) + (1.0 - alpha) * pitch_acc
        self.yaw += gz * dt

        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Convert to ROS Quaternion
        q_msg = euler_to_quaternion(self.roll, self.pitch, self.yaw)

        # -------------------------------------------------------------
        # Construct and Publish sensor_msgs/Imu
        # -------------------------------------------------------------
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = self.frame_id

        imu_msg.orientation = q_msg
        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)

        # Covariance matrices
        # Orientation: very confident in roll/pitch from gravity, yaw has standard gyro variance
        imu_msg.orientation_covariance = [
            1e-4, 0.0, 0.0,
            0.0, 1e-4, 0.0,
            0.0, 0.0, 1e-3
        ]
        imu_msg.angular_velocity_covariance = [
            1e-4, 0.0, 0.0,
            0.0, 1e-4, 0.0,
            0.0, 0.0, 1e-4
        ]
        imu_msg.linear_acceleration_covariance = [
            1e-2, 0.0, 0.0,
            0.0, 1e-2, 0.0,
            0.0, 0.0, 1e-2
        ]

        self.imu_pub.publish(imu_msg)

        # Publish yaw degrees for quick verification
        yaw_deg_msg = Float32()
        yaw_deg_msg.data = float(math.degrees(self.yaw))
        self.yaw_pub.publish(yaw_deg_msg)

        # Optional Magnetometer read
        if self.driver.mag_available:
            mag_data = self.driver.read_magnetometer()
            if mag_data is not None:
                mag_msg = MagneticField()
                mag_msg.header.stamp = now.to_msg()
                mag_msg.header.frame_id = self.frame_id
                mag_msg.magnetic_field.x = float(mag_data[0])
                mag_msg.magnetic_field.y = float(mag_data[1])
                mag_msg.magnetic_field.z = float(mag_data[2])
                mag_msg.magnetic_field_covariance = [
                    1e-4, 0.0, 0.0,
                    0.0, 1e-4, 0.0,
                    0.0, 0.0, 1e-4
                ]
                self.mag_pub.publish(mag_msg)

    def destroy_node(self):
        if self.driver:
            self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPU9150Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
