#!/usr/bin/env python3
"""
encoder_odom_node.py

Reads 4x quadrature Hall-effect encoders (LF, RF, LR, RR) on a skid-steer
4-wheel robot, computes wheel odometry, and publishes nav_msgs/Odometry on
/odom plus the odom -> base_link TF transform for Nav2 / SLAM.
"""

import math
import threading

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

# ---------------------------------------------------------------------------
# Robot / encoder constants
# ---------------------------------------------------------------------------
WHEEL_DIAMETER_M = 0.1013184
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M

TRACK_WIDTH_M = 1.0685756   # Effective skid-steer track width (calibrated: 0.8014317 / 0.75 turns)
WHEELBASE_M = 0.46     # Front-back distance

GEAR_RATIO = 19.2
PPR = 7
DECODING_MULTIPLIER = 1
TICKS_PER_WHEEL_REV = PPR * GEAR_RATIO * DECODING_MULTIPLIER  # 134.4 ticks/rev

METERS_PER_TICK = WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV

ENCODER_PINS = {
    "RF": (4, 27),
    "LF": (25, 22),
    "RR": (26, 20),
    "LR": (16, 21),
}


class EncoderCounter:
    """
    1x Quadrature Decoding:
    Only triggers on RISING edge of Channel A and samples Channel B level.
    Reduces hardware interrupt callbacks by 75% compared to 4x decoding,
    preventing dropped ticks on Raspberry Pi under high motor RPM and ROS CPU load.
    """

    def __init__(self, ha_pin: int, hb_pin: int):
        self.ha_pin = ha_pin
        self.hb_pin = hb_pin
        self.ticks = 0
        self._lock = threading.Lock()

        GPIO.setup(self.ha_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.hb_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # 1x Decoding: trigger only on RISING edge of Channel A
        GPIO.add_event_detect(self.ha_pin, GPIO.RISING, callback=self._on_rising_a)

    def _on_rising_a(self, _channel):
        # Quadrature direction: check Channel B state when Channel A rises
        b = GPIO.input(self.hb_pin)
        with self._lock:
            if b:
                self.ticks += 1
            else:
                self.ticks -= 1

    def get_ticks(self) -> int:
        with self._lock:
            return self.ticks



class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__("encoder_odom_node")

        # Inversion parameters for wheel direction calibration
        self.declare_parameter("invert_all", False)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("invert_lf", False)
        self.declare_parameter("invert_rf", False)
        self.declare_parameter("invert_lr", False)
        self.declare_parameter("invert_rr", False)

        # Odometry calibration parameters
        self.declare_parameter("wheel_diameter", WHEEL_DIAMETER_M)
        self.declare_parameter("track_width", TRACK_WIDTH_M)
        self.declare_parameter("gear_ratio", GEAR_RATIO)
        self.declare_parameter("ppr", float(PPR))
        self.declare_parameter("decoding_multiplier", float(DECODING_MULTIPLIER))
        self.declare_parameter("invert_angular", False)

        # IMU fusion parameters
        self.declare_parameter("use_imu", False)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("publish_tf", True)

        GPIO.setmode(GPIO.BCM)

        self.encoders = {
            name: EncoderCounter(ha, hb)
            for name, (ha, hb) in ENCODER_PINS.items()
        }

        self._last_ticks = {name: 0 for name in self.encoders}
        self._last_time = self.get_clock().now()

        # IMU state for gyro heading fusion
        self.latest_imu_wz = 0.0
        self.last_imu_time = None
        self.imu_fused = False
        imu_topic = self.get_parameter("imu_topic").value
        self.imu_sub = self.create_subscription(Imu, imu_topic, self._imu_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        publish_rate_hz = 20.0
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.update)

        self.get_logger().info(
            f"Encoder odom node initialized with parameters: wheel_diameter={WHEEL_DIAMETER_M}, track_width={TRACK_WIDTH_M}, decoding_multiplier={DECODING_MULTIPLIER}, use_imu={self.get_parameter('use_imu').value}"
        )

    def _imu_cb(self, msg: Imu):
        self.latest_imu_wz = msg.angular_velocity.z
        self.last_imu_time = self.get_clock().now()
        if not self.imu_fused:
            self.imu_fused = True
            self.get_logger().info(
                "IMU stream connected! Fusing gyro yaw rate with wheel odometry."
            )

    def _get_wheel_multiplier(self, name: str) -> float:
        inv = self.get_parameter("invert_all").value
        if name in ("LF", "LR") and self.get_parameter("invert_left").value:
            inv = not inv
        elif name in ("RF", "RR") and self.get_parameter("invert_right").value:
            inv = not inv
        if self.get_parameter(f"invert_{name.lower()}").value:
            inv = not inv
        return -1.0 if inv else 1.0

    def update(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self._last_time = now

        wheel_diameter = float(self.get_parameter("wheel_diameter").value)
        track_width = float(self.get_parameter("track_width").value)
        gear_ratio = float(self.get_parameter("gear_ratio").value)
        ppr = float(self.get_parameter("ppr").value)
        decoding_multiplier = float(self.get_parameter("decoding_multiplier").value)

        ticks_per_wheel_rev = ppr * gear_ratio * decoding_multiplier
        wheel_circumference = math.pi * wheel_diameter
        meters_per_tick = wheel_circumference / ticks_per_wheel_rev

        deltas_m = {}
        delta_ticks_all = {}
        for name, enc in self.encoders.items():
            ticks_now = enc.get_ticks()
            delta_ticks = ticks_now - self._last_ticks[name]
            self._last_ticks[name] = ticks_now
            mult = self._get_wheel_multiplier(name)
            deltas_m[name] = delta_ticks * meters_per_tick * mult
            delta_ticks_all[name] = delta_ticks

        # Average left and right side displacement
        left_m = (deltas_m["LF"] + deltas_m["LR"]) / 2.0
        right_m = (deltas_m["RF"] + deltas_m["RR"]) / 2.0

        d_center = (left_m + right_m) / 2.0

        # IMU heading rate fusion vs differential wheel encoder heading
        use_imu = self.get_parameter("use_imu").value
        imu_active = False
        if use_imu and self.last_imu_time is not None:
            age_s = (now - self.last_imu_time).nanoseconds / 1e9
            if age_s < 0.5:
                imu_active = True

        if imu_active:
            # Gyroscope directly measures body yaw rate without skid-steer scrubbing slippage
            wz = self.latest_imu_wz
            if self.get_parameter("invert_angular").value:
                wz = -wz
            d_theta = wz * dt
            v_angular = wz
        else:
            d_theta = (left_m - right_m) / track_width
            if self.get_parameter("invert_angular").value:
                d_theta = -d_theta
            v_angular = d_theta / dt

        # 2D Pose Integration using mid-step heading for accurate turning
        heading_mid = self.theta + (d_theta / 2.0)
        self.x += d_center * math.cos(heading_mid)
        self.y += d_center * math.sin(heading_mid)
        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        v_linear = d_center / dt

        self._publish_odom(now, v_linear, v_angular, imu_active)

    def _publish_odom(self, stamp: Time, v_linear: float, v_angular: float, imu_active: bool = False):
        q = self._yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular

        # Standard Covariance matrix configuration for 2D SLAM
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001 if imu_active else 0.01
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.001 if imu_active else 0.01

        self.odom_pub.publish(odom)

        if self.get_parameter("publish_tf").value:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
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
