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
from tf2_ros import TransformBroadcaster

# ---------------------------------------------------------------------------
# Robot / encoder constants
# ---------------------------------------------------------------------------
WHEEL_DIAMETER_M = 0.1013184
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M

TRACK_WIDTH_M = 0.8014317   # Left-right wheel center distance
WHEELBASE_M = 0.46     # Front-back distance

GEAR_RATIO = 19.2
PPR = 7
DECODING_MULTIPLIER = 4
TICKS_PER_WHEEL_REV = PPR * GEAR_RATIO * DECODING_MULTIPLIER  # 537.6 ticks/rev

METERS_PER_TICK = WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV

ENCODER_PINS = {
    "RF": (4, 27),
    "LF": (25, 22),
    "RR": (26, 20),
    "LR": (16, 21),
}


class EncoderCounter:
    # 4x quadrature decoding state transition lookup table
    # Index = (old_state << 2) | new_state where state = (A << 1) | B
    TRANSITIONS = [
        0,   1, -1,  0,
       -1,   0,  0,  1,
        1,   0,  0, -1,
        0,  -1,  1,  0
    ]

    def __init__(self, ha_pin: int, hb_pin: int):
        self.ha_pin = ha_pin
        self.hb_pin = hb_pin
        self.ticks = 0
        self._lock = threading.Lock()

        GPIO.setup(self.ha_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.hb_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        a = GPIO.input(self.ha_pin)
        b = GPIO.input(self.hb_pin)
        self.last_state = (a << 1) | b

        # 4x Decoding: detect BOTH rising and falling edges on both Channel A and Channel B
        GPIO.add_event_detect(self.ha_pin, GPIO.BOTH, callback=self._on_change)
        GPIO.add_event_detect(self.hb_pin, GPIO.BOTH, callback=self._on_change)

    def _on_change(self, _channel):
        a = GPIO.input(self.ha_pin)
        b = GPIO.input(self.hb_pin)
        new_state = (a << 1) | b
        with self._lock:
            old_state = self.last_state
            self.last_state = new_state
            idx = (old_state << 2) | new_state
            self.ticks += self.TRANSITIONS[idx]

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

        GPIO.setmode(GPIO.BCM)

        self.encoders = {
            name: EncoderCounter(ha, hb)
            for name, (ha, hb) in ENCODER_PINS.items()
        }

        self._last_ticks = {name: 0 for name in self.encoders}
        self._last_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        publish_rate_hz = 20.0
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.update)

        self.get_logger().info(
            f"Encoder odom node initialized with parameters: wheel_diameter={WHEEL_DIAMETER_M}, track_width={TRACK_WIDTH_M}, decoding_multiplier={DECODING_MULTIPLIER}"
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
        d_theta = (left_m - right_m) / track_width
        if self.get_parameter("invert_angular").value:
            d_theta = -d_theta

        # 2D Pose Integration using mid-step heading for accurate turning
        heading_mid = self.theta + (d_theta / 2.0)
        self.x += d_center * math.cos(heading_mid)
        self.y += d_center * math.sin(heading_mid)
        self.theta += d_theta

        v_linear = d_center / dt
        v_angular = d_theta / dt

        self._publish_odom(now, v_linear, v_angular)

    def _publish_odom(self, stamp: Time, v_linear: float, v_angular: float):
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
        odom.pose.covariance[35] = 0.01
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
