#!/usr/bin/env python3
"""
mpu9150_driver.py

Low-level I2C hardware driver for the InvenSense MPU-9150 9-DOF IMU.
MPU-9150 incorporates an MPU-6050 (3-axis gyro + 3-axis accel) and
an Asahi Kasei AK8975 3-axis magnetometer connected on the internal aux bus.
"""

import time
import math
from typing import Tuple, Optional, Dict
import smbus2

# ---- Register Definitions -------------------------------------------------
# MPU-6050 Core Registers
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_INT_PIN_CFG  = 0x37
REG_INT_ENABLE   = 0x38
REG_ACCEL_XOUT_H = 0x3B
REG_TEMP_OUT_H   = 0x41
REG_GYRO_XOUT_H  = 0x43
REG_USER_CTRL    = 0x6A
REG_PWR_MGMT_1   = 0x6B
REG_PWR_MGMT_2   = 0x6C
REG_WHO_AM_I     = 0x75

# AK8975 Magnetometer Registers (Aux / Bypass Address 0x0C)
AK8975_ADDR      = 0x0C
AK8975_WIA       = 0x00  # Device ID register (should return 0x48)
AK8975_ST1       = 0x02  # Status 1 (DRDY: Data Ready bit 0)
AK8975_HXL       = 0x03  # Data registers (6 bytes: X, Y, Z, little-endian)
AK8975_CNTL      = 0x0A  # Mode control register

# Conversions
GRAVITY_MSS = 9.80665
DEG2RAD = math.pi / 180.0


def _to_int16(high: int, low: int) -> int:
    val = (high << 8) | low
    return val - 65536 if val >= 32768 else val


def _to_int16_le(low: int, high: int) -> int:
    val = (high << 8) | low
    return val - 65536 if val >= 32768 else val


class MPU9150:
    """I2C driver for MPU-9150 IMU."""

    def __init__(
        self,
        bus_id: int = 1,
        address: int = 0x68,
        accel_scale_g: int = 2,
        gyro_scale_dps: int = 500
    ):
        self.bus_id = bus_id
        self.address = address
        self.accel_scale_g = accel_scale_g
        self.gyro_scale_dps = gyro_scale_dps

        self.bus: Optional[smbus2.SMBus] = None
        self.mag_available = False

        # Gyro zero-rate bias offsets (calibrated at startup)
        self.bias_gx = 0.0
        self.bias_gy = 0.0
        self.bias_gz = 0.0

        # Sensitivity scale factors
        self._accel_lsb = 16384.0 if accel_scale_g == 2 else 8192.0
        self._gyro_lsb = 65.5 if gyro_scale_dps == 500 else 131.0

    def connect(self) -> bool:
        """Connect to I2C bus and verify sensor presence."""
        try:
            self.bus = smbus2.SMBus(self.bus_id)
            who_am_i = self.bus.read_byte_data(self.address, REG_WHO_AM_I)
            if who_am_i not in (0x68, 0x69, 0x70, 0x71, 0x73):
                # Some revisions report slightly different whoami or 0x68
                return False
            self._init_sensor()
            return True
        except Exception:
            return False

    def _init_sensor(self):
        """Configure MPU-9150 internal registers."""
        if not self.bus:
            return

        # 1. Reset device
        self.bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        # 2. Wake up & select PLL with X-axis gyroscope clock reference
        self.bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x01)
        time.sleep(0.01)

        # 3. Enable all accel and gyro axes
        self.bus.write_byte_data(self.address, REG_PWR_MGMT_2, 0x00)

        # 4. DLPF Config (Digital Low-Pass Filter):
        # 0x03 = ~42 Hz filter bandwidth (filters chassis & motor vibrations)
        self.bus.write_byte_data(self.address, REG_CONFIG, 0x03)

        # 5. Sample rate divider: 1000Hz / (1 + 9) = 100Hz internal sampling
        self.bus.write_byte_data(self.address, REG_SMPLRT_DIV, 0x09)

        # 6. Gyroscope Full-Scale Range
        # 0x00 = ±250 dps, 0x08 = ±500 dps, 0x10 = ±1000 dps, 0x18 = ±2000 dps
        gyro_cfg = 0x08 if self.gyro_scale_dps == 500 else 0x00
        self.bus.write_byte_data(self.address, REG_GYRO_CONFIG, gyro_cfg)

        # 7. Accelerometer Full-Scale Range
        # 0x00 = ±2g, 0x08 = ±4g
        accel_cfg = 0x00 if self.accel_scale_g == 2 else 0x08
        self.bus.write_byte_data(self.address, REG_ACCEL_CONFIG, accel_cfg)

        # 8. Enable I2C Bypass Mode to expose AK8975 directly on primary I2C bus
        # Disable I2C master mode on USER_CTRL first
        self.bus.write_byte_data(self.address, REG_USER_CTRL, 0x00)
        # Set BYPASS_EN = 1 in INT_PIN_CFG (0x02)
        self.bus.write_byte_data(self.address, REG_INT_PIN_CFG, 0x02)
        time.sleep(0.01)

        # Check AK8975 Magnetometer
        self._init_magnetometer()

    def _init_magnetometer(self):
        """Verify and initialize AK8975 magnetometer."""
        try:
            wia = self.bus.read_byte_data(AK8975_ADDR, AK8975_WIA)
            if wia == 0x48:
                self.mag_available = True
                # Request initial single measurement
                self.bus.write_byte_data(AK8975_ADDR, AK8975_CNTL, 0x01)
        except Exception:
            self.mag_available = False

    def calibrate_gyro(self, samples: int = 200, delay_s: float = 0.01) -> Tuple[float, float, float]:
        """
        Calibrate gyro zero-rate offsets while the robot is stationary.
        Returns (bias_gx, bias_gy, bias_gz) in rad/s.
        """
        sum_gx, sum_gy, sum_gz = 0.0, 0.0, 0.0
        valid = 0

        for _ in range(samples):
            try:
                data = self.bus.read_i2c_block_data(self.address, REG_GYRO_XOUT_H, 6)
                raw_gx = _to_int16(data[0], data[1])
                raw_gy = _to_int16(data[2], data[3])
                raw_gz = _to_int16(data[4], data[5])

                gx = (raw_gx / self._gyro_lsb) * DEG2RAD
                gy = (raw_gy / self._gyro_lsb) * DEG2RAD
                gz = (raw_gz / self._gyro_lsb) * DEG2RAD

                sum_gx += gx
                sum_gy += gy
                sum_gz += gz
                valid += 1
            except Exception:
                pass
            time.sleep(delay_s)

        if valid > 0:
            self.bias_gx = sum_gx / valid
            self.bias_gy = sum_gy / valid
            self.bias_gz = sum_gz / valid

        return self.bias_gx, self.bias_gy, self.bias_gz

    def read_motion(self) -> Dict[str, float]:
        """
        Burst read 14 bytes (accel, temp, gyro) from MPU-9150.
        Returns dictionary with:
          ax, ay, az (m/s^2)
          gx, gy, gz (rad/s, bias-corrected)
          temp_c (degrees Celsius)
        """
        if not self.bus:
            raise RuntimeError("MPU-9150 bus not connected")

        # Burst read 14 bytes starting at REG_ACCEL_XOUT_H
        data = self.bus.read_i2c_block_data(self.address, REG_ACCEL_XOUT_H, 14)

        raw_ax = _to_int16(data[0], data[1])
        raw_ay = _to_int16(data[2], data[3])
        raw_az = _to_int16(data[4], data[5])
        raw_temp = _to_int16(data[6], data[7])
        raw_gx = _to_int16(data[8], data[9])
        raw_gy = _to_int16(data[10], data[11])
        raw_gz = _to_int16(data[12], data[13])

        # Convert accel to m/s^2
        ax = (raw_ax / self._accel_lsb) * GRAVITY_MSS
        ay = (raw_ay / self._accel_lsb) * GRAVITY_MSS
        az = (raw_az / self._accel_lsb) * GRAVITY_MSS

        # Convert temperature to Celsius (formula from MPU-6000 register map)
        temp_c = (raw_temp / 340.0) + 36.53

        # Convert gyro to rad/s and remove calibrated zero-rate bias
        gx = ((raw_gx / self._gyro_lsb) * DEG2RAD) - self.bias_gx
        gy = ((raw_gy / self._gyro_lsb) * DEG2RAD) - self.bias_gy
        gz = ((raw_gz / self._gyro_lsb) * DEG2RAD) - self.bias_gz

        return {
            "ax": ax,
            "ay": ay,
            "az": az,
            "gx": gx,
            "gy": gy,
            "gz": gz,
            "temp_c": temp_c,
        }

    def read_magnetometer(self) -> Optional[Tuple[float, float, float]]:
        """
        Read AK8975 3-axis magnetic flux density in Tesla.
        Returns (mx, my, mz) in Tesla, or None if data not ready / unavailable.
        """
        if not self.mag_available or not self.bus:
            return None

        try:
            st1 = self.bus.read_byte_data(AK8975_ADDR, AK8975_ST1)
            if not (st1 & 0x01):  # DRDY check
                return None

            # 6 data bytes + 1 status byte (ST2)
            data = self.bus.read_i2c_block_data(AK8975_ADDR, AK8975_HXL, 7)
            raw_mx = _to_int16_le(data[0], data[1])
            raw_my = _to_int16_le(data[2], data[3])
            raw_mz = _to_int16_le(data[4], data[5])

            # Trigger next single measurement
            self.bus.write_byte_data(AK8975_ADDR, AK8975_CNTL, 0x01)

            # AK8975 sensitivity: 0.3 uT / LSB. Convert to Tesla (1 uT = 1e-6 T)
            uT_to_T = 0.3e-6
            return (raw_mx * uT_to_T, raw_my * uT_to_T, raw_mz * uT_to_T)
        except Exception:
            return None

    def close(self):
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None
