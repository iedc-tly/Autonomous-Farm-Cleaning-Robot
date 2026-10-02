#!/usr/bin/env python3
"""
test_mpu9150.py

Standalone command-line diagnostic tool to verify MPU-9150 wiring and data stream.
Run via:
  ros2 run motor_control test_mpu9150
or directly:
  python3 src/motor_control/motor_control/test_mpu9150.py
"""

import sys
import time
import math
import smbus2

def scan_i2c(bus_id=1):
    print(f"Scanning I2C bus {bus_id}...")
    found = []
    try:
        bus = smbus2.SMBus(bus_id)
        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                found.append(addr)
            except Exception:
                pass
        bus.close()
    except Exception as e:
        print(f"Error accessing I2C bus {bus_id}: {e}")
        return []
    return found

def main():
    print("=" * 60)
    print("       MPU-9150 / MPU-6050 HARDWARE CONNECTION DIAGNOSTIC     ")
    print("=" * 60)

    devices = scan_i2c(1)
    if not devices:
        print("\n[!] No I2C devices detected on /dev/i2c-1.")
        print("    Please check physical wiring:")
        print("      - VCC -> 3.3V (Raspberry Pi Pin 1)")
        print("      - GND -> GND  (Raspberry Pi Pin 6)")
        print("      - SDA -> GPIO 2 (Raspberry Pi Pin 3)")
        print("      - SCL -> GPIO 3 (Raspberry Pi Pin 5)")
        print("      - AD0 -> GND  (Raspberry Pi Pin 9 or shared GND)")
        sys.exit(1)

    print(f"\n[+] Found {len(devices)} device(s) on I2C bus 1: {[hex(a) for a in devices]}")

    target_addr = None
    if 0x68 in devices:
        target_addr = 0x68
    elif 0x69 in devices:
        target_addr = 0x69

    if not target_addr:
        print("[!] Neither 0x68 nor 0x69 was detected. MPU-9150 is not responding.")
        sys.exit(1)

    print(f"[+] MPU-9150 detected at address: 0x{target_addr:02X}!")

    from motor_control.mpu9150_driver import MPU9150
    driver = MPU9150(bus_id=1, address=target_addr)
    if not driver.connect():
        print("[!] Failed to initialize MPU-9150 registers.")
        sys.exit(1)

    print(f"[+] Initialized MPU-9150 successfully. Magnetometer AK8975 available: {driver.mag_available}")
    print("[*] Calibrating gyro offsets for 2 seconds (KEEP SENSOR STILL)...")
    bx, by, bz = driver.calibrate_gyro(samples=100)
    print(f"[+] Done! Gyro biases: X={bx:.4f}, Y={by:.4f}, Z={bz:.4f} rad/s")

    print("\nStarting live sensor monitor (Press Ctrl+C to exit):")
    print(f"{'Accel X (m/s²)':>14} | {'Accel Y':>9} | {'Accel Z':>9} | {'Gyro Z (°/s)':>13} | {'Temp (°C)':>9}")
    print("-" * 65)

    try:
        while True:
            m = driver.read_motion()
            gz_degs = math.degrees(m['gz'])
            print(
                f"{m['ax']:14.2f} | {m['ay']:9.2f} | {m['az']:9.2f} | {gz_degs:13.2f} | {m['temp_c']:9.1f}",
                end="\r"
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting diagnostic.")
    finally:
        driver.close()

if __name__ == "__main__":
    main()
