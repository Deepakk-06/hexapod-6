#!/usr/bin/env python3
# encoding: utf-8
"""
Minimal register-level driver for the InvenSense MPU6050 6-axis IMU
(publicly documented register map, independent of any robot vendor).
"""
import smbus2
import time

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SCALE_2G = 16384.0   # LSB/g
GYRO_SCALE_250DPS = 131.0  # LSB/(deg/s)


class MPU6050:
    def __init__(self, bus_num=1, address=0x68):
        self.address = address
        self.bus = smbus2.SMBus(bus_num)
        # wake up (default is sleep mode at power-on)
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        self.bus.write_byte_data(self.address, SMPLRT_DIV, 0x07)   # 1kHz / (1+7) = 125Hz
        self.bus.write_byte_data(self.address, CONFIG, 0x03)       # DLPF ~44Hz
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)  # +-250 deg/s
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)  # +-2g

    def _read_word_signed(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            val -= 0x10000
        return val

    def read_accel(self):
        """Returns (ax, ay, az) in g."""
        ax = self._read_word_signed(ACCEL_XOUT_H) / ACCEL_SCALE_2G
        ay = self._read_word_signed(ACCEL_XOUT_H + 2) / ACCEL_SCALE_2G
        az = self._read_word_signed(ACCEL_XOUT_H + 4) / ACCEL_SCALE_2G
        return ax, ay, az

    def read_gyro(self):
        """Returns (gx, gy, gz) in rad/s."""
        import math
        gx = self._read_word_signed(GYRO_XOUT_H) / GYRO_SCALE_250DPS
        gy = self._read_word_signed(GYRO_XOUT_H + 2) / GYRO_SCALE_250DPS
        gz = self._read_word_signed(GYRO_XOUT_H + 4) / GYRO_SCALE_250DPS
        return math.radians(gx), math.radians(gy), math.radians(gz)
