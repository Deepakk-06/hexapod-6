#!/usr/bin/env python3
# encoding: utf-8
"""
Minimal pure-Python implementation of the Feetech SCServo / STS serial-bus
servo protocol used by the STS3215 (and SCS/SMS family) servos.

This talks directly over a UART/USB-serial link (e.g. through the SmartElex
serial bus servo driver board, which is a plain passthrough adapter with no
protocol translation of its own).

Protocol reference (publicly documented by Feetech, independent of any
particular robot vendor):

    Byte:      0     1     2      3        4          5..N-2      N-1
    Field:   0xFF  0xFF   ID   Length  Instruction   Params...   Checksum

    Checksum = ~(ID + Length + Instruction + sum(Params)) & 0xFF

Register map (STS/SMS control table, addresses in decimal):
    EEPROM
        5   ID
        6   Baud rate
    RAM
        40  Torque Enable        (1 byte)
        41  Acceleration         (1 byte)
        42  Goal Position        (2 bytes, little endian, signed magnitude not used - 0..4095)
        44  Goal Time            (2 bytes, ms)
        46  Goal Speed           (2 bytes)
        56  Present Position     (2 bytes)
        58  Present Speed        (2 bytes)
        60  Present Load         (2 bytes)
        62  Present Voltage      (1 byte, 0.1V units)
        63  Present Temperature  (1 byte, deg C)
        65  Moving flag          (1 byte)
"""

import serial
import struct
import threading

# --- Instructions ---
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_ACTION = 0x05
INST_SYNC_WRITE = 0x83
INST_SYNC_READ = 0x82

# --- Control table addresses (RAM) ---
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_TIME = 44
ADDR_GOAL_SPEED = 46
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66

BROADCAST_ID = 0xFE

POSITION_TICKS = 4096  # 0..4095, 12-bit magnetic encoder
POSITION_CENTER = 2048


class SCServoError(Exception):
    pass


def _checksum(data_bytes):
    return (~sum(data_bytes)) & 0xFF


def _build_packet(servo_id, instruction, params=b''):
    length = len(params) + 2
    body = bytes([servo_id, length, instruction]) + bytes(params)
    chk = _checksum(body)
    return b'\xff\xff' + body + bytes([chk])


class SCServoBus:
    """
    Thread-safe half-duplex driver for a chain of STS3215 servos on a single
    UART. Only one request is allowed to be in flight at a time since the
    bus is shared and half-duplex.
    """

    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, timeout=0.05):
        self._lock = threading.Lock()
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def close(self):
        with self._lock:
            if self.ser.is_open:
                self.ser.close()

    # ---------- low level ----------

    def _send(self, packet):
        self.ser.reset_input_buffer()
        self.ser.write(packet)

    def _read_status(self, expected_params=0):
        """Read a status packet: FF FF ID LEN ERR PARAMS... CHK"""
        header = self.ser.read(4)
        if len(header) < 4 or header[0] != 0xFF or header[1] != 0xFF:
            return None
        servo_id, length = header[2], header[3]
        rest = self.ser.read(length)
        if len(rest) < length:
            return None
        err = rest[0]
        params = rest[1:length - 1]
        return {'id': servo_id, 'error': err, 'params': params}

    # ---------- basic instructions ----------

    def ping(self, servo_id):
        with self._lock:
            self._send(_build_packet(servo_id, INST_PING))
            reply = self._read_status()
        return reply is not None and reply['error'] == 0

    def write(self, servo_id, address, data_bytes):
        """WRITE instruction: write data_bytes starting at control-table address."""
        params = bytes([address]) + bytes(data_bytes)
        with self._lock:
            self._send(_build_packet(servo_id, INST_WRITE, params))
            if servo_id != BROADCAST_ID:
                self._read_status()

    def read(self, servo_id, address, length):
        params = bytes([address, length])
        with self._lock:
            self._send(_build_packet(servo_id, INST_READ, params))
            reply = self._read_status()
        if reply is None or reply['error'] != 0:
            return None
        return reply['params']

    def sync_write(self, address, length, id_data_map):
        """
        SYNC_WRITE instruction: write the same-length block of data to many
        servos in one bus transaction (no status reply is sent for sync
        write, so this is the fast path for driving all 18 servos at once).

        id_data_map: {servo_id: bytes(length)}
        """
        params = bytearray([address, length])
        for sid, data in id_data_map.items():
            data = bytes(data)
            if len(data) != length:
                raise ValueError('data length mismatch for sync_write')
            params += bytes([sid]) + data
        with self._lock:
            self._send(_build_packet(BROADCAST_ID, INST_SYNC_WRITE, bytes(params)))
            # no status packet for sync write / broadcast

    # ---------- convenience wrappers ----------

    def enable_torque(self, servo_id, on=True):
        self.write(servo_id, ADDR_TORQUE_ENABLE, [1 if on else 0])

    def set_position(self, servo_id, position_ticks, time_ms=0, speed=0):
        position_ticks = max(0, min(POSITION_TICKS - 1, int(position_ticks)))
        data = struct.pack('<HHH', position_ticks, int(time_ms), int(speed))
        self.write(servo_id, ADDR_GOAL_POSITION, data)

    def sync_set_positions(self, id_pos_time_speed):
        """
        id_pos_time_speed: iterable of (servo_id, position_ticks, time_ms, speed)
        Writes goal position + goal time + goal speed to all servos in a
        single synchronized bus transaction so all legs start moving
        together.
        """
        id_data = {}
        for sid, pos, t_ms, speed in id_pos_time_speed:
            pos = max(0, min(POSITION_TICKS - 1, int(pos)))
            id_data[sid] = struct.pack('<HHH', pos, int(t_ms), int(speed))
        self.sync_write(ADDR_GOAL_POSITION, 6, id_data)

    def read_position(self, servo_id):
        raw = self.read(servo_id, ADDR_PRESENT_POSITION, 2)
        if raw is None:
            return None
        return struct.unpack('<H', raw)[0]

    def read_voltage_temp(self, servo_id):
        raw = self.read(servo_id, ADDR_PRESENT_VOLTAGE, 2)
        if raw is None:
            return None
        voltage_01v, temp_c = raw[0], raw[1]
        return voltage_01v / 10.0, temp_c
