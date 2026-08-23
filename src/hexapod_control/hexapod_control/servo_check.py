#!/usr/bin/env python3
# encoding: utf-8
"""
Standalone (non-ROS) sanity check: pings every servo ID 1-18 directly over
the serial bus and reports which respond. Run this FIRST, before anything
else, to confirm wiring and IDs are correct.

Usage:
    python3 servo_check.py /dev/ttyUSB0
"""
import sys
from sts3215_driver.scservo_protocol import SCServoBus


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    bus = SCServoBus(port=port, baudrate=1000000)

    print(f'Pinging servo IDs 1-18 on {port} ...')
    ok = []
    missing = []
    for sid in range(1, 19):
        alive = bus.ping(sid)
        print(f'  id {sid:2d}: {"OK" if alive else "no response"}')
        (ok if alive else missing).append(sid)

    print()
    print(f'{len(ok)}/18 servos responded: {ok}')
    if missing:
        print(f'MISSING: {missing}  <- check power, wiring, and ID programming for these')
    bus.close()


if __name__ == '__main__':
    main()
