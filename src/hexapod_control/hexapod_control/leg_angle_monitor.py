#!/usr/bin/env python3
# encoding: utf-8
"""
leg_angle_monitor

Live view of every joint's ACTUAL angle (converted from the real tick
position reported back by the servos via joint_raw_states -- not the
commanded angle, the measured one), for spotting calibration drift or
mechanical issues while the robot is actually moving.

Two modes:

1. Table mode (default) -- live-updating terminal table of all 18
   joint angles at once:

    ros2 run hexapod_control leg_angle_monitor --ros-args \\
        -p config_path:=install/hexapod_control/share/hexapod_control/config/servo_config.yaml

2. Plot mode -- live scrolling graph comparing specific joints over
   time, e.g. to directly overlay LM vs RM tibia while walking and see
   exactly how/when they diverge:

    ros2 run hexapod_control leg_angle_monitor --ros-args \\
        -p config_path:=install/hexapod_control/share/hexapod_control/config/servo_config.yaml \\
        -p mode:=plot -p compare:="LM:tibia,RM:tibia"

    (compare list is "LEG:JOINT,LEG:JOINT,..." -- plot any number of
    series, not just two, e.g. all 6 legs' tibia at once to compare
    the whole set rather than just one pair)

Both modes read joint_raw_states (published by sts3215_driver's
driver_node) and convert ticks back to degrees using the SAME
calibration file the gait engine uses, so what you see here is
directly comparable to what you'd expect the leg to be doing.
"""
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from hexapod_kinematics import geometry as geo
from hexapod_control.calibration import load_calibration


class LegAngleMonitorNode(Node):
    def __init__(self):
        super().__init__('leg_angle_monitor')

        self.declare_parameter('config_path', '')
        self.declare_parameter('mode', 'table')  # 'table' or 'plot'
        self.declare_parameter('compare', 'LM:tibia,RM:tibia')
        self.declare_parameter('window_s', 15.0)

        config_path = self.get_parameter('config_path').value
        if not config_path:
            self.get_logger().error('config_path parameter is required')
            raise SystemExit(1)
        self.calib = load_calibration(config_path)

        # reverse map: servo_id -> (leg, joint, JointCalibration)
        self.id_to_joint = {}
        for leg in geo.LEG_NAMES:
            for joint in ('coxa', 'femur', 'tibia'):
                sid, jcalib = self.calib[leg][joint]
                self.id_to_joint[sid] = (leg, joint, jcalib)

        self.mode = self.get_parameter('mode').value
        self.window_s = float(self.get_parameter('window_s').value)

        # latest angle (degrees) per (leg, joint), plus a start-time-relative
        # history buffer per series for plot mode
        self.latest_deg = {}
        self.history = {}  # (leg,joint) -> deque of (t, deg)
        self.start_time = time.time()
        self._lock = threading.Lock()

        self.sub = self.create_subscription(JointState, 'joint_raw_states', self.on_state, 10)
        self.get_logger().info('leg_angle_monitor ready, waiting for joint_raw_states ...')

    def on_state(self, msg: JointState):
        now = time.time() - self.start_time
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                sid = int(name)
                if sid not in self.id_to_joint:
                    continue
                leg, joint, jcalib = self.id_to_joint[sid]
                angle_deg = math.degrees(jcalib.ticks_to_angle(pos))
                self.latest_deg[(leg, joint)] = angle_deg
                buf = self.history.setdefault((leg, joint), deque())
                buf.append((now, angle_deg))
                while buf and now - buf[0][0] > self.window_s:
                    buf.popleft()

    def snapshot(self):
        with self._lock:
            return dict(self.latest_deg)

    def history_snapshot(self, key):
        with self._lock:
            buf = self.history.get(key)
            return list(buf) if buf else []


def run_table_mode(node):
    print('Waiting for data... (Ctrl-C to stop)\n')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            data = node.snapshot()
            if not data:
                continue

            lines = []
            lines.append('\033[H\033[J')  # clear screen, move cursor home
            lines.append('Live joint angles (degrees, actual position from servo telemetry)')
            lines.append('=' * 68)
            lines.append(f'{"Leg":<6}{"Coxa":>12}{"Femur":>12}{"Tibia":>12}')
            lines.append('-' * 68)
            for leg in geo.LEG_NAMES:
                row = f'{leg:<6}'
                for joint in ('coxa', 'femur', 'tibia'):
                    v = data.get((leg, joint))
                    row += f'{v:>11.1f} ' if v is not None else f'{"--":>11} '
                lines.append(row)
            lines.append('')
            lines.append('Compare mirrored legs (LF/RF, LM/RM, LR/RR) -- for symmetric')
            lines.append('commanded motion they should read close to equal-and-opposite.')
            print('\n'.join(lines))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


def run_plot_mode(node, compare_spec):
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    series = []
    for item in compare_spec.split(','):
        item = item.strip()
        if not item:
            continue
        leg, joint = item.split(':')
        leg = leg.strip().upper()
        joint = joint.strip().lower()
        if leg not in geo.LEG_NAMES or joint not in ('coxa', 'femur', 'tibia'):
            raise ValueError(f'invalid compare entry {item!r}, expected e.g. "LM:tibia"')
        series.append((leg, joint))

    if not series:
        raise ValueError('no valid entries in --compare')

    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(10, 5))
    lines = {}
    for leg, joint in series:
        (line,) = ax.plot([], [], label=f'{leg} {joint}')
        lines[(leg, joint)] = line

    ax.set_xlabel('time (s)')
    ax.set_ylabel('angle (deg)')
    ax.set_title('Live joint angle comparison (actual position, from servo telemetry)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    def update(_frame):
        all_t = []
        for key, line in lines.items():
            hist = node.history_snapshot(key)
            if not hist:
                continue
            ts, degs = zip(*hist)
            line.set_data(ts, degs)
            all_t.extend(ts)
        if all_t:
            ax.set_xlim(max(0, max(all_t) - node.window_s), max(all_t) + 0.5)
        ax.relim()
        ax.autoscale_view(scalex=False)
        return list(lines.values())

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
    plt.tight_layout()
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = LegAngleMonitorNode()

    if node.mode == 'plot':
        try:
            run_plot_mode(node, node.get_parameter('compare').value)
        except KeyboardInterrupt:
            pass
    else:
        run_table_mode(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
