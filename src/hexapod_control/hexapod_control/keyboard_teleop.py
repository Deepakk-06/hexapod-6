#!/usr/bin/env python3
# encoding: utf-8
"""
Simple keyboard teleop -> geometry_msgs/Twist on cmd_vel.

  w/s : forward / backward
  a/d : strafe left / right
  q/e : turn left / right
  space : stop
  +/- : increase / decrease speed
  ctrl-c to quit
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

KEY_BINDINGS = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('linear_speed', 0.04)   # m/s -- keep under max_stride_mm/period_s/1000
        self.declare_parameter('angular_speed', 0.3)   # rad/s
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

    def get_key(self, settings, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(__doc__)
        try:
            while rclpy.ok():
                key = self.get_key(settings)
                twist = Twist()
                if key in KEY_BINDINGS:
                    vx, vy, wz = KEY_BINDINGS[key]
                    twist.linear.x = vx * self.linear_speed
                    twist.linear.y = vy * self.linear_speed
                    twist.angular.z = wz * self.angular_speed
                    self.pub.publish(twist)
                elif key == ' ':
                    self.pub.publish(Twist())
                elif key == '+':
                    self.linear_speed += 0.01
                elif key == '-':
                    self.linear_speed = max(0.01, self.linear_speed - 0.01)
                elif key == '\x03':  # ctrl-c
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
