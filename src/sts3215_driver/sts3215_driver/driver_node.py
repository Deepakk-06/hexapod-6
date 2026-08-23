#!/usr/bin/env python3
# encoding: utf-8
"""
sts3215_driver_node

Subscribes:  servo_controller   (hexapod_msgs/ServosPosition)  -> drives servos
Publishes:   joint_raw_states   (sensor_msgs/JointState)       -> raw tick positions (name = str(servo_id))

Keeps the same "publish a batch with a duration, board fans out to all
servos" pattern the stock ROSpider software uses, just talking real
SCServo protocol underneath instead of their board's firmware.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from hexapod_msgs.msg import ServosPosition

from sts3215_driver.scservo_protocol import SCServoBus


class STS3215DriverNode(Node):
    def __init__(self):
        super().__init__('sts3215_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('servo_ids', list(range(1, 19)))
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('torque_on_startup', True)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.servo_ids = list(self.get_parameter('servo_ids').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.get_logger().info(f'Opening SCServo bus on {port} @ {baud}')
        self.bus = SCServoBus(port=port, baudrate=baud)

        if self.get_parameter('torque_on_startup').value:
            for sid in self.servo_ids:
                self.bus.enable_torque(sid, True)

        self.sub = self.create_subscription(
            ServosPosition, 'servo_controller', self.on_servo_cmd, 10)

        self.state_pub = self.create_publisher(JointState, 'joint_raw_states', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_states)

        self.get_logger().info('sts3215_driver ready')

    def on_servo_cmd(self, msg: ServosPosition):
        duration_ms = max(0.0, msg.duration) * 1000.0
        batch = []
        for p in msg.position:
            batch.append((int(p.id), int(round(p.position)), duration_ms, 0))
        if batch:
            self.bus.sync_set_positions(batch)

    def publish_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for sid in self.servo_ids:
            pos = self.bus.read_position(sid)
            if pos is None:
                continue
            js.name.append(str(sid))
            js.position.append(float(pos))
        if js.name:
            self.state_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = STS3215DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
