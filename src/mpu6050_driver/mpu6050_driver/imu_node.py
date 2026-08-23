#!/usr/bin/env python3
# encoding: utf-8
"""
Publishes sensor_msgs/Imu, and also a lightweight (roll, pitch) estimate
via complementary filter on a separate topic for the gait controller to
consume directly (avoids every consumer re-deriving RPY from quaternions).
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf_transformations import quaternion_from_euler

from mpu6050_driver.mpu6050 import MPU6050


class ImuNode(Node):
    def __init__(self):
        super().__init__('mpu6050_imu')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('complementary_alpha', 0.98)
        self.declare_parameter('frame_id', 'imu_link')

        bus_num = self.get_parameter('i2c_bus').value
        addr = self.get_parameter('i2c_address').value
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.alpha = float(self.get_parameter('complementary_alpha').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.imu = MPU6050(bus_num=bus_num, address=addr)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_t = time.time()

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.rpy_pub = self.create_publisher(Vector3Stamped, 'imu/roll_pitch_yaw', 10)

        self.timer = self.create_timer(1.0 / rate, self.update)
        self.get_logger().info('mpu6050_imu ready')

    def update(self):
        now = time.time()
        dt = max(1e-3, now - self._last_t)
        self._last_t = now

        ax, ay, az = self.imu.read_accel()
        gx, gy, gz = self.imu.read_gyro()

        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.hypot(ay, az))

        self.roll = self.alpha * (self.roll + gx * dt) + (1.0 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1.0 - self.alpha) * accel_pitch
        self.yaw += gz * dt  # gyro-only, will drift -- fine for short-term leveling use

        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        qx, qy, qz, qw = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.linear_acceleration.x = ax * 9.80665
        imu_msg.linear_acceleration.y = ay * 9.80665
        imu_msg.linear_acceleration.z = az * 9.80665
        self.imu_pub.publish(imu_msg)

        rpy_msg = Vector3Stamped()
        rpy_msg.header.stamp = stamp
        rpy_msg.header.frame_id = self.frame_id
        rpy_msg.vector.x = self.roll
        rpy_msg.vector.y = self.pitch
        rpy_msg.vector.z = self.yaw
        self.rpy_pub.publish(rpy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
