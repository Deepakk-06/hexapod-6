#!/usr/bin/env python3
# encoding: utf-8
"""
cmd_vel_odometry_node

Publishes nav_msgs/Odometry on 'odom' and broadcasts the odom->base_link
TF transform by integrating cmd_vel over time (standard unicycle dead
reckoning). This is OPEN LOOP -- it assumes the robot actually achieves
the commanded velocity, which for a gait-walking hexapod with no wheel
encoders is an approximation, not a measurement. It will drift over
time and distance, more than a wheeled robot with real encoders would.

It's good enough to let SLAM/Nav2 function (they need *some* odom source
for short-term motion prediction between lidar scan matches), but don't
expect long-duration dead-reckoning accuracy. If IMU yaw ever becomes
available and is trustworthy, that's the natural next improvement --
substitute integrated gyro yaw for the current pure cmd_vel.angular.z
integration, since gyro is a real measurement rather than an assumption.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class CmdVelOdometryNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_odometry_node')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('publish_tf', True)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('cmd_vel_odometry_node ready (open-loop dead reckoning)')

    def on_cmd_vel(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        # unicycle-model integration in the odom frame
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # open-loop dead reckoning -- flag high uncertainty that grows unbounded,
        # so any consumer (e.g. a future EKF fusing IMU) knows not to trust this alone
        odom.pose.covariance[0] = 0.05    # x
        odom.pose.covariance[7] = 0.05    # y
        odom.pose.covariance[35] = 0.1    # yaw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
