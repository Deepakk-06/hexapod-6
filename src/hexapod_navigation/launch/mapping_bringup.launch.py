#!/usr/bin/env python3
# encoding: utf-8
"""
Phase 1: odometry + lidar, nothing else. Use this to confirm the lidar
is scanning correctly and the TF tree is sane before adding SLAM/Nav2.

    ros2 launch hexapod_navigation mapping_bringup.launch.py

Then in another terminal:
    rviz2 -d install/hexapod_navigation/share/hexapod_navigation/rviz/scan_view.rviz

You still need the walking stack running separately (sts3215_driver +
hexapod_control gait_node) if you want cmd_vel to actually move the
robot/update odometry -- this launch file only covers sensing.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hexapod_navigation'), 'launch', 'sensors.launch.py')
        )
    )

    odom_node = Node(
        package='hexapod_control',
        executable='odometry_node',
        name='cmd_vel_odometry_node',
        output='screen',
    )

    return LaunchDescription([
        sensors_launch,
        odom_node,
    ])
