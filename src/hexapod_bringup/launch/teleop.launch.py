#!/usr/bin/env python3
# encoding: utf-8
"""
Standalone keyboard teleop launcher, run alongside walk.launch.py in a
second terminal:

    ros2 launch hexapod_bringup teleop.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package='hexapod_control',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',  # run in its own terminal so raw keyboard input works
    )
    return LaunchDescription([teleop_node])
