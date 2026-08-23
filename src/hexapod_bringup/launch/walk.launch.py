#!/usr/bin/env python3
# encoding: utf-8
"""
Brings up the full walking stack: STS3215 servo driver, MPU6050 IMU, and
the gait control node.

    ros2 launch hexapod_bringup walk.launch.py

Run stand_up once first (see README) before this, or set the
auto_stand_up launch argument to true.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_path = PathJoinSubstitution([
        get_package_share_directory('hexapod_control'), 'config', 'servo_config.yaml'
    ])

    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')
    gait_arg = DeclareLaunchArgument('gait', default_value='tripod')
    period_arg = DeclareLaunchArgument('step_period_s', default_value='1.0')
    # bus 2 (pins 27/28) is where the IMU was last physically wired during
    # testing -- note the MPU6050 was never actually confirmed working on
    # this hardware, see README section 3b
    imu_bus_arg = DeclareLaunchArgument('i2c_bus', default_value='2')
    leveling_arg = DeclareLaunchArgument('leveling_enabled', default_value='true')

    driver_node = Node(
        package='sts3215_driver',
        executable='driver_node',
        name='sts3215_driver',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': 1000000,
        }],
    )

    imu_node = Node(
        package='mpu6050_driver',
        executable='imu_node',
        name='mpu6050_imu',
        output='screen',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
        }],
    )

    gait_node = Node(
        package='hexapod_control',
        executable='gait_node',
        name='hexapod_gait_node',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'gait': LaunchConfiguration('gait'),
            'step_period_s': LaunchConfiguration('step_period_s'),
            'leveling_enabled': LaunchConfiguration('leveling_enabled'),
        }],
    )

    return LaunchDescription([
        port_arg, gait_arg, period_arg, imu_bus_arg, leveling_arg,
        driver_node,
        imu_node,
        gait_node,
    ])
