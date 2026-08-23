#!/usr/bin/env python3
# encoding: utf-8
"""
Brings up the RPLidar A1M8 and the static TF frames needed for
SLAM/Nav2: base_footprint -> base_link -> laser.

The lidar mount position/orientation is exposed as launch arguments
since it can't be measured from a photo -- tune these once you can see
scan output in RViz (see README section on lidar calibration).

    ros2 launch hexapod_navigation sensors.launch.py

To adjust the mount further (e.g. if you remeasure x/y/z offset):
    ros2 launch hexapod_navigation sensors.launch.py \\
        lidar_x:=0.01 lidar_y:=0.0 lidar_z:=0.10 lidar_yaw:=-1.5708
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0')
    frame_arg = DeclareLaunchArgument('laser_frame', default_value='laser')

    # Lidar mount pose relative to base_link, in meters/radians.
    # Defaults: centered on top, elevated ~100mm. yaw=-90deg (-1.5708 rad)
    # confirmed empirically on hardware -- the lidar's 0-degree reference
    # points 90 degrees off from the robot's forward direction, corrected here.
    x_arg = DeclareLaunchArgument('lidar_x', default_value='0.0')
    y_arg = DeclareLaunchArgument('lidar_y', default_value='0.0')
    z_arg = DeclareLaunchArgument('lidar_z', default_value='0.10')
    yaw_arg = DeclareLaunchArgument('lidar_yaw', default_value='-1.5708')

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        # base_link is the PARENT here (odometry publishes odom->base_link,
        # so base_link needs exactly one parent -- odom -- and base_footprint
        # hangs off it as a child, not the other way around, or TF ends up
        # with two disconnected trees).
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=[
            LaunchConfiguration('lidar_x'), LaunchConfiguration('lidar_y'), LaunchConfiguration('lidar_z'),
            LaunchConfiguration('lidar_yaw'), '0', '0',
            'base_link', LaunchConfiguration('laser_frame'),
        ],
    )

    # rplidar_ros package -- install with:
    #   sudo apt install ros-humble-rplidar-ros
    # If the executable name below doesn't match your installed version,
    # check with: ros2 pkg executables rplidar_ros
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,   # A1M8 default
            'frame_id': LaunchConfiguration('laser_frame'),
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    return LaunchDescription([
        port_arg, frame_arg, x_arg, y_arg, z_arg, yaw_arg,
        base_footprint_tf,
        laser_tf,
        rplidar_node,
    ])
