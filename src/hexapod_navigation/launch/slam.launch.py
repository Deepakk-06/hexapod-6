#!/usr/bin/env python3
# encoding: utf-8
"""
Phase 2: SLAM mapping. Starts sensors + odometry + slam_toolbox.

    ros2 launch hexapod_navigation slam.launch.py

Requires: sudo apt install ros-humble-slam-toolbox

Drive the robot around (teleop or otherwise) to build a map, then save it:
    ros2 run nav2_map_server map_saver_cli -f ~/my_map

View live in RViz:
    rviz2 -d install/hexapod_navigation/share/hexapod_navigation/rviz/scan_view.rviz
(add a Map display pointed at /map to also see the map being built)
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('hexapod_navigation')

    mapping_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'mapping_bringup.launch.py')
        )
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(nav_share, 'config', 'slam.yaml')],
    )

    return LaunchDescription([
        mapping_bringup,
        slam_toolbox_node,
    ])
