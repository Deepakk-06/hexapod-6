#!/usr/bin/env python3
# encoding: utf-8
"""
Phase 3: full autonomous navigation using a previously saved map.

    ros2 launch hexapod_navigation navigation.launch.py map:=/path/to/my_map.yaml

Save a map first with slam.launch.py (see README), then point this at it.

Brings up: lidar + static TF + odometry (sensors.launch.py's contents,
via mapping_bringup.launch.py) + map_server + amcl (localizes against
the saved map) + planner_server + controller_server + smoother_server +
behavior_server + bt_navigator + waypoint_follower, all managed by a
single lifecycle manager, using nav2_params.yaml (tuned for this
robot's actual ~0.06 m/s max speed and ~0.30m footprint -- NOT generic
wheeled-robot defaults).

You still need the walking stack running separately (sts3215_driver +
hexapod_control gait_node) for cmd_vel commands to actually move the
robot.

View with:
    ros2 launch nav2_bringup rviz_launch.py
(nav2_bringup's default RViz config includes costmap/plan/goal-setting
tools that our own scan_view.rviz doesn't -- use nav2's instead here)

Send a goal either via RViz's "2D Goal Pose" tool, or from the command line:
    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \\
      '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' --once
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('hexapod_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument('map', description='Full path to the saved map .yaml file')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
    )
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')

    mapping_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'mapping_bringup.launch.py')
        )
    )

    # nav2_bringup's own bringup_launch.py starts map_server, amcl,
    # planner_server, controller_server, smoother_server, behavior_server,
    # bt_navigator, waypoint_follower, and the lifecycle manager that
    # brings them all up in the correct order -- reusing it rather than
    # re-declaring each node ourselves, since that's a lot of boilerplate
    # nav2_bringup already gets right.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        map_arg, params_arg, autostart_arg,
        mapping_bringup,
        nav2_bringup,
    ])
