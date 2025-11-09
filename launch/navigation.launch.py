#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file navigation.launch.py
@brief Brings up SLAM Toolbox (online_async) and the Nav2 bringup.

@details
This file is included by `mission.launch.py` (controlled by its `nav2` argument),
and launches both SLAM and Nav2 with your configuration files.

@par Key arguments
- `use_sim_time` (bool, default: true) Use simulation time.

@par Includes
- slam_toolbox/online_async_launch.py (with `slam_params.yaml`)
- nav2_bringup/navigation_launch.py (with `nav2_params.yaml`)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ---------------------------
    # Arguments
    # ---------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---------------------------
    # Paths
    # ---------------------------
    pkg_share = FindPackageShare('trailblazer')
    config_dir = PathJoinSubstitution([pkg_share, 'config'])

    # ---------------------------
    # SLAM Toolbox
    # ---------------------------
    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                              'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_dir, 'slam_params.yaml'])
        }.items()
    )

    # ---------------------------
    # Nav2 bringup
    # ---------------------------
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                              'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_dir, 'nav2_params.yaml'])
        }.items()
    )

    # ---------------------------
    # Assemble LaunchDescription
    # ---------------------------
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam)
    ld.add_action(nav2)
    return ld