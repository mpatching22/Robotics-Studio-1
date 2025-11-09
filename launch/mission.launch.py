#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file mission.launch.py
@brief Top-level launcher for the Trailblazer simulation.

@details
Starts the full simulation stack (URDF, EKF, Gazebo, bridges) and your project nodes.
By default it also launches the Navigation stack (SLAM Toolbox + Nav2) via
`navigation.launch.py` so you can simply run:

    ros2 launch trailblazer mission.launch.py

Use `rviz:=true` to enable RViz. You can also disable Nav2+SLAM with `nav2:=false`
if you want to bring up just the world and your nodes.

@par Key arguments
- `use_sim_time` (bool, default: true) Use simulation time.
- `world` (str, default: "final") Selects the SDF world.
- `rviz` (bool, default: false) Launch RViz.
- `nav2` (bool, default: true) Include navigation.launch.py (SLAM + Nav2).

@par Nodes started here
- robot_state_publisher (URDF)
- robot_localization (EKF)
- Gazebo (ros_ign_gazebo) + model spawn
- ros_ign_bridge parameter bridge
- tf2 static_transform_publisher (odom→base_link)
- trailblazer.pose_relay
- trailblazer.altitude_lidar
- trailblazer.gui_node
- trailblazer.flight_control

@note Gazebo Fortress prints a deprecation notice about `ign_args`. We keep it here
for compatibility with your current setup; switching to `gz_args` can be done later.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ---------------------------
    # Arguments
    # ---------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 visualiser'
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='true',
        description='Include navigation.launch.py (SLAM + Nav2)'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='final',
        choices=['simple_trees', 'large_demo', 'final_terrain', 'final'],
        description='Gazebo world to load'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---------------------------
    # Common paths
    # ---------------------------
    pkg_share = FindPackageShare('trailblazer')
    config_dir = PathJoinSubstitution([pkg_share, 'config'])

    # ---------------------------
    # Robot description + state publisher
    # ---------------------------
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_share, 'urdf_drone', 'parrot.urdf.xacro'])
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # ---------------------------
    # Robot localisation (EKF)
    # ---------------------------
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )

    # ---------------------------
    # Gazebo + model spawn
    # ---------------------------
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            # Keep ign_args for your current environment (see note in file header)
            'ign_args': [
                PathJoinSubstitution([pkg_share, 'worlds',
                                      [LaunchConfiguration('world'), '.sdf']]),
                ' -r'
            ]
        }.items()
    )

    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/robot_description',
            '-z', '0.2',
            '-Y', '1.5708'  # 90° CCW yaw
        ]
    )

    # ---------------------------
    # Gazebo–ROS 2 bridge
    # ---------------------------
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([config_dir, 'gazebo_bridge.yaml']),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # ---------------------------
    # Optional: RViz2
    # ---------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_dir, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # ---------------------------
    # Navigation stack (SLAM + Nav2)
    # ---------------------------
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_share, 'launch', 'navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )

    # ---------------------------
    # Minimal TF: odom → base_link
    # ---------------------------
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # ---------------------------
    # Trailblazer nodes
    # ---------------------------
    pose_relay = Node(
        package='trailblazer',
        executable='pose_relay.py',
        name='pose_relay',
        namespace='rs1',
        output='screen',
        parameters=[{
            'source_topic': '/odometry',
            'source_type': 'odom',
            'output_topic': '/drone/pose_1hz',
            'point_topic': '/drone/position',
            'rate_hz': 1.0
        }]
    )

    altitude_lidar = Node(
        package='trailblazer',
        executable='altitude_lidar.py',
        name='altitude_lidar',
        namespace='rs1',
        output='screen',
        parameters=[{
            'scan_topic': '/downscan',
            'center_deg': 0.0,
            'window_deg': 5.0
        }]
    )

    gui_node = Node(
        package='trailblazer',
        executable='gui_node.py',
        name='trailblazer_gui',
        namespace='rs1',
        output='screen'
    )

    # Only keep parameters your node actually declares/uses by default
    flight_control = Node(
        package='trailblazer',
        executable='flight_control.py',
        name='flight_control',
        namespace='rs1',
        output='screen',
        parameters=[{
            'control_rate_hz': 10.0
        }]
    )

    # ---------------------------
    # Assemble LaunchDescription
    # ---------------------------
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(nav2_arg)
    ld.add_action(world_arg)

    ld.add_action(robot_state_publisher)
    ld.add_action(robot_localization)
    ld.add_action(gazebo)
    ld.add_action(robot_spawner)
    ld.add_action(gazebo_bridge)
    ld.add_action(rviz)
    ld.add_action(navigation)
    ld.add_action(odom_to_base)
    ld.add_action(pose_relay)
    ld.add_action(altitude_lidar)
    ld.add_action(gui_node)
    ld.add_action(flight_control)

    return ld