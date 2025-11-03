#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # === Paths ===
    pkg_path = FindPackageShare('trailblazer')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # === Launch Args ===
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('world', default_value='simple_trees.sdf'))

    # === Robot State Publisher ===
    robot_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}],
    )
    ld.add_action(robot_state_publisher)

    # === Robot Localization (EKF) ===
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time},
        ],
    )
    ld.add_action(ekf_node)

    # === Gazebo (ros_gz_sim) ===
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([pkg_path, 'worlds', world]), ' -r']
        }.items(),
    )
    ld.add_action(gazebo)

    # === Spawn drone ===
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-name', 'parrot', '-z', '0.25'],
    )
    ld.add_action(spawn)

    # === Bridge ===
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
                'use_sim_time': use_sim_time,
            }
        ],
    )
    ld.add_action(bridge)

    # === Optional RViz ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(rviz),
    )
    ld.add_action(rviz_node)

    # === Simple TF broadcaster (odom → base_link) ===
    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )
    ld.add_action(odom_tf)

    # === Pose relay ===
    pose_relay = Node(
        package='trailblazer',
        executable='pose_relay.py',
        name='pose_relay',
        namespace='rs1',
        output='screen',
        parameters=[
            {
                'source_topic': '/odometry',
                'source_type': 'odom',
                'output_topic': '/drone/pose_1hz',
                'point_topic': '/drone/position',
                'rate_hz': 1.0,
            }
        ],
    )
    ld.add_action(pose_relay)

    # === Altitude lidar ===
    altitude_lidar = Node(
        package='trailblazer',
        executable='altitude_lidar.py',
        name='altitude_lidar',
        namespace='rs1',
        output='screen',
        parameters=[{'scan_topic': '/downscan', 'center_deg': 0.0, 'window_deg': 5.0}],
    )
    ld.add_action(altitude_lidar)

    # === GUI ===
    gui_node = Node(
        package='trailblazer',
        executable='gui_node.py',
        name='trailblazer_gui',
        namespace='rs1',
        output='screen',
    )
    ld.add_action(gui_node)

    # === LiDAR perception ===
    lidar_node = Node(
        package='trailblazer',
        executable='lidar_perception_360.py',
        name='lidar_perception_360',
        namespace='rs1',
        output='screen',
        parameters=[
            {
                'scan_topic': '/scan',
                'num_sectors': 8,
                'front_sector_deg': 60.0,
                'min_obs_dist': 3.0,
            }
        ],
    )
    ld.add_action(lidar_node)

    # === Flight control ===
    flight_control = Node(
        package='trailblazer',
        executable='flight_control.py',
        name='flight_control',
        namespace='rs1',
        output='screen',
        parameters=[{'control_rate_hz': 10.0, 'rep_threshold': 3.0, 'k_rep': 2.0}],
    )
    ld.add_action(flight_control)

    return ld
