from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare('trailblazer')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher_node)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees.sdf',
        choices=['simple_trees.sdf', 'large_demo.sdf'],
        description='Which world to load'
    )
    ld.add_action(world_launch_arg)

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([pkg_path, 'worlds', LaunchConfiguration('world')]),
                ' -r'
            ]
        }.items()
    )
    ld.add_action(gazebo)

    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/robot_description',
            '-x', '1.5',
            '-y', '1.5',
            '-z', '1.0',  # Recommended starting altitude
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ]
    )
    ld.add_action(robot_spawner)

    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
                     'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                             'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_minimal.yaml'])
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(slam)

    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                              'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml'])
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    pose_relay = Node(
        package='trailblazer',
        executable='pose_relay.py',
        name='pose_relay',
        output='screen',
        parameters=[{
            'source_topic': '/odometry',
            'source_type': 'odom',
            'output_topic': '/drone/pose_1hz',
            'point_topic': '/drone/position',
            'rate_hz': 1.0
        }]
    )
    ld.add_action(pose_relay)

    robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    parameters=[PathJoinSubstitution([config_path, 'robot_localization.yaml'])],
    name='ekf_filter_node',
    output='screen'
    )
    ld.add_action(robot_localization_node)

    altitude_mixer = Node(
    package='trailblazer',
    executable='altitude_mixer.py',
    name='altitude_mixer',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'z_target': 1.5   # or 2.0 m, adjust as needed
    }]
    )
    ld.add_action(altitude_mixer)

    waypoint_planner = Node(
        package='trailblazer',
        executable='waypoint_planner.py',
        name='waypoint_planner',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'nav_cmd_topic': '/nav/cmd_vel',
            'slope_topic': '',            # leave '' unless you add a slope node
            'forward_speed': 1.0,
            'slow_distance': 5.0,
            'stop_distance': 2.0,
            'yaw_rate_max': 1.0,
            'sector_count': 12,
            'front_fov_deg': 90.0,
            'slope_limit_deg': 15.0,
            'slope_stop_deg': 25.0,
            'hz': 10.0,
            'goal_distance': 10.0          # set >0 to "go N meters then stop"
        }]
    )
    ld.add_action(waypoint_planner)



    return ld

