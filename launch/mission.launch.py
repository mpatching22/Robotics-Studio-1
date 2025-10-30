from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()
    pkg_path = FindPackageShare('trailblazer')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('world', default_value='simple_trees.sdf'))
    world = LaunchConfiguration('world')

    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_path, 'urdf_drone', 'parrot.urdf.xacro'])
        ]), value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher_node)

    robot_localization_node = Node(
        package='robot_localization', executable='ekf_node', name='robot_localization',
        parameters=[
            PathJoinSubstitution([config_path, 'robot_localization.yaml']),
            {'use_sim_time': use_sim_time}
        ]
    )
    ld.add_action(robot_localization_node)

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_ign_gazebo'),
            'launch',
            'ign_gazebo.launch.py'
        ]),
        launch_arguments={
            'ign_args': [
                PathJoinSubstitution([pkg_path, 'worlds', world]),
                ' -r'
            ]
        }.items()
    )
    ld.add_action(gazebo)

    robot_spawner = Node(
        package='ros_gz_sim', executable='create',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.2']
    )
    ld.add_action(robot_spawner)

    gazebo_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[
            {'config_file': PathJoinSubstitution([config_path, 'gazebo_bridge.yaml']),
             'use_sim_time': use_sim_time}
        ]
    )
    ld.add_action(gazebo_bridge)

    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,'41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    pose_relay = Node(
        package='trailblazer', executable='pose_relay.py', name='pose_relay',
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

    flight_control_node = Node(
        package='trailblazer', executable='flight_control.py', name='flight_control',
        output='screen',
        parameters=[{
            'control_rate_hz': 10.0,
            'rep_threshold': 3.0,
            'k_rep': 2.0
        }]
    )
    ld.add_action(flight_control_node)

    lidar_node = Node(
        package='trailblazer', executable='lidar_perception_360.py', name='lidar_perception_360',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'num_sectors': 8,
            'front_sector_deg': 60.0,
            'min_obs_dist': 3.0
        }]
    )
    ld.add_action(lidar_node)

    return ld
