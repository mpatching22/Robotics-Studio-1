from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('trailblazer')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # Load robot_description and start robot_state_publisher
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time': use_sim_time
                                      }])
    ld.add_action(robot_state_publisher_node)

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Start Gazebo to simulate the robot in the chosen world
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

    # Spawn robot in Gazebo
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-topic', '/robot_description',
            '-x', '1.5',      # Position between obstacles
            '-y', '1.5',      # Trees are at (0,3) and (3,0)
            '-z', '0.5',      # Low altitude for testing (was 10.0!)
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ]
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']), 
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,
                              'launch',
                              'navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
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
            'source_topic': '/odometry',     # or '/odometry'
            'source_type': 'odom',             # 'odom' if using /odometry
            'output_topic': '/drone/pose_1hz',
            'point_topic': '/drone/position',
            'rate_hz': 1.0
        }]
    )
    ld.add_action(pose_relay)

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[PathJoinSubstitution([config_path, 'nav2_params.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(amcl)

    # climb_controller = Node(
    #     package='trailblazer',
    #     executable='climb_controller.py',
    #     name='climb_controller',
    #     output='screen',
    #     parameters=[{
    #         'pose_topic': '/drone/pose_1hz',  # Use 1Hz pose from relay
    #         'msg_type': 'pose',               # PoseStamped messages
    #         'speed_mps': 0.1,                 # 0.1 m/s climb rate
    #         'target_z': 15.0,                 # Climb to 15m 
    #         'publish_rate_hz': 10.0,          # 10Hz command rate
    #         'timeout_sec': 3.0                # 3s timeout for pose data
    #     }]
    # )
    # ld.add_action(climb_controller)       

    # perception = Node(
    #     package='trailblazer',
    #     executable='lidar_perception_360.py',
    #     name='lidar_perception_360',
    #     output='screen',
    #     parameters=[{
    #         'scan_topic': '/scan',
    #         'num_sectors': 8,
    #         'front_sector_deg': 30.0,
    #         'min_obs_dist': 3.0,
    #         'use_sim_time': use_sim_time
    #     }]
    # )
    # ld.add_action(perception)
                                                                                      

    # gui_node = Node(
    #     package='trailblazer',              # your package name
    #     executable='gui.node.py',           # the Python entrypoint file
    #     name='trailblazer_gui',
    #     output='screen'
    # )
    # ld.add_action(gui_node)

    # flight_control_node = Node(
    #     package='trailblazer',
    #     executable='flight.control.py',   # must match installed filename
    #     name='flight_control',
    #     output='screen',
    #     parameters=[{
    #         'control_rate_hz': 10.0
    #     }]
    # )
    # ld.add_action(flight_control_node)

    return ld