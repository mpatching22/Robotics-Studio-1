from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = ['/path/to/nav2_params.yaml']  # <-- your edited yaml

    planner = Node(
        package='nav2_planner', executable='planner_server', output='screen',
        parameters=params)
    controller = Node(
        package='nav2_controller', executable='controller_server', output='screen',
        parameters=params)
    smoother = Node(
        package='nav2_smoother', executable='smoother_server', output='screen',
        parameters=params)
    behavior = Node(
        package='nav2_behaviors', executable='behavior_server', output='screen',
        parameters=params)
    bt_nav = Node(
        package='nav2_bt_navigator', executable='bt_navigator', output='screen',
        parameters=params)

    # This is the critical piece: it activates all lifecycle Nav2 servers
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', output='screen',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator'
            ],
        }])

    return LaunchDescription([planner, controller, smoother, behavior, bt_nav, lifecycle])
