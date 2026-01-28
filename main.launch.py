from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                '/workspace/path_planning_example/config/planner_server_params.yaml'
            ]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                '/workspace/path_planning_example/config/map_server_params.yaml'
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[
                {'autostart': True,
                 'bond_disable_heartbeat_timeout': True,
                 'node_names': ['map_server', 'planner_server']}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_publisher',
            output='screen',
            arguments=['20.0', '15.0', '0.0', '0.0', '0.0', '1.0', '0.0', 'map', 'base_link']
        )
    ])
