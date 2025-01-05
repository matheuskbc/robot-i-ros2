from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    param_config_file = DeclareLaunchArgument(
        name="config_file",
        default_value=os.path.join(
            get_package_share_directory("roomba_navigation"), 
            "config", 
            "map_config.yaml"
        ),
    )
    param_map_file = DeclareLaunchArgument(
        name="map_file",
        default_value=os.path.join(
            get_package_share_directory("roomba_navigation"), 
            "map", 
            "house.yaml"
        ),
    )
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    start_map_server = GroupAction(
    actions=[
        Node(
            name="map_server",
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[
                LaunchConfiguration(param_config_file.name), 
                {'yaml_filename': LaunchConfiguration(param_map_file.name)}
            ],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[
                {'autostart': True}, 
                {'node_names': ['map_server']}
            ],
        ),
    ]
    )

    ld = LaunchDescription()
    ld.add_action(param_map_file)
    ld.add_action(param_config_file)
    ld.add_action(start_map_server)
    return ld