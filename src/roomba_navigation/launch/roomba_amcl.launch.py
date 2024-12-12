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
            "roomba_config.yaml"
        )
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    start_amcl = GroupAction(
    actions=[
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn='false',
            respawn_delay=2.0,
            # parameters=[LaunchConfiguration(param_config_file.name)],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'autostart': True}, {'node_names': ['amcl']}],
        ),
    ]
    )

    ld = LaunchDescription()
    ld.add_action(param_config_file)
    ld.add_action(start_amcl)
    return ld