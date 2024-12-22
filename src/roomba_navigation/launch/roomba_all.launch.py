from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    roomba_navigation_path_ir = get_package_share_directory("roomba_navigation")

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([roomba_navigation_path_ir, "/launch/roomba_amcl.launch.py"]),
    )

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([roomba_navigation_path_ir, "/launch/roomba_map_server.launch.py"]),
    )

    mobe_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([roomba_navigation_path_ir, "/launch/roomba_navigation.launch.py"]),
    )

    ld = LaunchDescription()
    ld.add_action(amcl_launch)
    ld.add_action(map_launch)
    # ld.add_action(mobe_base_launch)

    return ld