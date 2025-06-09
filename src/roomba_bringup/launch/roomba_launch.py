from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    sweep_ros_path_ir = get_package_share_directory("sweep_ros")
    create_bringup_path_ir = get_package_share_directory("create_bringup")

    create_param_config_file = DeclareLaunchArgument(
        name="config_file",
        default_value=os.path.join(
            get_package_share_directory("roomba_brigup"), 
            "config", 
            "create_default.yaml"
        )
    )
    create_config_file = LaunchConfiguration(
        create_param_config_file.name,
    )

    sweep_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sweep_ros_path_ir, "/launch/sweep2laser.py"]),
    )

    roomba_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create_bringup_path_ir, "/launch/create_2.launch"]),
        parameters=[create_config_file],
    )


    ld = LaunchDescription()
    ld.add_action(sweep_launch)
    ld.add_action(roomba_launch)


    return ld