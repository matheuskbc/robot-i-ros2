from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    sweep_ros_path_ir = get_package_share_directory("sweep_ros")

    sweep_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sweep_ros_path_ir, "/launch/sweep_launch.py"]),
    )

    point_cloud_laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sweep_ros_path_ir, "/launch/pointcloud2laser_launch.py"]),
    )

    ld = LaunchDescription()
    ld.add_action(sweep_launch)
    ld.add_action(point_cloud_laser_launch)

    return ld