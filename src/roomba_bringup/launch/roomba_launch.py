import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    namespace=""
    yaml_substitutions = {
        'KEEPOUT_ZONE_ENABLED': True,
        'SPEED_ZONE_ENABLED': True,
    }
    map_path="/usr/src/ros2_ws/src/roomba_navigation/map/house_mettmanner_5.yaml"
    nav2_param=os.path.join(get_package_share_directory("roomba_bringup"), "config", "nav2_param.yaml")
    use_sim_time = "false"
    autostart = "true"
    use_composition = "true"


    rplidar_path_ir = get_package_share_directory("rplidar_ros")
    roomba_bringup_path_ir = get_package_share_directory("roomba_bringup")
    nav2_bringup_path_ir = get_package_share_directory("nav2_bringup")

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rplidar_ros_path_ir, "/launch/rplidar_a1_launch.py"]),
        launch_arguments={
            "frame_id": "laser_frame",
            "serial_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
        }.items()
    )

    roomba_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([roomba_bringup_path_ir, "/launch/create_2.launch"]),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_path_ir, "/launch/bringup_launch.py"]),
        launch_arguments={
            "map" : map_path,
            "params_file": nav2_param,
            "use_sim_time" : use_sim_time,
            "autostart": autostart,
            "use_composition": use_composition,
        }.items()
    )

    roomba_navigation_path_ir = get_package_share_directory("roomba_navigation")

    transforms_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([roomba_navigation_path_ir, "/launch/roomba_sensors_static_transforms_launch.py"]),
    )

    ld = LaunchDescription()
    ld.add_action(rplidar_launch)
    ld.add_action(roomba_launch)
    ld.add_action(transforms_launch)
    ld.add_action(nav2_launch)

    return ld
