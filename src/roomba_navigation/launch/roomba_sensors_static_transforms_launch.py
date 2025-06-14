import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    pkg_share_description = FindPackageShare(package='create_description').find('create_description')
    default_model_path = os.path.join(pkg_share_description, 'urdf/create_2.urdf.xacro')

    param_declare_model_path = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path, 
        description='Absolute path to robot urdf file'
    )
    model = LaunchConfiguration(param_declare_model_path.name)


    start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[
      { 
      'robot_description': Command(['xacro ', model])
      }
    ],
    arguments=[default_model_path]
    )

    imu_static_publisher = Node(
        name="imu_static_publisher",
        executable="static_transform_publisher",
        package="tf2_ros",
        output="screen",
        arguments=["-0.105", "0", "0.126", "0", "0", "0", "base_footprint", "imu_link"]
    )
    
    lidar_static_publisher = Node(
        name="lidar_static_publisher",
        executable="static_transform_publisher",
        package="tf2_ros",
        output="screen",
        arguments=["0", "0", "0.17", "0", "0", "0", "base_footprint", "laser_frame"]
    )

    ld = LaunchDescription()
    ld.add_action(imu_static_publisher)
    ld.add_action(lidar_static_publisher)
    ld.add_action(param_declare_model_path)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld

