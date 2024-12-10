# Author: Addison Sears-Collins
# Date: August 31, 2021
# Description: Launch a basic mobile robot
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  # File management section
  pkg_share_description = FindPackageShare(package='create_description').find('create_description')
  pkg_share_navigation = FindPackageShare(package='roomba_navigation').find('roomba_navigation')
  default_path_static_launch = os.path.join(pkg_share_navigation, "launch/roomba_sensors_static_transforms.py")
  default_model_path = os.path.join(pkg_share_description, 'urdf/create_2.urdf.xacro')
  robot_localization_file_path = os.path.join(pkg_share_navigation, 'config/ekf.yaml') 

  # Declare the launch arguments 

  param_declare_model_path = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file'
  )
        
  param_declare_use_robot_state_pub = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher'
  )

  # Launch config variables
  model = LaunchConfiguration(param_declare_model_path.name)
  use_robot_state_pub = LaunchConfiguration(param_declare_use_robot_state_pub.name)
   
  # Specify the actions

  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path]
  )

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[
      { 
      'robot_description': Command(['xacro ', model])
      }
    ],
    arguments=[default_model_path]
  )

  incude_static_transforms = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [default_path_static_launch]
    )
  )
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(param_declare_model_path)
  ld.add_action(param_declare_use_robot_state_pub)

  # Add any actions
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(incude_static_transforms)

  return ld