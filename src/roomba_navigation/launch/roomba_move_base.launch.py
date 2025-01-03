# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():


    # File management
    nav2_launch_file_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    # Declare parameters
    param_map_file = DeclareLaunchArgument(
        name="map_file",
        default_value=os.path.join(
            get_package_share_directory("roomba_navigation"),
            "map",
            "roomba_world.yaml"
        ),
        description="Full path to map file to load",
    )

    param_config_file = DeclareLaunchArgument(
        name="config_file",
        default_value=os.path.join(
            get_package_share_directory("roomba_navigation"), 
            "config", 
            "roomba_config.yaml"
        ),
        description="Full path to map file to load",
    )

    param_use_slam = DeclareLaunchArgument(
        name="use_slam",
        default_value="True",
        description="Use SLAM launch files",
    )

    param_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Whether or not use simulation time",
    )
    
    # Solve for configurations
    map_file = LaunchConfiguration(
        param_map_file.name,
    )
    config_file = LaunchConfiguration(
        param_config_file.name,
    )

    use_slam = LaunchConfiguration(
        param_use_slam.name,
    )

    use_sim_time = LaunchConfiguration(
        param_use_sim_time.name
    )

    # Actions
    action_launch_include_with_namespace = GroupAction(
        actions=[
            # PushRosNamespace(""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir, "/bringup_launch.py"]),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": config_file,
                    "slam": use_slam,
                }.items(),   
            ),
        ]
    )

    # Launch description
    ld = LaunchDescription()

    ld.add_action(param_map_file)
    ld.add_action(param_config_file)
    ld.add_action(param_use_slam)
    ld.add_action(param_use_sim_time)
    ld.add_action(action_launch_include_with_namespace)

    return ld