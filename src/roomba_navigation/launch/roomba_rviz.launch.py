
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from os.path import join


def generate_launch_description():

    # File management section
    roomba_navigation_path = FindPackageShare("roomba_navigation").find("roomba_navigation")

    # Params declaration section
    param_use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="True",
        description="Flag to start RVIZ"
    )

    param_rviz_file_path = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=join(roomba_navigation_path, "rviz/urdf_config.rviz"),
    )

    # Launch configurations
    param_rviz_file_path_ = LaunchConfiguration(param_rviz_file_path.name)
    param_use_rviz_ = LaunchConfiguration(param_use_rviz.name)

    # Node section
    rviz_node = Node(
        condition=IfCondition(param_use_rviz_),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', param_rviz_file_path_]
      )    

    ld = LaunchDescription()
    ld.add_action(param_use_rviz)
    ld.add_action(param_rviz_file_path)
    ld.add_action(rviz_node)

    return ld