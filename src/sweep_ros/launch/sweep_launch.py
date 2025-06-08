
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Node section
    sweep_node = Node(
        package='sweep_ros',
        executable='sweep_node',
        name='sweep_ros',
        output='screen',
      )    

    ld = LaunchDescription()
    ld.add_action(sweep_node)

    return ld