from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Node section
    point_cloud_to_laser_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
          {"target_frame": "laser_frame"},
          {"transform_tolerance": 0.001},
          {"min_height": -1.0},
          {"max_height": 1.0} ,
          {"angle_min": -3.14},
          {"angle_max": 3.14},
          {"angle_increment": 0.001},
          {"scan_time": 0.1},
          {"range_min": 0.0},
          {"range_max": 40.0},
          {"use_inf": True},
          {"concurrency_level": 1}
        ],
        remappings=[
            ("/cloud_in", "/pc2")
        ]
      )    

    ld = LaunchDescription()
    ld.add_action(point_cloud_to_laser_scan_node)

    return ld