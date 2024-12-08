from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    imu_static_publisher = Node(
        name="imu_static_publisher",
        executable="static_transform_publisher",
        package="tf2_ros",
        output="screen",
        arguments=["-0.105", "0", "0.126", "0", "0", "1", "base_footprint", "imu_link"]
    )
    
    lidar_static_publisher = Node(
        name="lidar_static_publisher",
        executable="static_transform_publisher",
        package="tf2_ros",
        output="screen",
        arguments=["0", "0", "0.17", "0", "0", "1", "base_footprint", "laser_frame"]
    )

    ld = LaunchDescription()
    ld.add_action(imu_static_publisher)
    ld.add_action(lidar_static_publisher)
    return ld

