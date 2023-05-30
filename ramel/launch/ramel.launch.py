import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Launch realsense2_camera node
    realsense_camera_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'aruco_ros2', 'marker_publisher.launch.xml'],
        output='screen'
    )

    # Launch aruco_ros2 node
    aruco_marker_publisher_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 'rgb_camera.profile:=1920x1080x30'],
        output='screen'
    )

     # Launch RViz2 with specific configuration
    rviz_config_file = get_package_share_directory('ramel') + '/config/rviz_config.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        realsense_camera_node,
        aruco_marker_publisher_node,
        rviz_node
    ])
