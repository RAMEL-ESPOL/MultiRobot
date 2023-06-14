import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():   
    # Namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='tb3',
        description='Namespace for the robot'
    )
    # Create the launch description
    ld = LaunchDescription()
    # Add the namespace argument
    ld.add_action(namespace_arg)
    
    # Get the string parameter passed from the main launch file
    ns = LaunchConfiguration('namespace')
    # Define the concatenated strings using the namespace variable
    base_frame = [LaunchConfiguration('namespace'), '_base_footprint']
    odom_frame = [LaunchConfiguration('namespace'), '_odom']
    map_frame = [LaunchConfiguration('namespace'), '_map']

    # Run slam_toolbox's online_sync node and remap the parameters
    ld.add_action(Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'base_frame': base_frame},
            {'odom_frame': odom_frame},
            {'map_frame': map_frame},
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('/map', ['/',LaunchConfiguration('namespace'), '/map']),
            ('/scan',['/',LaunchConfiguration('namespace'), '/scan']),
            ('/initialpose', ['/',LaunchConfiguration('namespace'), '/initialpose']),
        ],
    ))

    return ld

