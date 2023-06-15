#!/usr/bin/env python3
#
# ... (License and author information)

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    slam_r1 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ramel'),'launch','slam.launch.py')),
    	launch_arguments={
    		'namespace': 'tb3',
    		'mode':'localization',
    		'initial_pose':'[-2.0, -2.0, 0.01]',
        }.items()
    )
    slam_r2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ramel'),'launch','slam.launch.py')),
    	launch_arguments={
    		'namespace': 'tb3_2',
    		'mode':'localization',
    		'initial_pose':'[1.0, 1.0, 0.01]',
        }.items()
    )
    # Create the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('ramel'),'config','slam_config.rviz')]
    )
    nav1 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ramel'),'launch','navigation_launch.py')),
    	launch_arguments={
    		'namespace': 'tb3',
        }.items()
    )
    nav2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ramel'),'launch','navigation_launch.py')),
    	launch_arguments={
    		'namespace': 'tb3_2',
        }.items()
    )
    
    # Add the commands to the launch description
    # Define LaunchDescription variable
    ld = LaunchDescription()
    ld.add_action(slam_r1)
    ld.add_action(slam_r2)
    ld.add_action(rviz_node)
    #ld.add_action(nav1)
    #ld.add_action(nav2)
    return ld
