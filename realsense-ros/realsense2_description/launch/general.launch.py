#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_cam_description = get_package_share_directory('realsense2_description')

    # Sart World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cam_description, 'launch', 'gazebo_init.launch.py'),
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cam_description, 'launch', 'l515_spawn.launch.py'),
        )
    )   

    # RViz Node
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Enable GUI'
    )

    use_gui_param = DeclareLaunchArgument(
        'use_gui',
        default_value=LaunchConfiguration('gui'),
        description='Use GUI'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('realsense2_description'), 'rviz', 'urdf.rviz')],
        parameters=[{'use_sim_time': False}]
    )  

    return LaunchDescription([
        gui_arg,
        use_gui_param,
        start_world,
        spawn_robot_world,
        rviz_node
    ])