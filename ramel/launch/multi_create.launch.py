#!/usr/bin/env python3
#
# ... (License and author information)

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, EmitEvent, GroupAction, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.actions import OpaqueFunction

ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_gazebo_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set "false" to run gazebo headless.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]
# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'

def generate_launch_description():
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_create3_gazebo_bringup = get_package_share_directory('irobot_create_gazebo_bringup')
    gazebo_params_yaml_file = os.path.join(pkg_create3_gazebo_bringup, 'config', 'gazebo_params.yaml')
    pkg_irobot_create_description = get_package_share_directory('irobot_create_description')


    # Create3 robot spawn command
    create3_spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch', 'create3_spawn.launch.py')
        ),
        launch_arguments=[
            ('namespace', 'r1'),
            ('spawn_dock', 'false'),
            ('use_rviz', 'false'),
            ('x', '0.0'),
            ('y', '0.0'),
            ('z', '0.05'),
            ('yaw', '0.0'),
            ('id_marker', '10')
        ],
    )


    # Create3 robot spawn command
    create3_spawn_cmd_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch', 'create3_spawn.launch.py')
        ),
        launch_arguments=[
            ('namespace', 'r2'),
            ('spawn_dock', 'false'),
            ('use_rviz', 'false'),
            ('x', '1.5'),
            ('y', '-0.5'),
            ('z', '0.05'),
            ('yaw', '0.0'),
            ('id_marker', '11')
        ],
    )

    # Create3 robot spawn command
    create3_spawn_cmd_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch', 'create3_spawn.launch.py')
        ),
        launch_arguments=[
            ('namespace', 'r3'),
            ('spawn_dock', 'false'),
            ('use_rviz', 'false'),
            ('x', '0.0'),
            ('y', '-0.5'),
            ('z', '0.05'),
            ('yaw', '0.0'),
            ('id_marker', '12')
        ],
    )
    # Create3 robot spawn command
    create3_spawn_cmd_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch', 'create3_spawn.launch.py')
        ),
        launch_arguments=[
            ('namespace', 'r4'),
            ('spawn_dock', 'false'),
            ('use_rviz', 'false'),
            ('x', '1.5'),
            ('y', '0.0'),
            ('z', '0.05'),
            ('yaw', '0.0'),
            ('id_marker', '13')
        ],
    )
    # Create3 robot spawn command
    create3_spawn_cmd_5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch', 'create3_spawn.launch.py')
        ),
        launch_arguments=[
            ('namespace', 'r5'),
            ('spawn_dock', 'false'),
            ('use_rviz', 'false'),
            ('x', '-1.5'),
            ('y', '-0.5'),
            ('z', '0.05'),
            ('yaw', '0.0'),
            ('id_marker', '14')
        ],
    )
    # Create3 robot spawn command
    create3_spawn_cmd_6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch', 'create3_spawn.launch.py')
        ),
        launch_arguments=[
            ('namespace', 'r6'),
            ('spawn_dock', 'false'),
            ('use_rviz', 'false'),
            ('x', '-1.5'),
            ('y', '0.0'),
            ('z', '0.05'),
            ('yaw', '0.0'),
            ('id_marker', '15')
        ],
    )
    
    # Add a delay of 15 seconds
    delay_create2 = TimerAction(
        period=10.0,
        actions=[create3_spawn_cmd_2],
    )
    delay_create3 = TimerAction(
        period=30.0,
        actions=[create3_spawn_cmd_3],
    )
    delay_create4 = TimerAction(
        period=50.0,
        actions=[create3_spawn_cmd_4],
    )
    delay_create5 = TimerAction(
        period=70.0,
        actions=[create3_spawn_cmd_5],
    )
    delay_create6 = TimerAction(
        period=90.0,
        actions=[create3_spawn_cmd_6],
    )

    # Add the commands to the launch description
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(create3_spawn_cmd)
    ld.add_action(delay_create2)
    ld.add_action(delay_create3)
    #ld.add_action(delay_create4)
    #ld.add_action(delay_create5)
    #ld.add_action(delay_create6)
    return ld
