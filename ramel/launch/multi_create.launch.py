#!/usr/bin/env python3
#
# ... (License and author information)

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, EmitEvent, GroupAction, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
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

    # Get the model and urdf file
    model_folder = 'turtlebot3_burger'
    model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    model_path2 = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model2.sdf'
    )
    model_path3 = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model3.sdf'
    )
    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name
        )
    start_gazebo_ros_spawner_cmd = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'r2',
        '-file', model_path,
        '-x', '-2.5',
        '-y', '-1.0',
        '-z', '0.01',
        '-robot_namespace', 'r2'  # Set the robot namespace
    ],
    output='screen',
    )
    start_gazebo_ros_spawner_cmd_2 = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'r3',
        '-file', model_path2,
        '-x', '1',
        '-y', '1',
        '-z', '0.01',
         '-robot_namespace', 'r3'  # Set the robot namespace
    ],
    output='screen',
    )
    start_gazebo_ros_spawner_cmd_3 = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'r4',
        '-file', model_path3,
        '-x', '-2',
        '-y', '1.75',
        '-z', '0.01',
         '-robot_namespace', 'r4'  # Set the robot namespace
    ],
    output='screen',
    )

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
            ('z', '0.01'),
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
            ('z', '0.01'),
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
            ('z', '0.01'),
            ('yaw', '0.0'),
            ('id_marker', '12')
        ],
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
        
    robot_state_publisher_cmd_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='r2',  # Set the namespace for the first robot state publisher
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_prefix': 'r2/',
            'robot_description': robot_desc
        }],
    )
    robot_state_publisher_cmd_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='r3',  # Set the namespace for the second robot state publisher
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_prefix': 'r3/',
            'robot_description': robot_desc,
        }],
    )
    robot_state_publisher_cmd_3 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='r4',  # Set the namespace for the second robot state publisher
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_prefix': 'r4/',
            'robot_description': robot_desc,
        }],
    )
    # Add a delay of 5 seconds
    delay_create1 = TimerAction(
        period=0.0,
        actions=[create3_spawn_cmd],
    )
    # Add a delay of 5 seconds
    delay_create2 = TimerAction(
        period=5.0,
        actions=[create3_spawn_cmd_2],
    )
    delay_create3 = TimerAction(
        period=10.0,
        actions=[create3_spawn_cmd_3],
    )

    # Add the commands to the launch description
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    #ld.add_action(robot_state_publisher_cmd_1)
    #ld.add_action(robot_state_publisher_cmd_2)
    #ld.add_action(robot_state_publisher_cmd_3)
    #ld.add_action(start_gazebo_ros_spawner_cmd)
    #ld.add_action(start_gazebo_ros_spawner_cmd_2)
    #ld.add_action(start_gazebo_ros_spawner_cmd_3)
    #ld.add_action(create3_spawn_cmd)
    ld.add_action(delay_create1)
    ld.add_action(delay_create2)
    ld.add_action(delay_create3)
    return ld
