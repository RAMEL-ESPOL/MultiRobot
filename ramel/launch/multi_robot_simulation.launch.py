#!/usr/bin/env python3
#
# ... (License and author information)

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node

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

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    pkg_create3_gazebo_bringup = get_package_share_directory('irobot_create_gazebo_bringup')
    gazebo_params_yaml_file = os.path.join(pkg_create3_gazebo_bringup, 'config', 'gazebo_params.yaml')
    pkg_irobot_create_description = get_package_share_directory('irobot_create_description')
    # Set ignition resource path
    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(pkg_irobot_create_description).
                                                    parent.resolve())])
    
    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri = SetEnvironmentVariable(name='GAZEBO_MODEL_URI', value=[''])
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world,
             'extra-gazebo-args', '--ros-args', '--params-file', gazebo_params_yaml_file],
        output='screen',
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    # Gazebo
    gazebo_launch = PathJoinSubstitution(
        [pkg_create3_gazebo_bringup, 'launch', 'gazebo.launch.py'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world_path', world),
            ('use_gazebo_gui', LaunchConfiguration('use_gazebo_gui'))
        ]
    )
    # Get the model and urdf file
    model_folder = 'turtlebot3_burger'
    model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
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
        '-entity', 'tb3',
        '-file', model_path,
        '-x', '-0.5',
        '-y', '-0.5',
        '-z', '0.01',
         '-robot_namespace', 'tb3'  # Set the robot namespace
    ],
    output='screen',
    )
    start_gazebo_ros_spawner_cmd_2 = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'tb3_2',
        '-file', model_path,
        '-x', '0.5',
        '-y', '0.5',
        '-z', '0.01',
         '-robot_namespace', 'tb3_2'  # Set the robot namespace
    ],
    output='screen',
    )

    # Create3 robot spawn command
    create3_spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch', 'create3_spawn.launch.py')
        ),
        launch_arguments=[
            ('namespace', ''),
            ('use_rviz', 'true'),
            ('x', '0.0'),
            ('y', '0.0'),
            ('z', '0.01'),
            ('yaw', '0.0')
        ],
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_cmd_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='tb3',  # Set the namespace for the first robot state publisher
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
    )
    robot_state_publisher_cmd_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='tb3_2',  # Set the namespace for the second robot state publisher
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }],
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gz_model_uri)
    ld.add_action(gzserver)
    ld.add_action(gzclient_cmd)
    #ld.add_action(gazebo)
    ld.add_action(create3_spawn_cmd)
    ld.add_action(robot_state_publisher_cmd_1)
    ld.add_action(robot_state_publisher_cmd_2)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd_2)
    
    

    return ld
