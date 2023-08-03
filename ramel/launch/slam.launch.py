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
        default_value='r2',
        description='Namespace for the robot'
    )
    mode = DeclareLaunchArgument(
        'mode',
        default_value='mappping',
        description='SLAM mode'
    )
    initial_pose = DeclareLaunchArgument(
        'initial_pose',
        default_value='[0.0, 0.0, 0.0]',
        description='Map pose'
    )
    # Create the launch description
    ld = LaunchDescription()
    # Add the namespace argument
    ld.add_action(namespace_arg)
    ld.add_action(mode)
    ld.add_action(initial_pose)
    map_file = os.path.join(get_package_share_directory('ramel'), 'map', 'ramel_map_serialized')
    
    # Get the string parameter passed from the main launch file
    ns = LaunchConfiguration('namespace')
    # Define the concatenated strings using the namespace variable
    base_frame = [LaunchConfiguration('namespace'), '/base_link']
    odom_frame = [LaunchConfiguration('namespace'), '/odom']
    if LaunchConfiguration('mode') == "mapping":
    	map_frame = [LaunchConfiguration('namespace'), '/map']
    	map_topic= ['/',LaunchConfiguration('namespace'), '/map']
    else:
    	map_frame = 'map'
    	map_topic = '/map'

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
            {'mode': LaunchConfiguration('mode')},
            #{'map_start_at_dock': True},
            #{'map_start_pose': LaunchConfiguration('initial_pose')},
            #{'map_file_name': map_file},
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('/map', map_topic),
            ('/scan',['/',LaunchConfiguration('namespace'), '/scan']),
            ('/initialpose', ['/',LaunchConfiguration('namespace'), '/initialpose']),
        ],
    ))

    return ld

