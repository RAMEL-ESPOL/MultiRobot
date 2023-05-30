import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    # Get the path to the turtlebot3_gazebo package
    turtlebot3_gazebo_path = os.path.join(os.getcwd(), 'install', 'turtlebot3_gazebo','share','turtlebot3_gazebo')

    # Launch turtlebot3_gazebo empty_world.launch.py for the first TurtleBot3
    turtlebot3_1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_path, 'launch', 'empty_world.launch.py')),
    )

     # Launch turtlebot3_gazebo spawn.launch.py for the second TurtleBot3
    turtlebot3_2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_path, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'model': 'turtlebot3_burger',
            'x_pose': '2.0',
            'y_pose': '0.0',
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )

    return LaunchDescription([
        turtlebot3_1_launch,
        DeclareLaunchArgument('robot_name', default_value='tb3_2', description='Name of the second TurtleBot3'),
    ])
