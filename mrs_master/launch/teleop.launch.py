#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions.group_action import GroupAction

def generate_launch_description():
    # Define the command to launch teleop_keyboard in a new terminal window
    teleop_keyboard_cmd = ExecuteProcess(
        cmd=['terminator', '--command', 'bash -c "source install/setup.bash && exec ros2 run turtlebot3_teleop teleop_keyboard"'],
        output='screen',
    )

    # Define the command to launch teleop_keyboard2 in a new terminal window
    teleop_keyboard2_cmd = ExecuteProcess(
    	cmd=['terminator', '--command', 'bash -c "source install/setup.bash && exec ros2 run turtlebot3_teleop teleop_keyboard2"'],
    	output='screen',
    )

# Define the command to launch teleop_twist_keyboard in a new terminal window
    teleop_twist_keyboard_cmd = ExecuteProcess(
    	cmd=['terminator', '--command', 'bash -c "source install/setup.bash && exec ros2 run teleop_twist_keyboard teleop_twist_keyboard"'],
    	output='screen',
    )

    # Create a GroupAction to include all the processes
    teleop_group = GroupAction(
        actions=[
            teleop_keyboard_cmd,
            teleop_keyboard2_cmd,
            teleop_twist_keyboard_cmd,
        ]
    )

    # Add the commands and exit handler to the launch description
    ld = LaunchDescription()
    ld.add_action(teleop_group)

    return ld

