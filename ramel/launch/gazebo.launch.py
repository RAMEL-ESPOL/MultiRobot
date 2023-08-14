#!/usr/bin/env python3
#
# ... (License and author information)

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.conditions import IfCondition

def generate_launch_description():
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(
        get_package_share_directory('ramel'),
        'world',
        'lab_ramel_2.world'
    )

    pkg_irobot_create_description = get_package_share_directory('irobot_create_description')
    gazebo_models_create_path = os.path.join(pkg_irobot_create_description, 'models')

    pkg_share = get_package_share_directory('ramel')
    gazebo_models_path = os.path.join(pkg_share, 'models:')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    #os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_create_path


        # Set ignition resource path
    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(pkg_irobot_create_description).parent.resolve())])



    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )


    # Add the commands to the launch description
    # Define LaunchDescription variable
    ld = LaunchDescription()
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient_cmd)
    return ld
