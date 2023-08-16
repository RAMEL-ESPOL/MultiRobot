import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'

def generate_launch_description():
    num_cameras_arg = DeclareLaunchArgument(
        'number',
        default_value='3',
        description='Number of cameras'
    )
    # Create a LaunchDescription instance
    ld = LaunchDescription()

    # Specify the path to the first launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mrs_master'), 'launch', 'gazebo.launch.py'))
    )

    # Specify the path to the second launch file
    multi_robot = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mrs_master'), 'launch', 'multi_create.launch.py'))
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mrs_master'), 'launch', 'multi_robot_simulation.launch.py'))
    )

    # Specify the path to the third launch file
    multi_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mrs_master'), 'launch', 'multi_cam_simulation.launch.py')), launch_arguments={'number': LaunchConfiguration('number'),}.items()
    )
    # Add a delay of 5 seconds
    multi_robot_delay = TimerAction(
        period=5.0,
        actions=[multi_robot],
    )

    # Add the launch actions to the LaunchDescription
    ld.add_action(num_cameras_arg)
    ld.add_action(gazebo)
    ld.add_action(multi_cam)
    ld.add_action(multi_robot_delay)

    return ld

