import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription instance
    ld = LaunchDescription()

    # Define the path to mrs_master package
    mrs_master_path = get_package_share_directory('mrs_master')

    # Define the launch arguments to pass
    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(mrs_master_path, 'map', 'ramel_map.yaml'),
        description="Path to the map file"
    )
    use_namespace_arg = DeclareLaunchArgument(
        "use_namespace",
        default_value="True",
        description="Whether to use namespace"
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Whether to autostart"
    )
    use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composition"
    )
    n_arg = DeclareLaunchArgument(
        "n",
        default_value="3",
        description="Number of robots"
    )

    # Add the launch arguments to the LaunchDescription
    ld.add_action(map_file_arg)
    ld.add_action(use_namespace_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(n_arg)

    # Include the launch file using IncludeLaunchDescription
    mrs_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mrs_master_path, 'launch', 'bringup_launch.py')),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": LaunchConfiguration("use_composition"),
            "n": LaunchConfiguration("n"),
        }.items()
    )
    # Create the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        #output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('mrs_master'),'rviz','nav2.rviz')]
    )
    # Create the Pose FIlter node
    aruco_filter = Node(
        package='mrs_master',
        executable='aruco_pose_filter.py',
        name='aruco_filter',
        output='screen',
        arguments=['3']
    )
    pose_publisher = Node(
        package='mrs_master',
        executable='initial_pose_publisher.py',
        name='pose_publisher',
        output='screen',
        arguments=['3']
    )

    pose_publisher_delay = TimerAction(
        period=20.0,
        actions=[pose_publisher],
    )

    ld.add_action(mrs_bringup_launch)
    ld.add_action(rviz_node)
    ld.add_action(aruco_filter)
    ld.add_action(pose_publisher_delay)
    return ld

