import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Create a LaunchDescription instance
    ld = LaunchDescription()

    # Define the path to ramel package
    ramel_path = get_package_share_directory('ramel')

    # Define the launch arguments to pass
    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(ramel_path, 'config', 'ramel_lab_2.yaml'),
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
        PythonLaunchDescriptionSource(os.path.join(ramel_path, 'launch', 'bringup_launch.py')),
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
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('ramel'),'config','nav2.rviz')]
    )

    ld.add_action(mrs_bringup_launch)
    ld.add_action(rviz_node)
    return ld

