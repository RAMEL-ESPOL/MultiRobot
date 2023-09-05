import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def cameras_localization(context, *args, **kwargs):
    
    number_of_cameras = LaunchConfiguration('n_cams').perform(context)

    nodes_exec = []

    pkg_aruco_node = get_package_share_directory('aruco_ros')

    for i in range(int(number_of_cameras)):
        # Launch aruco_ros node
        nodes_exec.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_aruco_node, 'launch',
                                                           'marker_publisher.launch.py')),
                launch_arguments={
                                  'topic_c': "c"+str(i),
                                  'markerSize': '0.1',
                                  'ref_frame': 'map',
                                  }.items()))
       
    return nodes_exec 

def generate_launch_description():
    # Create a LaunchDescription instance
    ld = LaunchDescription()

    # Define the path to mrs_master package
    mrs_master_path = get_package_share_directory('mrs_master')

    # Define the launch arguments to pass
    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(mrs_master_path, 'map', 'map.yaml'),
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
    n_robots_arg = DeclareLaunchArgument(
        "n_robots",
        default_value="4",
        description="Number of robots"
    )
    n_cams_arg = DeclareLaunchArgument(
        "n_cams",
        default_value="7",
        description="Number of robots"
    )

    # Add the launch arguments to the LaunchDescription
    ld.add_action(map_file_arg)
    ld.add_action(use_namespace_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(n_robots_arg)
    ld.add_action(n_cams_arg)

    # Include the launch file using IncludeLaunchDescription
    mrs_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mrs_master_path, 'launch', 'bringup_launch.py')),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": LaunchConfiguration("use_composition"),
            "n": LaunchConfiguration("n_robots"),
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
        arguments=[LaunchConfiguration("n_cams")]
    )

    ld.add_action(mrs_bringup_launch)
    ld.add_action(rviz_node)
    ld.add_action(aruco_filter)
    ld.add_action(OpaqueFunction(function=cameras_localization))
    return ld

