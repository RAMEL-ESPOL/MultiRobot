import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='$(find realsense2_description)/urdf/test_d435i_camera.urdf.xacro',
        description='Path to URDF xacro file'
    )

    robot_description_cmd = f"$(find xacro)/xacro --inorder {LaunchConfiguration('model')} use_nominal_extrinsics:=true publish_pointcloud:=$(arg publish_pointcloud) add_plug:=$(arg add_plug) name:=$(arg name) topics_ns:=$(arg topics_ns)"
    robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_cmd,
        description='Robot description'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'publish_frequency': 30.0}]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('realsense2_description'), 'launch', 'gazebo.launch')),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'name': LaunchConfiguration('name'),
            'topics_ns': LaunchConfiguration('topics_ns'),
            'add_plug': LaunchConfiguration('add_plug'),
            'publish_pointcloud': LaunchConfiguration('publish_pointcloud')
        }.items()
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Enable GUI'
    )

    use_gui_param = DeclareLaunchArgument(
        'use_gui',
        default_value=LaunchConfiguration('gui'),
        description='Use GUI'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('realsense2_description'), 'rviz', 'urdf.rviz')],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        model_arg,
        robot_description,
        robot_state_publisher_node,
        gazebo_launch,
        gui_arg,
        use_gui_param,
        rviz_node
    ])

