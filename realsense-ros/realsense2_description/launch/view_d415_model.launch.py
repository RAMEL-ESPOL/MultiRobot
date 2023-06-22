import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_d415_camera.urdf.xacro')
    
    robot_description_cmd = f"xacro --inorder '{xacro_path}' use_nominal_extrinsics:=true add_plug:=true use_mesh:=true"
    robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_cmd,
        description='Robot description'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher'
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
        robot_description,
        robot_state_publisher_node,
        gui_arg,
        use_gui_param,
        rviz_node
    ])
