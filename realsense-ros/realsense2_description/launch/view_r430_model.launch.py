import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Robot Description
    robot_description_cmd = f"$(find xacro)/xacro --inorder '{get_package_share_directory('realsense2_description')}/urdf/test_r430_camera.urdf.xacro' use_nominal_extrinsics:=true"
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description_command': robot_description_cmd}]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', f'{get_package_share_directory('realsense2_description')}/rviz/urdf.rviz'],
        parameters=[{'use_gui': True}]
    )

    return LaunchDescription([
        robot_description,
        rviz_node
    ])

