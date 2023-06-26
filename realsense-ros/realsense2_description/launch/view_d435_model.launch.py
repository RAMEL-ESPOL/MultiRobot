import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf

def generate_launch_description():
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_d435_camera.urdf.xacro')
    
    robot_description_cmd = f"xacro --inorder '{xacro_path}' use_nominal_extrinsics:=true add_plug:=true use_mesh:=true"
    
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})
    
    robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_cmd,
        description='Robot description'
    )

    model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments=[urdf]
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
        model_node,
        rviz_node
    ])
