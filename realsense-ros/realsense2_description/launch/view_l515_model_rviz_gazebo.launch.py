import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf

def generate_launch_description():

    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_l515_camera.urdf.xacro')

    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})

    # Arguments
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='camera',
        description='Namespace and name for the camera'
    )
    topics_ns_arg = DeclareLaunchArgument(
        'topics_ns',
        default_value='camera',
        description='Namespace for the camera topics'
    )
    add_plug_arg = DeclareLaunchArgument(
        'add_plug',
        default_value='false',
        description='Flag to add plug to the camera'
    )
    publish_pointcloud_arg = DeclareLaunchArgument(
        'publish_pointcloud',
        default_value='true',
        description='Flag to publish point cloud'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value= xacro_path,
        description='Path to URDF xacro file'
    )

    # Robot Description
    robot_description_cmd = f"xacro --inorder '{xacro_path}' use_nominal_extrinsics:=true publish_pointcloud:=true add_plug:=false name:=camera topics_ns:=camera"
    
    robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_cmd,
        description='Robot description'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments=[urdf]
    )

    # Gazebo Launch
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_description'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'name': LaunchConfiguration('name'),
            'topics_ns': LaunchConfiguration('topics_ns'),
            'add_plug': LaunchConfiguration('add_plug'),
            'publish_pointcloud': LaunchConfiguration('publish_pointcloud')
        }.items()
    )

    # RViz Node
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
        name_arg,
        topics_ns_arg,
        add_plug_arg,
        publish_pointcloud_arg,
        model_arg,
        robot_description,
        robot_state_publisher_node,
        gazebo_launch,
        gui_arg,
        use_gui_param,
        rviz_node
    ])

