import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf
import xacro

def evaluate_xacro(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    topics_ns = LaunchConfiguration('topics_ns').perform(context)
    add_plug = LaunchConfiguration('add_plug').perform(context)
    publish_pointcloud = LaunchConfiguration('publish_pointcloud').perform(context)
    use_nominal_extrinsics = LaunchConfiguration('use_nominal_extrinsics').perform(context)
    use_mesh = LaunchConfiguration('use_mesh').perform(context)
    model = LaunchConfiguration('model').perform(context)
  
    robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      #namespace=LaunchConfiguration('topics_ns'),
      output='screen',
      parameters=[{
        'use_sim_time': False,
        'robot_description': xacro.process_file(model, mappings={'name': name,
                                                                 'topics_ns': topics_ns,
                                                                 'add_plug': add_plug,
                                                                 'publish_pointcloud': publish_pointcloud,
                                                                 'use_nominal_extrinsics': use_nominal_extrinsics,
                                                                 'use_mesh': use_mesh}).toxml()
      }])

    return [robot_state_publisher_node]

def generate_launch_description():

    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'cam_l515.urdf.xacro')

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
    use_nominal_extrinsics_arg = DeclareLaunchArgument(
        'use_nominal_extrinsics',
        default_value='true',
        description='Flag to create camera'
    )
    use_mesh_arg = DeclareLaunchArgument(
        'use_mesh',
        default_value='true',
        description='Flag to use meshes files'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value= xacro_path,
        description='Path to URDF xacro file'
    )


    #name_var = str(LaunchConfiguration('name'))
    #name_str = LaunchContext.perform_substitution(LaunchConfiguration('name'))
    #topic_var = str(LaunchConfiguration('topics_ns'))
    #topic_str = LaunchContext.perform_substitution(LaunchConfiguration('topics_ns'))
    name_str = ['a',LaunchConfiguration('name')]
    topic_str = ['a',LaunchConfiguration('topics_ns')]

    #urdf = to_urdf(xacro_path, {'name':''.join(name_str), 'topics_ns':''.join(topic_str), 'use_nominal_extrinsics': 'true', 'add_plug': 'true'})
    # Robot Description
    #robot_description_cmd = f"xacro --inorder '{xacro_path}' use_nominal_extrinsics:=true publish_pointcloud:=true add_plug:=false name:=c1 topics_ns:=c1"
    
    """robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_cmd,
        description='Robot description'
    )"""

    """
    doc = xacro.process_file(xacro_path, 
        mappings={name, 
                    topics_ns, 
                    publish_pointcloud, 
                    use_nominal_extrinsics, 
                    add_plug,
                    use_mesh
                    })
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                node_executable='robot_state_publisher_node',
                                namespace=LaunchConfiguration('topics_ns'),
                                output='screen',
                                parameters=[params])
    """

        # Robot State Publisher
    """robot_state_publisher_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('topics_ns'),
        output='screen',
        #arguments=[urdf]
        parameters=[{
                "robot_description": LaunchConfiguration('robot_desc')}]
    )"""

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
        use_nominal_extrinsics_arg,
        use_mesh_arg,
        model_arg,
        #create_robot_description_arg,
        #robot_description,
        #robot_state_publisher_node,
        gazebo_launch,
        gui_arg,
        use_gui_param,
        rviz_node,
        OpaqueFunction(function=evaluate_xacro)
    ])
