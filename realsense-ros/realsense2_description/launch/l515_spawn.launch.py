import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import xacro

def robot_state_node(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    robot_desc = LaunchConfiguration('robot_desc').perform(context)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('topics_ns'),
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    return [robot_state_publisher_node]


def generate_launch_description():

    

    urdf = os.path.join(get_package_share_directory('box_bot_description'), 'robot/', 'box_bot.urdf')
    
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'cam_l515.urdf.xacro')
    
    assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

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
        default_value='true',
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
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
    )

    context = LaunchContext()

    name = LaunchConfiguration('name').perform(context)
    topics_ns = LaunchConfiguration('topics_ns').perform(context)
    add_plug = LaunchConfiguration('add_plug').perform(context)
    publish_pointcloud = LaunchConfiguration('publish_pointcloud').perform(context)
    use_nominal_extrinsics = LaunchConfiguration('use_nominal_extrinsics').perform(context)
    use_mesh = LaunchConfiguration('use_mesh').perform(context)
    model = LaunchConfiguration('model').perform(context)
    
    robot_desc = xacro.process_file(model, mappings={'name': name,
                                                    'topics_ns': topics_ns,
                                                    'add_plug': add_plug,
                                                    'publish_pointcloud': publish_pointcloud,
                                                    'use_nominal_extrinsics': use_nominal_extrinsics,
                                                    'use_mesh': use_mesh}).toxml()
    
    robot_desc_arg = DeclareLaunchArgument(
            'robot_desc',
            default_value=robot_desc,
            description='Xml of the model'
    )

    models = os.path.join(get_package_share_directory('realsense2_description'))
    xml = robot_desc.replace('"', '\\"')
    xml = xml.replace('package://realsense2_description', models)

    return LaunchDescription([
        name_arg,
        topics_ns_arg,
        add_plug_arg,
        publish_pointcloud_arg,
        use_nominal_extrinsics_arg,
        use_mesh_arg,
        model_arg,
        use_sim_time_arg,
        robot_desc_arg,
        Node(package='realsense2_description', executable='l515_spawn.py', arguments=[xml], output='screen'),
        OpaqueFunction(function=robot_state_node)
    ])