import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable, OpaqueFunction
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import xacro


def eval_xacro(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    add_plug = LaunchConfiguration('add_plug').perform(context)
    publish_pointcloud = LaunchConfiguration('publish_pointcloud').perform(context)
    use_nominal_extrinsics = LaunchConfiguration('use_nominal_extrinsics').perform(context)
    use_mesh = LaunchConfiguration('use_mesh').perform(context)
    model = LaunchConfiguration('model').perform(context)
    
    robot_desc = xacro.process_file(model, mappings={'name': name,
                                                    'namespace': namespace,
                                                    'add_plug': add_plug,
                                                    'publish_pointcloud': publish_pointcloud,
                                                    'use_nominal_extrinsics': use_nominal_extrinsics,
                                                    'use_mesh': use_mesh}).toxml()
    
    robot_desc_arg = DeclareLaunchArgument(
            'robot_desc',
            default_value=robot_desc,
            description='Xml of the model'
    )
    return [robot_desc_arg]

def robot_state_node(context, *args, **kwargs):
    
    name = LaunchConfiguration('name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    add_plug = LaunchConfiguration('add_plug').perform(context)
    publish_pointcloud = LaunchConfiguration('publish_pointcloud').perform(context)
    use_nominal_extrinsics = LaunchConfiguration('use_nominal_extrinsics').perform(context)
    use_mesh = LaunchConfiguration('use_mesh').perform(context)
    model = LaunchConfiguration('model').perform(context)
    x_pos = LaunchConfiguration('x').perform(context)
    y_pos = LaunchConfiguration('y').perform(context)
    z_pos = LaunchConfiguration('z').perform(context)
    
    robot_desc = xacro.process_file(model, mappings={'name': name,
                                                    'namespace': namespace,
                                                    'add_plug': add_plug,
                                                    'publish_pointcloud': publish_pointcloud,
                                                    'use_nominal_extrinsics': use_nominal_extrinsics,
                                                    'use_mesh': use_mesh,
                                                    'x_pos': x_pos,
                                                    'y_pos': y_pos,
                                                    'z_pos': z_pos
                                                    }).toxml()

    use_sim_time = eval(LaunchConfiguration('use_sim_time').perform(context))
    models = os.path.join(get_package_share_directory('realsense2_description'))
    xml = robot_desc.replace('package://realsense2_description', models)
    #xml = ""+xml
    xml2 = xml.replace('"', '\\"')
    #robot_desc_arg = OpaqueFunction(function=eval_xacro)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    spawn_cam = Node(
        package='realsense2_description',
        executable='l515_spawn.py',
        arguments=['-xml', xml,
                   '-n', name,
                   '-ns', namespace,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   ],
        output='screen'
    )
    #"""
    swpan_args = '{name: \"'+ name + '_l515\", xml: \"'  +  xml2 + '\" }'
    cam_spawner = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args]
    )
    #"""

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', name,
            '-topic', namespace+'/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen',
    )

    start_gazebo_ros_spawner = Node(
        package='realsense2_description',
        executable='l515_spawn.py',
        arguments=[
            '-entity', name,
            '-topic', '/'+namespace+'/robot_description',
            '-pck', 'realsense2_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen',
    )

    return [robot_state_publisher_node, start_gazebo_ros_spawner]

def spawn_cam(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    add_plug = LaunchConfiguration('add_plug').perform(context)
    publish_pointcloud = LaunchConfiguration('publish_pointcloud').perform(context)
    use_nominal_extrinsics = LaunchConfiguration('use_nominal_extrinsics').perform(context)
    use_mesh = LaunchConfiguration('use_mesh').perform(context)
    model = LaunchConfiguration('model').perform(context)
    
    robot_desc = xacro.process_file(model, mappings={'name': name,
                                                    'namespace': namespace,
                                                    'add_plug': add_plug,
                                                    'publish_pointcloud': publish_pointcloud,
                                                    'use_nominal_extrinsics': use_nominal_extrinsics,
                                                    'use_mesh': use_mesh}).toxml()

    #robot_desc_arg = OpaqueFunction(function=eval_xacro)
    #xml = LaunchConfiguration('robot_desc').perform(context)
    #xml = OpaqueFunction(function=eval_xacro).perform(context)

    models = os.path.join(get_package_share_directory('realsense2_description'))

    #xml = robot_desc.replace('"', '\\"')
    xml = robot_desc.replace('package://realsense2_description', models)

    spawn_cam = Node(package='realsense2_description', executable='l515_spawn.py', arguments=['-xml', robot_desc], output='screen')
    return [spawn_cam]

def generate_launch_description():

    pkg_install_path = get_package_share_directory('realsense2_description')

    gazebo_models_path = os.path.join(pkg_install_path, 'meshes')
    #os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    urdf = os.path.join(pkg_install_path, 'urdf/', 'cam_l515.urdf.xacro')
    
    xacro_path = os.path.join(pkg_install_path, 'urdf', 'cam_l515.urdf.xacro')
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Arguments
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='camera',
        description='Namespace and name for the camera'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
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
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='x-position for the camera'
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value= '0.0',
        description='y-position for the camera'
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='1.0',
        description='z-position for the camera'
    )

    #context = LaunchContext()
    """
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
    )"""
    """
    models = os.path.join(get_package_share_directory('realsense2_description'))
    robot_desc = OpaqueFunction(function=eval_xacro)

    xml = xml.replace('"', '\\"')
    xml = xml.replace('package://realsense2_description', models)

    spawn_cam = Node(package='realsense2_description', executable='l515_spawn.py', arguments=[xml], output='screen')
    """
    
    world_file_name = 'empty.world'
    world = os.path.join(get_package_share_directory('realsense2_description'), 'worlds', world_file_name)
    gui = LaunchConfiguration('gui', default='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    spawn_world = DeclareLaunchArgument(
          'world',
          default_value=[world, ''],
          description='SDF world file')

    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui),
    )
    
    return LaunchDescription([
        #SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        name_arg,
        namespace_arg,
        add_plug_arg,
        publish_pointcloud_arg,
        use_nominal_extrinsics_arg,
        use_mesh_arg,
        model_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        #spawn_world,
        #gazebo,
        #gzserver,
        #gzclient,
        #OpaqueFunction(function=eval_xacro),
        #spawn_cam,
        #OpaqueFunction(function=spawn_cam),
        #robot_desc_arg,
        #Node(package='realsense2_description', executable='l515_spawn.py', arguments=[xml], output='screen'),
        OpaqueFunction(function=robot_state_node)     
    ])