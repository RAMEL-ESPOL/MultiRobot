import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
import xacro

def evaluate_xacro(context, *args, **kwargs):
    name = LaunchConfiguration('name',default='camera').perform(context)
    topics_ns = LaunchConfiguration('topics_ns',default='camera').perform(context)
    add_plug = LaunchConfiguration('add_plug',default='true').perform(context)
    publish_pointcloud = LaunchConfiguration('publish_pointcloud',default='true').perform(context)
    use_nominal_extrinsics = LaunchConfiguration('use_nominal_extrinsics',default='true').perform(context)
    use_mesh = LaunchConfiguration('use_mesh', default='true').perform(context)
    model = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'cam_l515.urdf.xacro')

    models = os.path.join(get_package_share_directory('realsense2_description'))
    
    robot_desc = xacro.process_file(model, mappings={'name': name,
                                                    'namespace': topics_ns,
                                                    'add_plug': add_plug,
                                                    'publish_pointcloud': publish_pointcloud,
                                                    'use_nominal_extrinsics': use_nominal_extrinsics,
                                                    'use_mesh': use_mesh}).toxml()
    
    
    xml = robot_desc.replace('"', '\\"')
    xml = xml.replace('package://realsense2_description', models)

    swpan_args = '{name: \"'+ name + '_l515\", xml: \"'  +  xml + '\" }'
    urdf_spawner = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args]
    )
    return [urdf_spawner]

def generate_launch_description():
    # Arguments
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'cam_l515.urdf.xacro')
    paused = LaunchConfiguration('paused', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    debug = LaunchConfiguration('debug', default='false')
    model = LaunchConfiguration('model', default=xacro_path)
    name = LaunchConfiguration('name', default='camera')
    topics_ns = LaunchConfiguration('topics_ns', default='camera')
    add_plug = LaunchConfiguration('add_plug', default='false')
    publish_pointcloud = LaunchConfiguration('publish_pointcloud', default='$(arg publish_pointcloud)')

    #world_file_name = 'floating_marker.world' #'empty.world'
    world_file_name = 'empty.world'
    
    world = os.path.join(get_package_share_directory('realsense2_description'), 'worlds', world_file_name)
    
    
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

    """
    # Empty World Launch    
    gazebo_cmd =  ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
    )
        
 
    #gazebo_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    empty_world_launch =  ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'
    )"""
    
    """IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path),
        launch_arguments={
            'debug': debug,
            'gui': gui,
            'paused': paused,
            'use_sim_time': use_sim_time,
            'headless': headless
        }.items()
    )"""
    
    # URDF Spawner Node
    """urdf_spawner = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args],
            output='screen'
    )"""
    """
    urdf_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "robot_description",
		            "-entity", "camera",
                    ],
        respawn=False,
        output='screen'
    )"""
    
    return LaunchDescription([
    	#robot_description,
        #gz_model_uri,
        gzserver,
        gzclient,
        #gazebo_cmd,
    	#empty_world_launch,
    	#urdf_spawner
        OpaqueFunction(function=evaluate_xacro)
    ])
