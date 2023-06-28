import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def evaluate_xacro(context, *args, **kwargs):
    aruco_dictionary = LaunchConfiguration('aruco_dictionary').perform(context)
    aruco_id = LaunchConfiguration('aruco_id').perform(context)
    aruco_description_name = LaunchConfiguration('aruco_description_name').perform(context)
    model = LaunchConfiguration('model').perform(context)

    models = os.path.join(get_package_share_directory('aruco_description'))
    """
    robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      namespace=LaunchConfiguration('aruco_description_name'),
      output='screen',
      parameters=[{
        'use_sim_time': False,
        'robot_description': xacro.process_file(model, mappings={'aruco_dictionary': aruco_dictionary,
                                                                'aruco_id': aruco_id,
                                                                'aruco_name': aruco_description_name}).toxml()
      }])
    """
    
    robot_desc = xacro.process_file(model, mappings={'aruco_dictionary': aruco_dictionary,
                                                    'aruco_id': aruco_id,
                                                    'aruco_name': aruco_description_name}).toxml()
    
    
    xml = robot_desc.replace('"', '\\"')
    xml = xml.replace('package://aruco_description', models)

    swpan_args = '{name: \"'+ aruco_description_name + '\", xml: \"'  +  xml + '\" }'
    spawn_entity = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args]
    )
    """
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "robot_description",
		            "-entity", "camera",
                    ],
        parameters=[],
        respawn=False,
        output='screen'
    )"""

    return [spawn_entity]

def generate_launch_description():

    xacro_path = os.path.join(get_package_share_directory('aruco_description'), 'urdf', 'general_aruco.urdf.xacro')

    aruco_dictionary_arg = DeclareLaunchArgument(
        'aruco_dictionary',
        default_value='DICT_4X4_50',
        description='Name of aruco dictionary'
    )

    aruco_id_arg = DeclareLaunchArgument(
        'aruco_id',
        default_value='0',
        description='id of the aruco code'
    )

    aruco_description_name_arg = DeclareLaunchArgument(
        'aruco_description_name',
        default_value='my_aruco_description',
        description='Name of the ros parameter storing the aruco descrip condition=IfCondition(gui),tion'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value= xacro_path,
        description='Path to URDF xacro file'
    )

    aruco_description_param = ExecuteProcess(
        cmd=[
            'ros2',
            'param',
            'set',
            LaunchConfiguration('aruco_description_name'),
            LaunchConfiguration('aruco_description_cmd')
        ],
        output='screen'
    )

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
        output='screen'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_description'), 'launch', 'gazebo.launch.py')
        ),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "robot_description",
		            "-entity", LaunchConfiguration("aruco_description_name"),
                    ],
        parameters=[],
        respawn=False,
        output='screen'
    )

    


    return LaunchDescription([
        aruco_dictionary_arg,
        aruco_id_arg,
        aruco_description_name_arg,
        model_arg,
        #aruco_description_param,
        #gzserver,
        #gzclient,
        #spawn_entity,
        gazebo_launch,
        OpaqueFunction(function=evaluate_xacro)
    ])
