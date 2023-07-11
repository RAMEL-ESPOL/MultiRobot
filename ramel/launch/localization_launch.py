import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def localization_n_robots(context, *args, **kwargs):
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server']

    remappings = []
    
    robot_count= int(LaunchConfiguration('n').perform(context))
    # Create a list to hold the nodes
    node_list = []

    # Add map server node
    node_list.append(
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'topic_name': "/map"},
                        {'frame_id': "map"},
                        {'yaml_filename': map_yaml_file}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
        )
    )
    
    robot_count = 3  # Number of robots
    for i in range(robot_count):
        robot_namespace = f'r{i+1}'
        #tf_prefix = f'r{i}/'  # Set the tf_prefix for the robot
        params_file = os.path.join(get_package_share_directory('ramel'), 'config', 'nav2_multirobot_params.yaml')
        # Create our own temporary YAML files that include substitutions
        param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'base_frame_id':f'{robot_namespace}/base_link',
        'odom_frame_id':f'{robot_namespace}/odom'
    }

        configured_params_i = RewrittenYaml(
            source_file=params_file,
            root_key=robot_namespace,
            param_rewrites=param_substitutions,
            convert_types=True
        )

        lifecycle_nodes.append(f'{robot_namespace}/amcl')

        # Add amcl node for each robot
        node_list.append(
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace=robot_namespace,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params_i],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
            )
        )

    # Add lifecycle manager node
    node_list.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'bond_timeout': 0.0},
                        {'node_names': lifecycle_nodes}]
        )
    )

    # Create the GroupAction with the list of nodes
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=node_list
    )
    return [load_nodes]

def generate_launch_description():
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('ramel'), 'config', 'ramel_map.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('ramel'), 'config', 'nav2_multirobot_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='The name of the container that nodes will load in if use composition'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'
    )
    declare_num = DeclareLaunchArgument(
        'n', default_value='3',
        description='Number of navigation modules.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_num)

    # Add the actions to launch all of the localization nodes
    #ld.add_action(load_nodes)
    ld.add_action(OpaqueFunction(function=localization_n_robots))

    return ld

