from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    topic = perform_substitutions(context, [LaunchConfiguration('topic_c')])

    aruco_marker_publisher_params = {
        'image_is_rectified': False,
        'marker_size': LaunchConfiguration('markerSize'),
        'reference_frame': LaunchConfiguration('ref_frame'),
        'camera_frame': topic + '_color_optical_frame',
        'topic': topic,
    }

    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        namespace = LaunchConfiguration('topic_c'),
        parameters=[aruco_marker_publisher_params],
        remappings=[('/camera_info', '/' + topic + '/color/camera_info'),
                    ('/image', '/' + topic + '/color/image_raw')],
        output="screen",
    )

    return [aruco_marker_publisher]


def generate_launch_description():

    topic_arg = DeclareLaunchArgument(
        'topic_c', default_value='camera',
        description='Topic of the camera'
    )

    marker_size_arg = DeclareLaunchArgument(
        'markerSize', default_value='0.05',
        description='Marker size in m. '
    )

    side_arg = DeclareLaunchArgument(
        'side', default_value='left',
        description='Side. ',
        choices=['left', 'right'],
    )

    reference_frame = DeclareLaunchArgument(
        'ref_frame', default_value='origin_frame',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(topic_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(side_arg)
    ld.add_action(reference_frame)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
