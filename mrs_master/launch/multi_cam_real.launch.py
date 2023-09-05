#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def cameras_generator(context, *args, **kwargs):
    
    number_of_cameras = LaunchConfiguration('number').perform(context)

    cameras = []

    for i in range(int(number_of_cameras)):
        cam_name_cam = "cam_c"+str(i)
        cam_name = "c"+str(i)
        x_pos = float(i)
        cameras.append({'name': cam_name, 'namespace': cam_name, 'x_pose': x_pos*2-2.0, 'y_pose': 0.0, 'z_pose': 3.5})
    print(cameras)

    spawn_cameras_cmds = []
    spawn_aruco_nodes = []

    nodes_exec = []

    pkg_cam_description = get_package_share_directory('realsense2_description')
    pkg_aruco_node = get_package_share_directory('aruco_ros')


    for camera in cameras:
        print("#############"+str(camera))
        nodes_exec.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_cam_description, 'launch',
                                                           'l515_spawn.launch.py')),
                launch_arguments={
                                  'x': TextSubstitution(text=str(camera['x_pose'])),
                                  'y': TextSubstitution(text=str(camera['y_pose'])),
                                  'z': TextSubstitution(text=str(camera['z_pose'])),
                                  'name': camera['name'],
                                  'namespace': camera['namespace'],
                                  }.items()))
        # Launch aruco_ros node
        nodes_exec.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_aruco_node, 'launch',
                                                           'marker_publisher.launch.py')),
                launch_arguments={
                                  'topic_c': camera['namespace'],
                                  'markerSize': '0.1',
                                  'ref_frame': 'map',
                                  }.items()))
       
    return nodes_exec 
    

def gen_cam_list(number_of_camerass):

    cameras = []

    for i in range(number_of_camerass):
        cam_name_cam = "cam_c"+str(i)
        cam_name = "c"+str(i)
        x_pos = float(i)
        cameras.append({'name': cam_name, 'namespace': cam_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 2.00})


    return cameras 

def generate_launch_description():
    pkg_cam_description = get_package_share_directory('realsense2_description')
    pkg_aruco_node = get_package_share_directory('aruco_ros')

    num_cameras_arg = DeclareLaunchArgument(
        'number',
        default_value='3',
        description='Number of cameras'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('realsense2_description'), 'rviz', 'urdf.rviz')],
        parameters=[{'use_sim_time': False}]
    )
    
    # Launch realsense2_camera node
    realsense_camera_node1 = ExecuteProcess(
        cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 'rgb_camera.profile:=1920x1080x30', 'camera_name:=c0'],
        output='screen'
    )
    camera_link_1 = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '-0.10', '-1.65', '1.95', '-0.4296169', '0.2328929', '0.6644971', '0.5653617', 'map', 'c0_link'],
        output='screen'
    )
    aruco_detector = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_aruco_node, 'launch',
                                                           'marker_publisher.launch.py')),
                launch_arguments={
                                  'topic_c': 'c0',
                                  'markerSize': '0.1',
                                  'ref_frame': 'map',
                                  }.items())
    
    aruco_filter = Node(
        package='mrs_master',
        executable='aruco_pose_filter.py',
        name='aruco_filter',
        output='screen',
        arguments=['1']
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    #ld.add_action(num_cameras_arg)
    #ld.add_action(rviz_node)
    #ld.add_action(OpaqueFunction(function=cameras_generator))
    ld.add_action(realsense_camera_node1)
    ld.add_action(camera_link_1)
    ld.add_action(aruco_detector)
    ld.add_action(aruco_filter)
    """
    for spawn_robot_cmd in spawn_cameras_cmds:
        ld.add_action(spawn_robot_cmd)
    
    for spawn_aruco_node in spawn_aruco_nodes:
        ld.add_action(spawn_aruco_node)
    """
    return ld
