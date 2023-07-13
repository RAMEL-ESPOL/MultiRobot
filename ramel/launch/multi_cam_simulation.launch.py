#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext

def cameras_generator(context, *args, **kwargs):
    
    number_of_cameras = LaunchConfiguration('number').perform(context)

    cameras = []

    for i in range(int(number_of_cameras)):
        cam_name_cam = "cam_c"+str(i)
        cam_name = "c"+str(i)
        x_pos = float(i)
        cameras.append({'name': cam_name, 'namespace': cam_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 2.00})
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
                                  #'robot_urdf': urdf,
                                  'x': TextSubstitution(text=str(camera['x_pose'])),
                                  'y': TextSubstitution(text=str(camera['y_pose'])),
                                  'z': TextSubstitution(text=str(camera['z_pose'])),
                                  'name': camera['name'],
                                  'namespace': camera['namespace'],
                                  }.items()))
        nodes_exec.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_aruco_node, 'launch',
                                                           'marker_publisher.launch.py')),
                launch_arguments={
                                  'topic_c': camera['namespace'],
                                  'markerSize': '0.1',
                                  #'ref_frame': '',
                                  }.items()))
        
        # Launch aruco_ros2 node
        """
        nodes_exec.append(ExecuteProcess(
                cmd=['ros2',
                     'launch',
                     'aruco_ros',
                     'marker_publisher.launch.xml',
                     'topic_c:='+camera['namespace'],
                     'markerSize:=0.1'
                     #'ref_frame:=',
                     ],
                output='screen'
        ))"""

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

    urdf = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'cam_l515.urdf.xacro')
    pkg_cam_description = get_package_share_directory('realsense2_description')
    pkg_aruco_node = get_package_share_directory('aruco_ros')
    assert os.path.exists(urdf), "camera.urdf doesnt exist in "+str(urdf)

    num_cameras_arg = DeclareLaunchArgument(
        'number',
        default_value='5',
        description='Number of cameras'
    )

    #world_file_name = 'floating_marker.world'
    world_file_name = 'empty.world'
    world = os.path.join(get_package_share_directory('realsense2_description'), 'worlds', world_file_name)
    gui = LaunchConfiguration('gui', default='true')

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

    #context = LaunchContext()
    # Names and poses of the robots
    #cameras = OpaqueFunction(function=cameras_generator) 
    #print(cameras)

    # We create the list of spawn robots commands
    #spawn_cameras_cmds = []

    # We create the list of spawn aruco nodes
    #spawn_aruco_nodes = []
    """
    print(str(cameras))
    for camera in cameras:
        print("#############"+str(camera))
        spawn_cameras_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_cam_description, 'launch',
                                                           'l515_spawn.launch.py')),
                launch_arguments={
                                  #'robot_urdf': urdf,
                                  'x': TextSubstitution(text=str(camera['x_pose'])),
                                  'y': TextSubstitution(text=str(camera['y_pose'])),
                                  'z': TextSubstitution(text=str(camera['z_pose'])),
                                  'name': camera['name'],
                                  'namespace': camera['namespace'],
                                  }.items()))

        # Launch aruco_ros2 node
        
        spawn_aruco_nodes.append(ExecuteProcess(
                cmd=['ros2',
                     'launch',
                     'aruco_ros2',
                     'marker_publisher.launch.xml',
                     'topic_c:='+camera['namespace'],
                     'markerSize:=0.1'
                     #'ref_frame:=',
                     ],
                output='screen'
        ))
    """

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
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(num_cameras_arg)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(gui_arg)
    ld.add_action(use_gui_param)
    ld.add_action(rviz_node)
    ld.add_action(OpaqueFunction(function=cameras_generator))
    """
    for spawn_robot_cmd in spawn_cameras_cmds:
        ld.add_action(spawn_robot_cmd)
    
    for spawn_aruco_node in spawn_aruco_nodes:
        ld.add_action(spawn_aruco_node)
    """
    return ld