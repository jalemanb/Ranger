#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'ranger_gazebo'
    urdf_pkg_name='ranger_description'

    # Custom parameters
    robot_model = 'ranger_mini_v2' # Make Launch Argument
    world_name = 'empty_world'     # Make Launch Argument
    x_pose = '0.1'                 # Make Launch Argument
    y_pose = '0.0'                 # Make Launch Argument
    z_pose = '0.35'                # Make Launch Argument
    R_ = '0.0'                     # Make Launch Argument
    P_ = '0.0'                     # Make Launch Argument
    Y_ = '0.0'                     # Make Launch Argument

    # Get the urdf file
    urdf_path = os.path.join(
        get_package_share_directory(urdf_pkg_name),
        'models/urdf',
        'ranger_mini.sdf'
    )

    # Set The Spawner Robot
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_model,
            '-file', urdf_path,
            '-world', world_name,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', R_,
            '-P', P_,
            '-Y', Y_
        ],
        output='screen',
        )
    
    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge_params = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ros_gz_bridge.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(ros_gz_bridge)

    return ld