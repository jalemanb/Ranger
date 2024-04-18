#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ranger_gazebo_launch_package = get_package_share_directory('ranger_gazebo')
    ranger_description_launch_package = get_package_share_directory('ranger_description')

    robot_model = LaunchConfiguration('robot_model', default='ranger_mini_v2')
    x_pose = LaunchConfiguration('x_pose', default='0.1')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.35')
    R_ = LaunchConfiguration('R_', default='0.0')
    P_ = LaunchConfiguration('P_', default='0.0')
    Y_ = LaunchConfiguration('Y_', default='0.0')

    # Get the urdf file
    urdf_path = os.path.join(
        ranger_description_launch_package,
        'models/urdf',
        'ranger_mini.sdf'
    )
    
    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge_params = os.path.join(
        ranger_gazebo_launch_package,
        'config',
        'ros_gz_bridge.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(name='robot_model', default_value=robot_model, description='Robot Model Name'),
        DeclareLaunchArgument(name='x_pose', default_value=x_pose, description='Robot Initial Translation in X'),
        DeclareLaunchArgument(name='y_pose', default_value=y_pose, description='Robot Initial Translation in Y'),
        DeclareLaunchArgument(name='z_pose', default_value=z_pose, description='Robot Initial Translation in Z'),
        DeclareLaunchArgument(name='R_', default_value=R_, description='Robot Initial Roll'),
        DeclareLaunchArgument(name='P_', default_value=P_, description='Robot Initial Pitch'),
        DeclareLaunchArgument(name='Y_', default_value=Y_, description='Robot Initial Yaw'),

        # Set The Spawner Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_model,
                '-file', urdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-R', R_,
                '-P', P_,
                '-Y', Y_
            ],
            output='screen',
            ),

        # Start the ros_gz_bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ],
            output='screen',
            )
    ])