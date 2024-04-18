#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ranger_gazebo_launch_package = get_package_share_directory('ranger_gazebo')

    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_name = LaunchConfiguration('world_name', default='maze')

    return LaunchDescription([

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='Name of the world to be used (Excluding .sdf), it has to be located on the ranger_gazebo/worlds directory'),

        # GZ Server Node
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': ['-r -s -v3 ', os.path.join(ranger_gazebo_launch_package, 'worlds/'), world_name, '.sdf'], 'gz_version':'8','on_exit_shutdown': 'true'}.items()
            ),
            
        # GZ Client Node
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': '-g -v3 ', 'gz_version':'8'}.items()
            ),
    ])