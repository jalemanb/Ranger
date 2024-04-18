from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # Package Name
    ranger_description_launch_package = get_package_share_directory("ranger_description")

    # RViZ Publisher GUI Flag
    default_rviz_config_path = os.path.join(ranger_description_launch_package, "rviz","display_gz.rviz")

    return LaunchDescription([

        # Use Sim Time Launch Argument
        DeclareLaunchArgument(name='use_sim_time', default_value=use_sim_time,
                                        description='Using Gazeo Simulation Time'),
        # Showing RViz
        DeclareLaunchArgument(name='use_rviz', default_value=use_rviz,
                                        description='Running RViz Visualizer'),
        # Get RViz2 Config File
        DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute Path to Rviz Config File'),
        # Loading Robot URDF
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ranger_description_launch_package,
                         "launch","load_description.launch.py")),
            launch_arguments={ 'use_sim_time': use_sim_time}.items()
            ),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            ),
    ])
