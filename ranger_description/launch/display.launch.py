from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    # Package Name
    ranger_description_launch_package = get_package_share_directory('ranger_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    use_rviz = LaunchConfiguration('use_rviz', default='false')

    jsp_gui = LaunchConfiguration('jsp_gui', default='true')

    # RViZ Publisher GUI Flag
    default_rviz_config_path = os.path.join(ranger_description_launch_package, 'rviz', 'display.rviz')
 
    return LaunchDescription([
        # Joint State Publisher GUI Flag
        DeclareLaunchArgument(name='jsp_gui', default_value=jsp_gui, choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui'),

        DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'),
        # Loading Robot URDF
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ranger_description_launch_package,
                         "launch","load_description.launch.py")),
            launch_arguments={ 'use_sim_time': use_sim_time}.items()
            ),

        # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
        Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                condition=UnlessCondition(LaunchConfiguration('jsp_gui')),
                parameters=[{'use_sim_time': use_sim_time}],
            ),

        # Joint State Publisher GUI Node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('jsp_gui')),
            parameters=[{'use_sim_time': use_sim_time}],
            ),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}],
            )
    ])
