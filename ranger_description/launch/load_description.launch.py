from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Package Name
    ranger_description_launch_package = get_package_share_directory('ranger_description')

    # URDF Path
    urdf_path = PathJoinSubstitution([ranger_description_launch_package,"models/urdf/ranger_mini.urdf"])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    return LaunchDescription([

        # Use Sim Time Launch Argument
        DeclareLaunchArgument(name='use_sim_time', default_value=use_sim_time,
                                        description='Using Gazeo Simulation Time'),
        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,'use_sim_time': use_sim_time}]
            )
    ])