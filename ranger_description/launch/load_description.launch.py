from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package Name
    ranger_description_launch_package = FindPackageShare('ranger_description')

    # URDF loading
    urdf_path = PathJoinSubstitution([ranger_description_launch_package,"models/urdf/ranger_mini.urdf"])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Robot State Publisher Node
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,'use_sim_time': use_sim_time}]
        ))

    return ld
