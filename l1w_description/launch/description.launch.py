from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Run robot_state_publisher with the L1W xacro (namespace comes from the parent launch)."""
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('model'), ' ',
                 LaunchConfiguration('xacro_args')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution(
                [FindPackageShare('l1w_description'), 'urdf', 'l1w.urdf.xacro'])),
        DeclareLaunchArgument('xacro_args', default_value='', description='e.g. use_sensors:=false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
