from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the SDK driver and, optionally, robot_state_publisher under one namespace."""
    args = [
        DeclareLaunchArgument('namespace', default_value='l1w'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('l1w_driver'), 'config', 'l1w_driver.yaml'])),
        DeclareLaunchArgument('robot_ip', default_value='192.168.234.1'),
        DeclareLaunchArgument('use_description', default_value='true'),
    ]

    driver = Node(
        package='l1w_driver',
        executable='l1w_driver_node',
        name='l1w_driver_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'robot_ip': LaunchConfiguration('robot_ip')},
        ],
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('l1w_description'), 'launch', 'description.launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_description')),
    )

    return LaunchDescription(args + [
        GroupAction([PushRosNamespace(LaunchConfiguration('namespace')), driver, description]),
    ])
