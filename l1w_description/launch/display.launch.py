from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """View the URDF in RViz without the robot, using joint_state_publisher(_gui)."""
    pkg = FindPackageShare('l1w_description')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'model', default_value=PathJoinSubstitution([pkg, 'urdf', 'l1w.urdf.xacro'])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', 'description.launch.py'])),
            launch_arguments={
                'model': LaunchConfiguration('model'),
                'xacro_args': 'camera_stream_frames:=true',
            }.items(),
        ),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'l1w.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
