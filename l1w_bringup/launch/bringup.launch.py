from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Bring up the full L1W: SDK driver + description, Livox Mid-360 and RealSense D435i."""
    cfg = PathJoinSubstitution([FindPackageShare('l1w_bringup'), 'config'])

    args = [
        DeclareLaunchArgument('namespace', default_value='l1w'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.234.1'),
        DeclareLaunchArgument('use_base', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument(
            'lidar_config', default_value=PathJoinSubstitution([cfg, 'MID360_config.json'])),
        DeclareLaunchArgument(
            'camera_config', default_value=PathJoinSubstitution([cfg, 'realsense_params.yaml'])),
        DeclareLaunchArgument(
            'lidar_xfer_format', default_value='1',
            description='0 = PointCloud2, 1 = CustomMsg (FAST-LIO)'),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('l1w_driver'), 'launch', 'l1w_driver.launch.py'])),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_description': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_base')),
    )

    lidar = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': ParameterValue(LaunchConfiguration('lidar_xfer_format'), value_type=int),
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': LaunchConfiguration('lidar_config'),
            'cmdline_input_bd_code': 'livox0000000001',
        }],
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={'config_file': LaunchConfiguration('camera_config')}.items(),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    return LaunchDescription(args + [base, lidar, camera])
