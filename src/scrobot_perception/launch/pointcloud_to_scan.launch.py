import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('scrobot_perception')
    params_file = os.path.join(pkg, 'config', 'pointcloud_to_scan.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    min_height = LaunchConfiguration('min_height')
    max_height = LaunchConfiguration('max_height')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='depth_pointcloud_to_scan',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'min_height': ParameterValue(min_height, value_type=float),
                'max_height': ParameterValue(max_height, value_type=float),
                'range_min': ParameterValue(range_min, value_type=float),
                'range_max': ParameterValue(range_max, value_type=float),
            },
        ],
        remappings=[
            ('cloud_in', '/camera/camera/depth/points'),
            ('scan', '/camera/camera/depth/scan'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument('min_height', default_value='0.08'),
        DeclareLaunchArgument('max_height', default_value='0.60'),
        DeclareLaunchArgument('range_min', default_value='0.20'),
        DeclareLaunchArgument('range_max', default_value='3.00'),
        LogInfo(msg=['depth_pointcloud_to_scan min_height=', min_height]),
        LogInfo(msg=['depth_pointcloud_to_scan max_height=', max_height]),
        LogInfo(msg=['depth_pointcloud_to_scan range=[', range_min, ', ', range_max, ']']),
        pointcloud_to_scan,
    ])
