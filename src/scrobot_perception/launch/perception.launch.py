import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_pkg = get_package_share_directory('scrobot_perception')
    params = os.path.join(perception_pkg, 'config', 'perception.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    min_height = LaunchConfiguration('min_height')
    max_height = LaunchConfiguration('max_height')

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
            ('detections', '/apriltag/detections'),
        ],
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='depth_pointcloud_to_scan',
        output='screen',
        parameters=[
            params,
            {
                'use_sim_time': use_sim_time,
                'min_height': min_height,
                'max_height': max_height,
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
        DeclareLaunchArgument(
            'min_height',
            default_value='0.08',
            description='Minimum obstacle height in base_footprint [m].',
        ),
        DeclareLaunchArgument(
            'max_height',
            default_value='0.70',
            description='Maximum obstacle height in base_footprint [m].',
        ),
        LogInfo(msg=['Perception scan height band: ', min_height, ' to ', max_height, ' m']),
        apriltag,
        pointcloud_to_scan,
    ])
