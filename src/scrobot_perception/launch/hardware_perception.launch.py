import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_pkg = get_package_share_directory('scrobot_perception')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_apriltag = LaunchConfiguration('launch_apriltag')
    launch_depth_scan = LaunchConfiguration('launch_depth_scan')

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[
            os.path.join(perception_pkg, 'config', 'perception.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
            ('detections', '/apriltag/detections'),
        ],
        condition=IfCondition(launch_apriltag),
    )

    depth_scan = Node(
        package='scrobot_perception',
        executable='depth_obstacle_scan',
        name='depth_obstacle_scan',
        output='screen',
        parameters=[
            os.path.join(perception_pkg, 'config', 'depth_obstacle_scan.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/camera/camera/depth/scan'),
        ],
        condition=IfCondition(launch_depth_scan),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'launch_apriltag',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'launch_depth_scan',
            default_value='true',
            choices=['true', 'false'],
        ),
        apriltag,
        depth_scan,
    ])
