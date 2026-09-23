import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('scrobot_bringup')
    perception_pkg = get_package_share_directory('scrobot_perception')

    depth_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'depth_localization.launch.py')
        )
    )

    depth_scan = Node(
        package='scrobot_perception',
        executable='depth_obstacle_scan',
        name='depth_obstacle_scan',
        output='screen',
        parameters=[
            os.path.join(perception_pkg, 'config', 'depth_obstacle_scan.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/camera/camera/depth/scan'),
        ],
    )

    return LaunchDescription([
        depth_localization,
        depth_scan,
    ])
