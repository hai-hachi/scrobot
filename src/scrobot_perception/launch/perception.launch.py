import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_pkg = get_package_share_directory('scrobot_perception')
    params = os.path.join(perception_pkg, 'config', 'perception.yaml')
    yolo_params = os.path.join(
        perception_pkg,
        'config',
        'yolo_shuttle_detector.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_yolo = LaunchConfiguration('enable_yolo')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    publish_debug_image = LaunchConfiguration('publish_debug_image')
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

    depth_scan_self_filter = Node(
        package='scrobot_perception',
        executable='depth_scan_self_filter',
        name='depth_scan_self_filter',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='depth_pointcloud_to_scan',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cloud_in', '/camera/camera/depth/points'),
            ('scan', '/camera/camera/depth/scan_raw'),
        ],
    )

    yolo_detector = Node(
        package='scrobot_perception',
        executable='yolo_shuttle_detector',
        name='yolo_shuttle_detector',
        output='screen',
        condition=IfCondition(enable_yolo),
        parameters=[
            yolo_params,
            {
                'use_sim_time': use_sim_time,
                'model_path': model_path,
                'device': device,
                'publish_debug_image': publish_debug_image,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'enable_yolo',
            default_value='false',
            choices=['true', 'false'],
            description='Start the production YOLO + aligned-depth detector.',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=EnvironmentVariable(
                'SCROBOT_YOLO_MODEL',
                default_value='',
            ),
            description='Path to the shuttle YOLO model.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='Ultralytics inference device.',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='false',
            choices=['true', 'false'],
        ),
        apriltag,
        depth_scan_self_filter,
        pointcloud_to_scan,
        yolo_detector,
    ])
