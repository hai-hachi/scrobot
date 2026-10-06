import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_pkg = get_package_share_directory('scrobot_perception')
    params = os.path.join(
        perception_pkg,
        'config',
        'yolo_shuttle_detector.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    publish_debug_image = LaunchConfiguration('publish_debug_image')

    detector = Node(
        package='scrobot_perception',
        executable='yolo_shuttle_detector',
        name='yolo_shuttle_detector',
        output='screen',
        parameters=[
            params,
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
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=EnvironmentVariable(
                'SCROBOT_YOLO_MODEL',
                default_value='',
            ),
            description=(
                'Path to shuttle YOLO .pt model. Can also be supplied through '
                'SCROBOT_YOLO_MODEL.'
            ),
        ),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='Ultralytics inference device, e.g. 0 or cpu.',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='false',
            choices=['true', 'false'],
        ),
        detector,
    ])
