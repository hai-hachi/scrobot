import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_pkg = get_package_share_directory('scrobot_perception')
    params = os.path.join(
        perception_pkg,
        'config',
        'fake_shuttle_detector.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    fake_detector = Node(
        package='scrobot_perception',
        executable='fake_shuttle_detector.py',
        name='fake_shuttle_detector',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo simulation time.',
        ),
        fake_detector,
    ])
