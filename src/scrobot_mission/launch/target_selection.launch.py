import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_pkg = get_package_share_directory('scrobot_mission')
    params = os.path.join(mission_pkg, 'config', 'target_selector.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    selector = Node(
        package='scrobot_mission',
        executable='target_selector',
        name='shuttle_target_selector',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        selector,
    ])
