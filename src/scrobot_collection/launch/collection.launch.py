import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('scrobot_collection'), 'config', 'local_collect.yaml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false']
        ),
        Node(
            package='scrobot_collection',
            executable='shuttle_collection_filter',
            name='shuttle_collection_filter',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='scrobot_collection',
            executable='local_collect_controller',
            name='local_collect_controller',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
    ])
