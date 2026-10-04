import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_manager_name = LaunchConfiguration('controller_manager_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'controller_manager_name',
            default_value='/controller_manager',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                controller_manager_name,
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager',
                controller_manager_name,
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
