import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('scrobot_evaluation')
    params_file = os.path.join(package_dir, 'config', 'patrol_eval.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_name = LaunchConfiguration('run_name')

    logger = Node(
        package='scrobot_evaluation',
        executable='patrol_trajectory_logger',
        name='patrol_trajectory_logger',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'run_name': run_name,
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
            'run_name',
            default_value='',
            description='Optional patrol evaluation run directory name.',
        ),
        logger,
    ])
