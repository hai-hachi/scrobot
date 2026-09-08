import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('scrobot_evaluation')
    params_file = os.path.join(package_dir, 'config', 'evaluation.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    logger = Node(
        package='scrobot_evaluation',
        executable='evaluation_logger',
        name='evaluation_logger',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        logger,
    ])
