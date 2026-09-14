import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('scrobot_evaluation')
    params = os.path.join(pkg, 'config', 'collection_session_eval.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_name = LaunchConfiguration('run_name')
    output_root = LaunchConfiguration('output_root')
    pole_exclusion_radius = LaunchConfiguration('pole_exclusion_radius')

    evaluator = Node(
        package='scrobot_evaluation',
        executable='collection_session_evaluator',
        name='collection_session_evaluator',
        output='screen',
        parameters=[
            params,
            {
                'use_sim_time': use_sim_time,
                'run_name': run_name,
                'output_root': output_root,
                'pole_exclusion_radius': pole_exclusion_radius,
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
            description='Optional run folder name. Empty = timestamp.',
        ),
        DeclareLaunchArgument(
            'output_root',
            default_value=os.path.expanduser(
                '~/scrobot_ws/evaluation_results/collection_session'
            ),
            description='Root directory for collection-session results.',
        ),
        DeclareLaunchArgument(
            'pole_exclusion_radius',
            default_value='0.60',
            description='Radius [m] around each net pole intentionally excluded from collection.',
        ),
        evaluator,
    ])
