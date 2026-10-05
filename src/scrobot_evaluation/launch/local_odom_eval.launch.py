import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('scrobot_evaluation')
    params_file = os.path.join(package_dir, 'config', 'local_odom_eval.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_test = LaunchConfiguration('run_test')
    test_type = LaunchConfiguration('test_type')
    run_name = LaunchConfiguration('run_name')
    output_root = LaunchConfiguration('output_root')

    logger = Node(
        package='scrobot_evaluation',
        executable='local_odom_logger',
        name='local_odom_logger',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'run_name': run_name,
                'output_root': output_root,
            },
        ],
    )

    runner = Node(
        package='scrobot_evaluation',
        executable='local_odom_test_runner',
        name='local_odom_test_runner',
        output='screen',
        condition=IfCondition(run_test),
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'test_type': test_type,
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
            'run_test',
            default_value='true',
            choices=['true', 'false'],
            description='Launch the automatic motion test runner.',
        ),
        DeclareLaunchArgument(
            'test_type',
            default_value='suite',
            choices=['static', 'straight', 'rotate', 'arc', 'suite'],
        ),
        DeclareLaunchArgument(
            'run_name',
            default_value='',
            description='Optional output directory name.',
        ),
        DeclareLaunchArgument(
            'output_root',
            default_value=os.path.expanduser(
                '~/scrobot_ws/evaluation_results/local_odom'
            ),
            description='Root directory for local-odometry evaluation runs.',
        ),
        logger,
        runner,
    ])
