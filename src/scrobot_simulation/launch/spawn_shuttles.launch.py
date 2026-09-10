import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulation_share = get_package_share_directory('scrobot_simulation')
    default_config = os.path.join(simulation_share, 'config', 'shuttle_spawn.yaml')

    config = LaunchConfiguration('config')
    mode = LaunchConfiguration('mode')
    visual = LaunchConfiguration('visual')
    batch = LaunchConfiguration('batch')

    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'scrobot_simulation', 'spawn_shuttles',
            '--config', config,
            '--mode', mode,
            '--visual', visual,
            '--batch', batch,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument(
            'mode',
            default_value='single',
            choices=['single', 'random', 'cluster', 'mixed'],
        ),
        DeclareLaunchArgument('visual', default_value='detail', choices=['detail', 'fast']),
        DeclareLaunchArgument('batch', default_value='1'),
        spawn,
    ])
