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
    seed = LaunchConfiguration('seed')
    count = LaunchConfiguration('count')
    density = LaunchConfiguration('density')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'scrobot_simulation', 'spawn_shuttles',
            '--config', config,
            '--mode', mode,
            '--visual', visual,
            '--batch', batch,
            '--seed', seed,
            '--count', count,
            '--density', density,
            '--x', x,
            '--y', y,
            '--z', z,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('mode', default_value='single', choices=['single', 'random']),
        DeclareLaunchArgument('visual', default_value='detail', choices=['detail', 'fast']),
        DeclareLaunchArgument('batch', default_value='1'),
        DeclareLaunchArgument('seed', default_value='r'),
        DeclareLaunchArgument('count', default_value='20'),
        DeclareLaunchArgument('density', default_value='uniform', choices=['uniform', 'center', 'net']),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.08'),
        spawn,
    ])
