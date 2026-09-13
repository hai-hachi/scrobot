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
    count = LaunchConfiguration('count')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')

    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'scrobot_simulation', 'spawn_shuttles',
            '--config', config,
            '--mode', mode,
            '--visual', visual,
            '--batch', batch,
            '--count', count,
            '--x', x,
            '--y', y,
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
        DeclareLaunchArgument(
            'visual',
            default_value='detail',
            choices=['detail', 'fast'],
        ),
        DeclareLaunchArgument(
            'batch',
            default_value='',
            description=(
                'Optional entity-name batch suffix. Leave empty to auto-generate '
                'a unique batch so shuttles can be spawned repeatedly while Gazebo runs.'
            ),
        ),
        DeclareLaunchArgument(
            'count',
            default_value='20',
            description='Number of shuttles for random, cluster, or mixed modes.',
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Single-shuttle X position in the Gazebo world/map frame.',
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Single-shuttle Y position in the Gazebo world/map frame.',
        ),
        spawn,
    ])
