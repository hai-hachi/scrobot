from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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
        DeclareLaunchArgument(
            'config',
            default_value='/dev/null',
            description=(
                'Optional YAML config path. Leave default to use the package '
                'config/shuttle_spawn.yaml through the direct ros2-run interface.'
            ),
        ),
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
