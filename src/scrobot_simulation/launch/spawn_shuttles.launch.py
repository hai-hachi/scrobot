from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    world = LaunchConfiguration('world')
    name = LaunchConfiguration('name')
    prefix = LaunchConfiguration('prefix')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    count = LaunchConfiguration('count')
    seed = LaunchConfiguration('seed')
    density = LaunchConfiguration('density')
    court_length = LaunchConfiguration('court_length')
    court_width = LaunchConfiguration('court_width')
    margin = LaunchConfiguration('margin')
    spawn_height = LaunchConfiguration('spawn_height')
    spawn_delay = LaunchConfiguration('spawn_delay')

    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'scrobot_simulation', 'spawn_shuttles',
            '--mode', mode,
            '--world', world,
            '--name', name,
            '--prefix', prefix,
            '--x', x,
            '--y', y,
            '--z', z,
            '--roll', roll,
            '--pitch', pitch,
            '--yaw', yaw,
            '--count', count,
            '--seed', seed,
            '--density', density,
            '--court-length', court_length,
            '--court-width', court_width,
            '--margin', margin,
            '--spawn-height', spawn_height,
            '--spawn-delay', spawn_delay,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='single', choices=['single', 'random']),
        DeclareLaunchArgument('world', default_value='badminton_court'),
        DeclareLaunchArgument('name', default_value=''),
        DeclareLaunchArgument('prefix', default_value='shuttle'),

        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.08'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='1.57079632679'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        DeclareLaunchArgument('count', default_value='20'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument(
            'density',
            default_value='uniform',
            choices=['uniform', 'center', 'net'],
        ),
        DeclareLaunchArgument('court_length', default_value='13.40'),
        DeclareLaunchArgument('court_width', default_value='6.10'),
        DeclareLaunchArgument('margin', default_value='0.15'),
        DeclareLaunchArgument('spawn_height', default_value='0.08'),
        DeclareLaunchArgument('spawn_delay', default_value='0.05'),

        spawn,
    ])
