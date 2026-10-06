import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _spawn_shuttles(context):
    config = LaunchConfiguration('config').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    batch = LaunchConfiguration('batch').perform(context).strip()
    count = LaunchConfiguration('count').perform(context).strip()
    seed = LaunchConfiguration('seed').perform(context).strip()
    x = LaunchConfiguration('x').perform(context).strip()
    y = LaunchConfiguration('y').perform(context).strip()

    cmd = [
        'ros2', 'run', 'scrobot_simulation', 'spawn_shuttles',
        '--config', config,
        '--mode', mode,
    ]

    # Empty launch arguments mean "use shuttle_spawn.yaml". Only explicit
    # overrides are forwarded to the spawner.
    if batch:
        cmd += ['--batch', batch]
    if count:
        cmd += ['--count', count]
    if seed:
        cmd += ['--seed', seed]
    if x:
        cmd += ['--x', x]
    if y:
        cmd += ['--y', y]

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    simulation_share = get_package_share_directory('scrobot_simulation')
    default_config = os.path.join(
        simulation_share,
        'config',
        'shuttle_spawn.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Shuttle distribution YAML.',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='single',
            choices=['single', 'random', 'cluster', 'mixed'],
        ),
        DeclareLaunchArgument(
            'batch',
            default_value='',
            description=(
                'Optional entity-name batch suffix. Empty lets the spawner '
                'choose a unique batch.'
            ),
        ),
        DeclareLaunchArgument(
            'count',
            default_value='',
            description=(
                'Optional shuttle-count override. Empty uses the selected '
                'mode count from shuttle_spawn.yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'seed',
            default_value='',
            description=(
                'Optional integer layout seed for random/cluster/mixed modes. '
                'Empty uses the YAML value or a new random seed.'
            ),
        ),
        DeclareLaunchArgument(
            'x',
            default_value='',
            description=(
                'Optional single-shuttle world X override. Empty uses YAML.'
            ),
        ),
        DeclareLaunchArgument(
            'y',
            default_value='',
            description=(
                'Optional single-shuttle world Y override. Empty uses YAML.'
            ),
        ),
        OpaqueFunction(function=_spawn_shuttles),
    ])
