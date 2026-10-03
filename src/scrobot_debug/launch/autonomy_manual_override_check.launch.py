import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(pkg, launch_file, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg),
                'launch',
                launch_file,
            )
        ),
        launch_arguments=(args or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    spawn_shuttles = LaunchConfiguration('spawn_shuttles')
    enable_magnetometer = LaunchConfiguration('enable_magnetometer')
    shuttle_mode = LaunchConfiguration('shuttle_mode')
    shuttle_count = LaunchConfiguration('shuttle_count')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')

    stack_delay = LaunchConfiguration('stack_delay')
    mission_delay = LaunchConfiguration('mission_delay')
    shuttle_delay = LaunchConfiguration('shuttle_delay')

    simulation = include(
        'scrobot_simulation',
        'simulation.launch.py',
        {
            'use_sim_time': use_sim_time,
            'x': robot_x,
            'y': robot_y,
            'yaw': robot_yaw,
            'enable_magnetometer': enable_magnetometer,
        },
    )

    control = include(
        'scrobot_control',
        'control_stack.launch.py',
        {'use_sim_time': use_sim_time},
    )

    localization = include(
        'scrobot_localization',
        'localization.launch.py',
        {'use_sim_time': use_sim_time},
    )

    perception = include(
        'scrobot_perception',
        'perception.launch.py',
        {'use_sim_time': use_sim_time},
    )

    shuttle_perception = include(
        'scrobot_perception',
        'shuttle_perception_sim.launch.py',
        {'use_sim_time': use_sim_time},
    )

    mission = include(
        'scrobot_mission',
        'sweep_mission.launch.py',
        {'use_sim_time': use_sim_time},
    )

    telemetry = include(
        'scrobot_debug',
        'telemetry.launch.py',
        {'use_sim_time': use_sim_time},
    )

    rviz_debug = include(
        'scrobot_debug',
        'rviz.launch.py',
        {'use_sim_time': use_sim_time},
        condition=IfCondition(rviz),
    )

    shuttles = include(
        'scrobot_simulation',
        'spawn_shuttles.launch.py',
        {
            'mode': shuttle_mode,
            'count': shuttle_count,
        },
        condition=IfCondition(spawn_shuttles),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'enable_magnetometer',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Temporary compatibility switch: the current baseline '
                'localization config still expects magnetometer input.'
            ),
        ),
        DeclareLaunchArgument(
            'spawn_shuttles',
            default_value='false',
            choices=['true', 'false'],
            description='Spawn a shuttle distribution for collection testing.',
        ),
        DeclareLaunchArgument(
            'shuttle_mode',
            default_value='mixed',
            choices=['single', 'random', 'cluster', 'mixed'],
        ),
        DeclareLaunchArgument('shuttle_count', default_value='20'),

        DeclareLaunchArgument('robot_x', default_value='2.0'),
        DeclareLaunchArgument('robot_y', default_value='3.05'),
        DeclareLaunchArgument('robot_yaw', default_value='3.14159'),

        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('mission_delay', default_value='6.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='5.0'),

        simulation,
        rviz_debug,
        TimerAction(
            period=stack_delay,
            actions=[
                control,
                localization,
                perception,
                shuttle_perception,
                telemetry,
            ],
        ),
        TimerAction(
            period=shuttle_delay,
            actions=[shuttles],
        ),
        TimerAction(
            period=mission_delay,
            actions=[mission],
        ),
    ])
