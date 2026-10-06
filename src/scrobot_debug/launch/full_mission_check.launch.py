import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def include(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                'launch',
                launch_file,
            )
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')

    shuttle_mode = LaunchConfiguration('shuttle_mode')
    shuttle_count = LaunchConfiguration('shuttle_count')
    shuttle_seed = LaunchConfiguration('shuttle_seed')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    launch_rviz = LaunchConfiguration('launch_rviz')
    run_evaluation = LaunchConfiguration('run_evaluation')
    evaluation_run_name = LaunchConfiguration('evaluation_run_name')

    stack_delay = LaunchConfiguration('stack_delay')
    shuttle_delay = LaunchConfiguration('shuttle_delay')
    evaluator_delay = LaunchConfiguration('evaluator_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    mission_delay = LaunchConfiguration('mission_delay')

    simulation = include(
        'scrobot_simulation',
        'simulation.launch.py',
        {
            'use_sim_time': use_sim_time,
            'x': robot_x,
            'y': robot_y,
            'z': robot_z,
            'yaw': robot_yaw,
            'enable_magnetometer': 'false',
        },
    )

    control = include(
        'scrobot_control',
        'control_stack.launch.py',
        {'use_sim_time': use_sim_time},
    )

    local_localization = include(
        'scrobot_localization',
        'localization.launch.py',
        {
            'use_sim_time': use_sim_time,
            'use_magnetometer': 'false',
        },
    )

    perception = include(
        'scrobot_perception',
        'perception.launch.py',
        {
            'use_sim_time': use_sim_time,
            'enable_yolo': 'true',
            'model_path': model_path,
            'device': device,
            'publish_debug_image': 'false',
        },
    )

    shuttles = include(
        'scrobot_simulation',
        'spawn_shuttles.launch.py',
        {
            'mode': shuttle_mode,
            'count': shuttle_count,
            'seed': shuttle_seed,
            'batch': shuttle_seed,
        },
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

    evaluation = include(
        'scrobot_evaluation',
        'collection_session_eval.launch.py',
        {
            'use_sim_time': use_sim_time,
            'run_name': evaluation_run_name,
        },
        condition=IfCondition(run_evaluation),
    )

    rviz = include(
        'scrobot_debug',
        'rviz.launch.py',
        {'use_sim_time': use_sim_time},
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=EnvironmentVariable(
                'SCROBOT_YOLO_MODEL',
                default_value='',
            ),
            description='Production shuttle YOLO model.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='Ultralytics inference device.',
        ),

        DeclareLaunchArgument(
            'shuttle_mode',
            default_value='mixed',
            choices=['random', 'cluster', 'mixed'],
        ),
        DeclareLaunchArgument(
            'shuttle_count',
            default_value='50',
            description='Number of shuttles in the final mission layout.',
        ),
        DeclareLaunchArgument(
            'shuttle_seed',
            default_value='20261003',
            description='Repeatable shuttle-layout seed.',
        ),

        # Start near the positive-Y net post and face approximately toward the
        # net so the initial AprilTag acquisition starts from a known geometry.
        DeclareLaunchArgument('robot_x', default_value='2.0'),
        DeclareLaunchArgument('robot_y', default_value='3.05'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='3.14159'),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Enable the master RViz view with court, sweep path, '
                'mission goals, costmaps, and evaluation trajectories.'
            ),
        ),
        DeclareLaunchArgument(
            'run_evaluation',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'evaluation_run_name',
            default_value='',
            description='Empty uses a timestamped evaluator output folder.',
        ),

        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='5.0'),
        DeclareLaunchArgument('evaluator_delay', default_value='6.0'),
        DeclareLaunchArgument('rviz_delay', default_value='6.0'),
        DeclareLaunchArgument(
            'mission_delay',
            default_value='15.0',
            description=(
                'Delay so the 50-shuttle spawn, sensors, controllers, '
                'perception, and evaluator are ready before mission autostart.'
            ),
        ),

        simulation,
        TimerAction(
            period=stack_delay,
            actions=[
                control,
                local_localization,
                perception,
                telemetry,
            ],
        ),
        TimerAction(period=shuttle_delay, actions=[shuttles]),
        TimerAction(period=evaluator_delay, actions=[evaluation]),
        TimerAction(period=rviz_delay, actions=[rviz]),
        TimerAction(period=mission_delay, actions=[mission]),
    ])
