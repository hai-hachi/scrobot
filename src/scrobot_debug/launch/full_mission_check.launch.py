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
    publish_debug_image = LaunchConfiguration('publish_debug_image')
    yolo_max_detection_rate = LaunchConfiguration('yolo_max_detection_rate')
    gz_headless = LaunchConfiguration('gz_headless')
    gz_verbosity = LaunchConfiguration('gz_verbosity')

    shuttle_mode = LaunchConfiguration('shuttle_mode')
    shuttle_count = LaunchConfiguration('shuttle_count')
    shuttle_seed = LaunchConfiguration('shuttle_seed')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_telemetry = LaunchConfiguration('launch_telemetry')
    launch_monitoring = LaunchConfiguration('launch_monitoring')
    monitor_foxglove = LaunchConfiguration('monitor_foxglove')
    monitor_recording = LaunchConfiguration('monitor_recording')
    monitor_auto_snapshot = LaunchConfiguration('monitor_auto_snapshot')
    return_to_idle = LaunchConfiguration('return_to_idle')
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
            'gz_headless': gz_headless,
            'gz_verbosity': gz_verbosity,
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
            'publish_debug_image': publish_debug_image,
            'max_detection_rate': yolo_max_detection_rate,
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
        {
            'use_sim_time': use_sim_time,
            'return_to_idle': return_to_idle,
            'idle_x': robot_x,
            'idle_y': robot_y,
            'idle_yaw': robot_yaw,
        },
    )

    telemetry = include(
        'scrobot_debug',
        'telemetry.launch.py',
        {'use_sim_time': use_sim_time},
        condition=IfCondition(launch_telemetry),
    )

    monitoring = include(
        'scrobot_diagnostics',
        'monitor.launch.py',
        {
            'use_sim_time': use_sim_time,
            'expect_mission': 'true',
            'expect_perception': 'true',
            'expect_localization': 'true',
            'expect_scan': 'true',
            'enable_foxglove': monitor_foxglove,
            'record_incidents': monitor_recording,
            'auto_snapshot': monitor_auto_snapshot,
        },
        condition=IfCondition(launch_monitoring),
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
            'gz_headless',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Run Gazebo server-only (-s); RGB/depth sensors still render. '
                'Set false to show the Gazebo GUI.'
            ),
        ),
        DeclareLaunchArgument(
            'gz_verbosity',
            default_value='2',
            description='Warnings/errors without high-volume Gazebo info logs.',
        ),
        DeclareLaunchArgument(
            'yolo_max_detection_rate',
            default_value='15.0',
            description='YOLO limit [Hz]. Use 15 for the validated benchmark.',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Publish annotated YOLO RGB to '
                '/perception/shuttle_debug/image for RQt/presentation capture.'
            ),
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

        # Preserve the validated regression start by default. For presentation
        # recording, override these with the designated top-right idle pose
        # (8.19, 1.70, 3.14159) and set return_to_idle:=true.
        DeclareLaunchArgument('robot_x', default_value='2.0'),
        DeclareLaunchArgument('robot_y', default_value='3.05'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='3.14159'),

        DeclareLaunchArgument(
            'launch_telemetry',
            default_value='false',
            description='Keep legacy one-line terminal telemetry (can disable when using UI).',
        ),
        DeclareLaunchArgument(
            'launch_monitoring',
            default_value='true',
            description='Start the read-only SCROBOT health monitor.',
        ),
        DeclareLaunchArgument(
            'monitor_foxglove',
            default_value='false',
            description='Start the optional Foxglove WebSocket bridge at port 8765.',
        ),
        DeclareLaunchArgument(
            'monitor_recording',
            default_value='false',
            description='Enable optional memory-bounded rosbag2 MCAP incident recorder.',
        ),
        DeclareLaunchArgument(
            'monitor_auto_snapshot',
            default_value='false',
            description='Ask incident recorder to save snapshots on persistent critical faults.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Enable the master RViz view with court, sweep path, '
                'mission goals, costmaps, and evaluation trajectories.'
            ),
        ),
        DeclareLaunchArgument(
            'return_to_idle',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'After the sweep, use Nav2 to return to the original spawn/idle '
                'pose before publishing COMPLETE.'
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
                monitoring,
            ],
        ),
        TimerAction(period=shuttle_delay, actions=[shuttles]),
        TimerAction(period=evaluator_delay, actions=[evaluation]),
        TimerAction(period=rviz_delay, actions=[rviz]),
        TimerAction(period=mission_delay, actions=[mission]),
    ])
