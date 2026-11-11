import os
import sys
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


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
    # Same installed simulation-only YAML read by simulation.launch.py.
    # This controls whether Collision Monitor goes straight to diff_drive,
    # or through the optional closed-loop wheel RPM response surrogate.
    calibration_path = (
        Path(get_package_share_directory('scrobot_simulation'))
        / 'config/mobility_calibration.yaml'
    )
    with calibration_path.open('r', encoding='utf-8') as stream:
        mobility = yaml.safe_load(stream)
    if not isinstance(mobility, dict) or mobility.get('schema_version') != 1:
        raise RuntimeError(f'Invalid Mobility Lab published config: {calibration_path}')
    sys.path.insert(0, str(Path(get_package_share_directory('scrobot_simulation')) / 'launch'))
    from odometry_rate_config import load_rates, prepare_configurations
    sim_pkg = Path(get_package_share_directory('scrobot_simulation'))
    rates = load_rates(sim_pkg / 'config/odometry_calibration.yaml')
    generated_odometry = prepare_configurations(
        rates,
        Path(get_package_share_directory('scrobot_control')) / 'config/controllers.yaml',
        Path(get_package_share_directory('scrobot_localization')) / 'config/ekf.yaml',
    )
    motor = mobility['motor']
    if motor['model'] not in ('baseline', 'first_order'):
        raise RuntimeError(f"Invalid simulation motor model: {motor['model']}")
    motor_model_enabled = motor['model'] == 'first_order'
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_magnetometer = LaunchConfiguration('use_magnetometer')

    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    publish_debug_image = LaunchConfiguration('publish_debug_image')

    shuttle_mode = LaunchConfiguration('shuttle_mode')
    shuttle_count = LaunchConfiguration('shuttle_count')
    shuttle_seed = LaunchConfiguration('shuttle_seed')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    return_to_idle = LaunchConfiguration('return_to_idle')

    stack_delay = LaunchConfiguration('stack_delay')
    shuttle_delay = LaunchConfiguration('shuttle_delay')
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
            'enable_magnetometer': use_magnetometer,
        },
    )

    control = include(
        'scrobot_control',
        'control_stack.launch.py',
        {
            'use_sim_time': use_sim_time,
            'collision_monitor_output': (
                '/cmd_vel_motor_target' if motor_model_enabled
                else '/diff_drive_controller/cmd_vel'
            ),
        },
    )

    # This path is simulation-only. It never appears in robot.launch.py.
    # The physical STM32 already includes its closed-loop motor PID and must
    # not have a second response filter applied.
    sim_motor_response = Node(
        package='scrobot_simulation',
        executable='sim_motor_response',
        name='sim_motor_response',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tau_left': float(motor['tau_left']),
            'tau_right': float(motor['tau_right']),
            'gain_left': float(motor['gain_left']),
            'gain_right': float(motor['gain_right']),
            'command_timeout': 0.4,
        }],
    ) if motor_model_enabled else None

    local_localization = include(
        'scrobot_localization',
        'localization.launch.py',
        {
            'use_sim_time': use_sim_time,
            'use_magnetometer': use_magnetometer,
            'ekf_params': generated_odometry['ekf_params'],
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'use_magnetometer',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Enable simulated /imu/mag and Madgwick magnetic fusion. '
                'Set false to reproduce the historical Oct 7 benchmark.'
            ),
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
            'return_to_idle',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'After the sweep, use Nav2 to return to the original spawn/idle '
                'pose before publishing COMPLETE.'
            ),
        ),
        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='5.0'),
        DeclareLaunchArgument(
            'mission_delay',
            default_value='15.0',
            description=(
                'Delay so the 50-shuttle spawn, sensors, controllers, '
                'perception, and evaluator are ready before mission autostart.'
            ),
        ),

        simulation,
        *([sim_motor_response] if sim_motor_response is not None else []),
        TimerAction(
            period=stack_delay,
            actions=[
                control,
                local_localization,
                perception,
            ],
        ),
        TimerAction(period=shuttle_delay, actions=[shuttles]),
        TimerAction(period=mission_delay, actions=[mission]),
    ])
