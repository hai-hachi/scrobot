import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    debug_share = get_package_share_directory('scrobot_debug')
    simulation_share = get_package_share_directory('scrobot_simulation')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(
        debug_share, 'worlds', 'shuttle_physics_test.sdf'
    )
    bridge_config = os.path.join(
        debug_share, 'config', 'shuttle_physics_bridge.yaml'
    )
    simulation_models = os.path.join(simulation_share, 'models')

    count = LaunchConfiguration('count')
    spacing = LaunchConfiguration('spacing')
    drop_height = LaunchConfiguration('drop_height')
    orientation = LaunchConfiguration('orientation')
    workers = LaunchConfiguration('workers')
    spawn_delay = LaunchConfiguration('spawn_delay')
    auto_spawn = LaunchConfiguration('auto_spawn')

    apply_impulse = LaunchConfiguration('apply_impulse')
    impulse_delay = LaunchConfiguration('impulse_delay')
    impulse_duration = LaunchConfiguration('impulse_duration')
    force_x = LaunchConfiguration('force_x')
    force_y = LaunchConfiguration('force_y')
    force_z = LaunchConfiguration('force_z')
    torque_x = LaunchConfiguration('torque_x')
    torque_y = LaunchConfiguration('torque_y')
    torque_z = LaunchConfiguration('torque_z')

    gz_verbosity = LaunchConfiguration('gz_verbosity')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v ', gz_verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='shuttle_physics_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    telemetry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(debug_share, 'launch', 'telemetry.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'summary_rate': '0.0',
            'min_rosout_level': '30',
        }.items(),
    )

    monitor = Node(
        package='scrobot_debug',
        executable='shuttle_physics_monitor',
        name='shuttle_physics_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'report_rate': 2.0,
            'linear_settle_threshold': 0.01,
            'angular_settle_threshold': 0.20,
            'settle_hold_time': 1.0,
        }],
    )

    spawner = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'scrobot_debug', 'spawn_shuttle_physics',
            '--count', count,
            '--spacing', spacing,
            '--drop-height', drop_height,
            '--orientation', orientation,
            '--workers', workers,
        ],
        output='screen',
    )

    impulse = Node(
        package='scrobot_debug',
        executable='shuttle_impulse_test',
        name='shuttle_impulse_test',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'delay': impulse_delay,
            'duration': impulse_duration,
            'force_x': force_x,
            'force_y': force_y,
            'force_z': force_z,
            'torque_x': torque_x,
            'torque_y': torque_y,
            'torque_z': torque_z,
        }],
        condition=IfCondition(apply_impulse),
    )

    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='1'),
        DeclareLaunchArgument('spacing', default_value='0.15'),
        DeclareLaunchArgument('drop_height', default_value='0.0'),
        DeclareLaunchArgument(
            'orientation',
            default_value='sideways',
            choices=['sideways', 'upright'],
        ),
        DeclareLaunchArgument('workers', default_value='8'),
        DeclareLaunchArgument('spawn_delay', default_value='5.0'),
        DeclareLaunchArgument(
            'auto_spawn',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Automatically spawn the configured shuttles after spawn_delay. '
                'Leave false for interactive spawn/delete/respawn testing.'
            ),
        ),

        DeclareLaunchArgument(
            'apply_impulse',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument('impulse_delay', default_value='5.0'),
        DeclareLaunchArgument('impulse_duration', default_value='0.05'),
        DeclareLaunchArgument('force_x', default_value='0.0'),
        DeclareLaunchArgument('force_y', default_value='0.03'),
        DeclareLaunchArgument('force_z', default_value='0.0'),
        DeclareLaunchArgument('torque_x', default_value='0.0'),
        DeclareLaunchArgument('torque_y', default_value='0.0'),
        DeclareLaunchArgument('torque_z', default_value='0.0'),

        DeclareLaunchArgument('gz_verbosity', default_value='2'),

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=simulation_models,
        ),

        gazebo,
        bridge,
        telemetry,
        monitor,
        TimerAction(
            period=spawn_delay,
            actions=[spawner],
            condition=IfCondition(auto_spawn),
        ),
        TimerAction(period=spawn_delay, actions=[impulse]),
    ])
