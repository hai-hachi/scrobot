import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg, launch_file, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', launch_file)
        ),
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    output_dir = LaunchConfiguration('output_dir')
    session_name = LaunchConfiguration('session_name')
    target_images = LaunchConfiguration('target_images')
    positive_pose_fraction = LaunchConfiguration('positive_pose_fraction')
    settle_time = LaunchConfiguration('settle_time')
    random_seed = LaunchConfiguration('random_seed')

    shuttle_mode = LaunchConfiguration('shuttle_mode')
    shuttle_count = LaunchConfiguration('shuttle_count')
    shuttle_seed = LaunchConfiguration('shuttle_seed')

    spawn_delay = LaunchConfiguration('spawn_delay')
    capture_delay = LaunchConfiguration('capture_delay')

    simulation = include(
        'scrobot_simulation',
        'simulation.launch.py',
        {
            'use_sim_time': use_sim_time,
            'x': '2.0',
            'y': '0.0',
            'yaw': '0.0',
            'enable_magnetometer': 'false',
        },
    )

    shuttles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('scrobot_simulation'),
                'launch',
                'spawn_shuttles.launch.py',
            )
        ),
        launch_arguments={
            'mode': shuttle_mode,
            'count': shuttle_count,
        }.items(),
    )

    capture = Node(
        package='scrobot_debug',
        executable='yolo_dataset_capture',
        name='yolo_dataset_capture',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_dir': output_dir,
            'session_name': session_name,
            'target_images': target_images,
            'positive_pose_fraction': positive_pose_fraction,
            'settle_time': settle_time,
            'random_seed': random_seed,
        }],
    )

    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=capture,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='Synthetic YOLO dataset capture completed.'
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=os.path.expanduser(
                '~/Desktop/yoloshuttle/dataset/gazebo_scrobot'
            ),
            description='Destination YOLO dataset directory.',
        ),
        DeclareLaunchArgument(
            'session_name',
            default_value='',
            description='Optional filename/session prefix. Empty creates a timestamp.',
        ),
        DeclareLaunchArgument(
            'target_images',
            default_value='1200',
            description='Number of labeled RGB frames to generate before exiting.',
        ),
        DeclareLaunchArgument(
            'positive_pose_fraction',
            default_value='0.85',
            description=(
                'Fraction of randomized robot viewpoints biased toward a shuttle '
                'at the 0.50-1.68 m camera-relative focus range.'
            ),
        ),
        DeclareLaunchArgument(
            'settle_time',
            default_value='0.40',
            description='Simulation seconds to wait after each robot teleport.',
        ),
        DeclareLaunchArgument(
            'random_seed',
            default_value='42',
            description='Robot-viewpoint random seed.',
        ),
        DeclareLaunchArgument(
            'shuttle_mode',
            default_value='mixed',
            choices=['single', 'random', 'cluster', 'mixed'],
        ),
        DeclareLaunchArgument('shuttle_count', default_value='50'),
        DeclareLaunchArgument(
            'shuttle_seed',
            default_value='',
            description=(
                'Reserved for repeatable shuttle layouts; the current shuttle '
                'launcher auto-selects a seed when empty.'
            ),
        ),
        DeclareLaunchArgument('spawn_delay', default_value='4.0'),
        DeclareLaunchArgument('capture_delay', default_value='6.0'),

        simulation,
        TimerAction(period=spawn_delay, actions=[shuttles]),
        TimerAction(period=capture_delay, actions=[capture]),
        shutdown_when_done,
    ])
