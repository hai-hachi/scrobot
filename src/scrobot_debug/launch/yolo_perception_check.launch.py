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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_pkg = get_package_share_directory('scrobot_simulation')
    perception_pkg = get_package_share_directory('scrobot_perception')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    publish_debug_image = LaunchConfiguration('publish_debug_image')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    shuttle_x = LaunchConfiguration('shuttle_x')
    shuttle_y = LaunchConfiguration('shuttle_y')
    shuttle_delay = LaunchConfiguration('shuttle_delay')
    detector_delay = LaunchConfiguration('detector_delay')
    monitor_delay = LaunchConfiguration('monitor_delay')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_pkg,
                'launch',
                'simulation.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_name': 'badminton_court',
            'x': robot_x,
            'y': robot_y,
            'z': robot_z,
            'yaw': robot_yaw,
            'enable_magnetometer': 'false',
        }.items(),
    )

    spawn_shuttle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_pkg,
                'launch',
                'spawn_shuttles.launch.py',
            )
        ),
        launch_arguments={
            'mode': 'single',
            'batch': 'yolo_check',
            'x': shuttle_x,
            'y': shuttle_y,
        }.items(),
    )

    detector_params = os.path.join(
        perception_pkg,
        'config',
        'yolo_shuttle_detector.yaml',
    )

    detector = Node(
        package='scrobot_perception',
        executable='yolo_shuttle_detector',
        name='yolo_shuttle_detector',
        output='screen',
        emulate_tty=True,
        parameters=[
            detector_params,
            {
                'use_sim_time': use_sim_time,
                'model_path': model_path,
                'device': device,
                'publish_debug_image': publish_debug_image,
            },
        ],
    )

    detector_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=detector,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='YOLO shuttle detector exited; stopping debug test.'
                    )
                )
            ],
        )
    )

    monitor = Node(
        package='scrobot_debug',
        executable='yolo_perception_monitor',
        name='yolo_perception_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'report_rate': 1.0,
            'stale_timeout': 2.0,
        }],
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
            description='Absolute path to gazebo_simple_v2.pt or deployed model.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='Ultralytics inference device.',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            choices=['true', 'false'],
        ),

        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),

        # Default target is directly ahead of a robot spawned at the origin.
        DeclareLaunchArgument('shuttle_x', default_value='1.00'),
        DeclareLaunchArgument('shuttle_y', default_value='0.00'),

        DeclareLaunchArgument(
            'shuttle_delay',
            default_value='4.0',
            description='Wait for Gazebo robot before spawning the shuttle.',
        ),
        DeclareLaunchArgument(
            'detector_delay',
            default_value='5.0',
            description='Wait for camera topics before starting YOLO.',
        ),
        DeclareLaunchArgument(
            'monitor_delay',
            default_value='6.0',
            description='Wait before starting terminal status monitor.',
        ),

        simulation,
        TimerAction(
            period=shuttle_delay,
            actions=[spawn_shuttle],
        ),
        TimerAction(
            period=detector_delay,
            actions=[detector],
        ),
        detector_exit,
        TimerAction(
            period=monitor_delay,
            actions=[monitor],
        ),
    ])
