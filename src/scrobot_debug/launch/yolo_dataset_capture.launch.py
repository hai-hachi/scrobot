import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    capture_rate = LaunchConfiguration('capture_rate')
    shuttle_mode = LaunchConfiguration('shuttle_mode')
    shuttle_count = LaunchConfiguration('shuttle_count')
    spawn_delay = LaunchConfiguration('spawn_delay')
    stack_delay = LaunchConfiguration('stack_delay')
    capture_delay = LaunchConfiguration('capture_delay')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')

    simulation = include(
        'scrobot_simulation',
        'simulation.launch.py',
        {
            'use_sim_time': use_sim_time,
            'x': robot_x,
            'y': robot_y,
            'yaw': robot_yaw,
            'enable_magnetometer': 'false',
        },
    )

    # Local EKF provides odom -> base_footprint so the production control stack
    # can run without enabling the debug-only diff-drive odom TF.
    localization = include(
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
        {'use_sim_time': use_sim_time}.items(),
    )

    control = include(
        'scrobot_control',
        'control_stack.launch.py',
        {'use_sim_time': use_sim_time},
    )

    shuttles = include(
        'scrobot_simulation',
        'spawn_shuttles.launch.py',
        {
            'mode': shuttle_mode,
            'count': shuttle_count,
        },
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
            'capture_rate': capture_rate,
        }],
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
            'capture_rate',
            default_value='2.0',
            description='Saved RGB frames per second.',
        ),
        DeclareLaunchArgument(
            'shuttle_mode',
            default_value='mixed',
            choices=['single', 'random', 'cluster', 'mixed'],
        ),
        DeclareLaunchArgument('shuttle_count', default_value='40'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('spawn_delay', default_value='4.0'),
        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('capture_delay', default_value='6.0'),

        simulation,
        TimerAction(
            period=stack_delay,
            actions=[localization, perception, control],
        ),
        TimerAction(period=spawn_delay, actions=[shuttles]),
        TimerAction(period=capture_delay, actions=[capture]),
    ])
