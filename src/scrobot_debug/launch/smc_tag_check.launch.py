import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                'launch',
                launch_file,
            )
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_distance = LaunchConfiguration('target_distance')
    preferred_tag_id = LaunchConfiguration('preferred_tag_id')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')

    stack_delay = LaunchConfiguration('stack_delay')
    approach_delay = LaunchConfiguration('approach_delay')

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

    perception = include(
        'scrobot_perception',
        'perception.launch.py',
        {
            'use_sim_time': use_sim_time,
            'enable_yolo': 'false',
        },
    )

    localization = include(
        'scrobot_localization',
        'localization.launch.py',
        {
            'use_sim_time': use_sim_time,
            'use_magnetometer': 'false',
        },
    )

    control = include(
        'scrobot_control',
        'control_stack.launch.py',
        {'use_sim_time': use_sim_time},
    )

    global_localization = include(
        'scrobot_localization',
        'global_localization.launch.py',
        {'use_sim_time': use_sim_time},
    )

    telemetry = include(
        'scrobot_debug',
        'telemetry.launch.py',
        {
            'use_sim_time': use_sim_time,
            'summary_rate': '2.0',
            'min_rosout_level': '20',
        },
    )

    approach = ExecuteProcess(
        cmd=[
            'ros2', 'action', 'send_goal',
            '/approach_tag',
            'scrobot_interfaces/action/ApproachTag',
            [
                '{preferred_tag_id: ', preferred_tag_id,
                ', target_distance: ', target_distance,
                ', timeout_sec: 30.0}',
            ],
            '--feedback',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'target_distance',
            default_value='0.80',
            description='Desired planar color-camera standoff from tag [m].',
        ),
        DeclareLaunchArgument(
            'preferred_tag_id',
            default_value='0',
            description='Court AprilTag ID used for the isolated SMC test.',
        ),

        # Near tag 0, intentionally displaced from its desired pose so the
        # controller must correct both position and heading.
        DeclareLaunchArgument('robot_x', default_value='1.50'),
        DeclareLaunchArgument('robot_y', default_value='1.80'),
        DeclareLaunchArgument('robot_yaw', default_value='2.80'),

        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('approach_delay', default_value='7.0'),

        simulation,
        TimerAction(
            period=stack_delay,
            actions=[
                perception,
                localization,
                control,
                global_localization,
                telemetry,
            ],
        ),
        TimerAction(
            period=approach_delay,
            actions=[approach],
        ),
    ])
