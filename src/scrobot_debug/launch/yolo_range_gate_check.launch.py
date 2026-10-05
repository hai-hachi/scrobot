import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                'launch',
                filename,
            )
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    mission_pkg = get_package_share_directory('scrobot_mission')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    shuttle_x = LaunchConfiguration('shuttle_x')
    shuttle_y = LaunchConfiguration('shuttle_y')

    perception_delay = LaunchConfiguration('perception_delay')
    shuttle_delay = LaunchConfiguration('shuttle_delay')
    filter_delay = LaunchConfiguration('filter_delay')
    monitor_delay = LaunchConfiguration('monitor_delay')

    local_params = os.path.join(
        mission_pkg,
        'config',
        'local_collect.yaml',
    )

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

    perception = include(
        'scrobot_perception',
        'perception.launch.py',
        {
            'use_sim_time': use_sim_time,
            'enable_yolo': 'true',
            'model_path': model_path,
            'device': device,
            'publish_debug_image': 'true',
        },
    )

    spawn_shuttle = include(
        'scrobot_simulation',
        'spawn_shuttles.launch.py',
        {
            'mode': 'single',
            'batch': 'yolo_range_gate',
            'x': shuttle_x,
            'y': shuttle_y,
        },
    )

    collection_filter = Node(
        package='scrobot_mission',
        executable='shuttle_collection_filter',
        name='shuttle_collection_filter',
        output='screen',
        parameters=[
            local_params,
            {
                'use_sim_time': use_sim_time,
                # For this isolated gate test, avoid requiring map->odom.
                # Pole exclusion is irrelevant because all test points are
                # near the robot and far from the net poles.
                'map_frame': 'base_link',
            },
        ],
    )

    monitor = Node(
        package='scrobot_debug',
        executable='yolo_range_gate_monitor',
        name='yolo_range_gate_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': 'base_link',
            'min_target_range': 0.50,
            'max_target_range': 1.80,
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
        ),
        DeclareLaunchArgument('device', default_value='0'),

        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),

        DeclareLaunchArgument(
            'shuttle_x',
            default_value='0.55',
            description='Single shuttle world X [m].',
        ),
        DeclareLaunchArgument(
            'shuttle_y',
            default_value='0.0',
            description='Single shuttle world Y [m].',
        ),

        DeclareLaunchArgument('perception_delay', default_value='3.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='4.0'),
        DeclareLaunchArgument('filter_delay', default_value='5.0'),
        DeclareLaunchArgument('monitor_delay', default_value='5.5'),

        simulation,
        TimerAction(period=perception_delay, actions=[perception]),
        TimerAction(period=shuttle_delay, actions=[spawn_shuttle]),
        TimerAction(period=filter_delay, actions=[collection_filter]),
        TimerAction(period=monitor_delay, actions=[monitor]),
    ])
