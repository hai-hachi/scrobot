"""Legacy full regression: standard sim runtime plus optional test tooling.

Every normal simulation node is started by scrobot_bringup/sim.launch.py.
This retains historical full_mission_check launch defaults and node identities.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def include(package, filename, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', filename)
        ),
        launch_arguments=(args or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    stack_delay = LaunchConfiguration('stack_delay')
    evaluator_delay = LaunchConfiguration('evaluator_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    run_evaluation = LaunchConfiguration('run_evaluation')
    launch_rviz = LaunchConfiguration('launch_rviz')

    sim_arguments = {
        key: LaunchConfiguration(key)
        for key in (
            'use_sim_time', 'use_magnetometer', 'model_path', 'device',
            'publish_debug_image', 'shuttle_mode', 'shuttle_count', 'shuttle_seed',
            'robot_x', 'robot_y', 'robot_z', 'robot_yaw',
            'return_to_idle', 'stack_delay', 'shuttle_delay', 'mission_delay',
        )
    }
    runtime = include('scrobot_bringup', 'sim.launch.py', sim_arguments)
    telemetry = include(
        'scrobot_debug', 'telemetry.launch.py',
        {'use_sim_time': use_sim_time}
    )
    evaluation = include(
        'scrobot_evaluation', 'collection_session_eval.launch.py',
        {
            'use_sim_time': use_sim_time,
            'run_name': LaunchConfiguration('evaluation_run_name'),
        },
        condition=IfCondition(run_evaluation),
    )
    rviz = include(
        'scrobot_debug', 'rviz.launch.py',
        {'use_sim_time': use_sim_time},
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'use_magnetometer', default_value='true', choices=['true', 'false'],
            description='Disable to reproduce the magnetometer-off October benchmark.',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=EnvironmentVariable('SCROBOT_YOLO_MODEL', default_value=''),
            description='Production shuttle YOLO .pt model.',
        ),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('publish_debug_image', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('shuttle_mode', default_value='mixed', choices=['random', 'cluster', 'mixed']),
        DeclareLaunchArgument('shuttle_count', default_value='50'),
        DeclareLaunchArgument('shuttle_seed', default_value='20261003'),
        DeclareLaunchArgument('robot_x', default_value='2.0'),
        DeclareLaunchArgument('robot_y', default_value='3.05'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='3.14159'),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('return_to_idle', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('run_evaluation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('evaluation_run_name', default_value=''),
        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='5.0'),
        DeclareLaunchArgument('evaluator_delay', default_value='6.0'),
        DeclareLaunchArgument('rviz_delay', default_value='6.0'),
        DeclareLaunchArgument('mission_delay', default_value='15.0'),
        runtime,
        TimerAction(period=stack_delay, actions=[telemetry]),
        TimerAction(period=evaluator_delay, actions=[evaluation]),
        TimerAction(period=rviz_delay, actions=[rviz]),
    ])
