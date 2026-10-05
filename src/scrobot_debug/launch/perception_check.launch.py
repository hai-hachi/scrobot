import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_pkg = get_package_share_directory('scrobot_simulation')
    perception_pkg = get_package_share_directory('scrobot_perception')
    debug_pkg = get_package_share_directory('scrobot_debug')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    launch_rviz = LaunchConfiguration('launch_rviz')
    publish_debug_image = LaunchConfiguration('publish_debug_image')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    shuttle_x = LaunchConfiguration('shuttle_x')
    shuttle_y = LaunchConfiguration('shuttle_y')

    perception_delay = LaunchConfiguration('perception_delay')
    shuttle_delay = LaunchConfiguration('shuttle_delay')
    yolo_delay = LaunchConfiguration('yolo_delay')
    monitor_delay = LaunchConfiguration('monitor_delay')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg, 'launch', 'simulation.launch.py')
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

    production_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_pkg, 'launch', 'perception.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_shuttle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg, 'launch', 'spawn_shuttles.launch.py')
        ),
        launch_arguments={
            'mode': 'single',
            'batch': 'perception_check',
            'x': shuttle_x,
            'y': shuttle_y,
        }.items(),
    )

    yolo_params = os.path.join(
        perception_pkg,
        'config',
        'yolo_shuttle_detector.yaml',
    )

    yolo = Node(
        package='scrobot_perception',
        executable='yolo_shuttle_detector',
        name='yolo_shuttle_detector',
        output='screen',
        parameters=[
            yolo_params,
            {
                'use_sim_time': use_sim_time,
                'model_path': model_path,
                'device': device,
                'publish_debug_image': publish_debug_image,
            },
        ],
    )

    monitor = Node(
        package='scrobot_debug',
        executable='perception_monitor',
        name='perception_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'report_rate': 1.0,
            'stale_timeout': 2.0,
        }],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(debug_pkg, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
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
        ),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('shuttle_x', default_value='1.00'),
        DeclareLaunchArgument('shuttle_y', default_value='0.00'),
        DeclareLaunchArgument('perception_delay', default_value='3.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='4.0'),
        DeclareLaunchArgument('yolo_delay', default_value='5.0'),
        DeclareLaunchArgument('monitor_delay', default_value='6.0'),
        simulation,
        TimerAction(period=perception_delay, actions=[production_perception]),
        TimerAction(period=shuttle_delay, actions=[spawn_shuttle]),
        TimerAction(period=yolo_delay, actions=[yolo]),
        TimerAction(period=monitor_delay, actions=[monitor, rviz]),
    ])
