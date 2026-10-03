import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_pkg = get_package_share_directory('scrobot_simulation')
    control_pkg = get_package_share_directory('scrobot_control')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    start_control = LaunchConfiguration('start_control')
    start_shuttle = LaunchConfiguration('start_shuttle')
    start_monitor = LaunchConfiguration('start_monitor')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    shuttle_x = LaunchConfiguration('shuttle_x')
    shuttle_y = LaunchConfiguration('shuttle_y')
    shuttle_batch = LaunchConfiguration('shuttle_batch')

    control_delay = LaunchConfiguration('control_delay')
    shuttle_delay = LaunchConfiguration('shuttle_delay')
    monitor_delay = LaunchConfiguration('monitor_delay')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'x': robot_x,
            'y': robot_y,
            'z': robot_z,
            'yaw': robot_yaw,
            'enable_magnetometer': 'false',
        }.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'control_stack.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(start_control),
    )

    spawn_shuttle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg, 'launch', 'spawn_shuttles.launch.py')
        ),
        launch_arguments={
            'mode': 'single',
            'visual': 'detail',
            'batch': shuttle_batch,
            'x': shuttle_x,
            'y': shuttle_y,
        }.items(),
        condition=IfCondition(start_shuttle),
    )

    monitor = Node(
        package='scrobot_debug',
        executable='shuttle_sim_monitor.py',
        name='shuttle_sim_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pickup_offset_x': 0.165,
            'pickup_half_length': 0.030,
            'pickup_half_width': 0.150,
            'shuttle_radius': 0.034,
            'movement_warning_threshold': 0.001,
            'report_rate': 2.0,
        }],
        condition=IfCondition(start_monitor),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Launch RViz with the simulation.',
        ),
        DeclareLaunchArgument(
            'start_control',
            default_value='true',
            choices=['true', 'false'],
            description='Start ros2_control and the command pipeline.',
        ),
        DeclareLaunchArgument(
            'start_shuttle',
            default_value='true',
            choices=['true', 'false'],
            description='Spawn one detailed shuttle for the test.',
        ),
        DeclareLaunchArgument(
            'start_monitor',
            default_value='true',
            choices=['true', 'false'],
            description='Start the shuttle physics/collision monitor.',
        ),

        DeclareLaunchArgument('robot_x', default_value='-1.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),

        DeclareLaunchArgument(
            'shuttle_x',
            default_value='0.0',
            description='World X coordinate of the single test shuttle [m].',
        ),
        DeclareLaunchArgument(
            'shuttle_y',
            default_value='0.0',
            description='World Y coordinate of the single test shuttle [m].',
        ),
        DeclareLaunchArgument(
            'shuttle_batch',
            default_value='',
            description='Optional suffix for the spawned shuttle entity name.',
        ),

        DeclareLaunchArgument('control_delay', default_value='4.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='4.5'),
        DeclareLaunchArgument('monitor_delay', default_value='5.0'),

        simulation,
        TimerAction(period=control_delay, actions=[control]),
        TimerAction(period=shuttle_delay, actions=[spawn_shuttle]),
        TimerAction(period=monitor_delay, actions=[monitor]),
    ])
