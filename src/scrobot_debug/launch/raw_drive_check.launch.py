import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulation_pkg = get_package_share_directory('scrobot_simulation')
    debug_pkg = get_package_share_directory('scrobot_debug')
    control_pkg = get_package_share_directory('scrobot_control')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')
    drive_contact_mu = LaunchConfiguration('drive_contact_mu')
    caster_contact_mu = LaunchConfiguration('caster_contact_mu')
    controller_delay = LaunchConfiguration('controller_delay')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x': robot_x,
            'y': robot_y,
            'yaw': robot_yaw,
            'enable_magnetometer': 'false',
            'drive_contact_mu': drive_contact_mu,
            'caster_contact_mu': caster_contact_mu,
        }.items(),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'controllers.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    telemetry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(debug_pkg, 'launch', 'telemetry.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz_debug = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(debug_pkg, 'launch', 'rviz_local.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(rviz),
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
        ),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'drive_contact_mu',
            default_value='5.0',
            description='Drive-wheel Gazebo contact friction for skid tuning.',
        ),
        DeclareLaunchArgument(
            'caster_contact_mu',
            default_value='0.05',
            description='Caster Gazebo contact friction for skid tuning.',
        ),
        DeclareLaunchArgument('controller_delay', default_value='4.0'),

        simulation,
        rviz_debug,
        TimerAction(
            period=controller_delay,
            actions=[controllers, telemetry],
        ),
    ])
