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
    perception_pkg = get_package_share_directory('scrobot_perception')
    control_pkg = get_package_share_directory('scrobot_control')
    localization_pkg = get_package_share_directory('scrobot_localization')
    debug_pkg = get_package_share_directory('scrobot_debug')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_magnetometer = LaunchConfiguration('use_magnetometer')
    launch_rviz = LaunchConfiguration('launch_rviz')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    perception_delay = LaunchConfiguration('perception_delay')
    localization_delay = LaunchConfiguration('localization_delay')
    control_delay = LaunchConfiguration('control_delay')
    global_delay = LaunchConfiguration('global_delay')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x': robot_x,
            'y': robot_y,
            'z': robot_z,
            'yaw': robot_yaw,
            'enable_magnetometer': use_magnetometer,
        }.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_pkg, 'launch', 'perception.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_magnetometer': use_magnetometer,
        }.items(),
    )

    # Production controller config keeps enable_odom_tf=false. The EKF in this
    # launch is therefore the only owner of odom -> base_footprint.
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'control_stack.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    global_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'global_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    monitor = Node(
        package='scrobot_debug',
        executable='localization_monitor',
        name='localization_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'report_rate': 1.0,
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
            'use_magnetometer',
            default_value='false',
            choices=['true', 'false'],
            description='Enable simulated /imu/mag and fuse it in Madgwick.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Launch RViz only when visual inspection is needed.',
        ),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('perception_delay', default_value='3.0'),
        DeclareLaunchArgument('localization_delay', default_value='3.5'),
        DeclareLaunchArgument('control_delay', default_value='4.0'),
        DeclareLaunchArgument('global_delay', default_value='5.0'),

        simulation,
        TimerAction(period=perception_delay, actions=[perception]),
        TimerAction(period=localization_delay, actions=[localization]),
        TimerAction(period=control_delay, actions=[control]),
        TimerAction(
            period=global_delay,
            actions=[global_localization, monitor, rviz],
        ),
    ])
