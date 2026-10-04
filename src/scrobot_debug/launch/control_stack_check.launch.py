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
    debug_pkg = get_package_share_directory('scrobot_debug')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    spawn_delay = LaunchConfiguration('spawn_delay')
    perception_delay = LaunchConfiguration('perception_delay')
    control_delay = LaunchConfiguration('control_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    launch_rviz = LaunchConfiguration('launch_rviz')
    enable_magnetometer = LaunchConfiguration('enable_magnetometer')

    rviz_config = os.path.join(
        debug_pkg,
        'config',
        'control_stack_check.rviz',
    )

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
            'spawn_delay': spawn_delay,
            'enable_magnetometer': enable_magnetometer,
        }.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_pkg, 'launch', 'perception.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'control_stack.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    telemetry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(debug_pkg, 'launch', 'telemetry.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    safety_markers = Node(
        package='scrobot_debug',
        executable='control_safety_markers',
        name='control_safety_markers',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='control_stack_check_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'robot_x',
            default_value='0.0',
            description='Initial robot X [m].',
        ),
        DeclareLaunchArgument(
            'robot_y',
            default_value='0.0',
            description='Initial robot Y [m].',
        ),
        DeclareLaunchArgument(
            'robot_z',
            default_value='0.003',
            description='Initial robot Z [m].',
        ),
        DeclareLaunchArgument(
            'robot_yaw',
            default_value='0.0',
            description='Initial robot yaw [rad].',
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='2.0',
            description='Delay before spawning the robot [s].',
        ),
        DeclareLaunchArgument(
            'perception_delay',
            default_value='3.0',
            description='Delay before starting perception scan sources [s].',
        ),
        DeclareLaunchArgument(
            'control_delay',
            default_value='5.0',
            description='Delay before starting the control stack [s].',
        ),
        DeclareLaunchArgument(
            'rviz_delay',
            default_value='6.0',
            description='Delay before starting the local safety RViz view [s].',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Open the local RViz safety view.',
        ),
        DeclareLaunchArgument(
            'enable_magnetometer',
            default_value='false',
            choices=['true', 'false'],
            description='Enable the optional legacy HMC5883L simulation sensor.',
        ),

        simulation,
        TimerAction(period=perception_delay, actions=[perception]),
        TimerAction(period=control_delay, actions=[control]),
        TimerAction(period=control_delay, actions=[telemetry]),
        TimerAction(period=control_delay, actions=[safety_markers]),
        TimerAction(period=rviz_delay, actions=[rviz]),
    ])
