from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    world_name = LaunchConfiguration(
        'world_name'
    )

    robot_name = LaunchConfiguration(
        'robot_name'
    )

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # ==========================================================
    # Simulation robot Xacro
    # ==========================================================
    #
    # This wrapper contains:
    #   - scrobot_description
    #   - Gazebo sensors
    #   - ros2_control
    #   - gz_ros2_control
    # ==========================================================

    xacro_file = PathJoinSubstitution([
        FindPackageShare('scrobot_simulation'),
        'urdf',
        'scrobot_sim.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
        ]),
        value_type=str,
    )

    # ==========================================================
    # Robot State Publisher
    # ==========================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ==========================================================
    # Spawn entity in Gazebo
    # ==========================================================

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_scrobot',
        output='screen',
        arguments=[
            '-world', world_name,
            '-name', robot_name,
            '-topic', '/robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo simulation time.',
        ),

        DeclareLaunchArgument(
            'world_name',
            default_value='badminton_court',
            description='Gazebo world name.',
        ),

        DeclareLaunchArgument(
            'robot_name',
            default_value='scrobot',
            description='Gazebo entity name.',
        ),

        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Initial robot X [m].',
        ),

        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Initial robot Y [m].',
        ),

        DeclareLaunchArgument(
            'z',
            default_value='0.003',
            description='Initial robot Z [m].',
        ),

        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial robot yaw [rad].',
        ),

        robot_state_publisher,
        spawn_robot,
    ])
