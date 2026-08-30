from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        FindPackageShare('scrobot_simulation'),
        'urdf',
        'scrobot_sim.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file
        ]),
        value_type=str
    )

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',

        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-world', 'badminton_court',
            '-name', 'scrobot',

            '-topic', 'robot_description',

            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ],

        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'x',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'y',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'z',
            default_value='0.02'
        ),

        DeclareLaunchArgument(
            'yaw',
            default_value='0.0'
        ),

        robot_state_publisher,

        spawn_robot

    ])