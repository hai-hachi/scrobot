import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_pkg = get_package_share_directory('scrobot_control')

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('scrobot_description'),
        'urdf',
        'scrobot.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' use_real_hardware:=true',
            ' serial_port:=',
            serial_port,
            ' baud_rate:=',
            baud_rate,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            os.path.join(control_pkg, 'config', 'controllers.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[
            ('robot_description', '/robot_description'),
        ],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyAMA0',
            description='Raspberry Pi PL011 UART connected to STM32 USART6.',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='STM32 UART protocol-v2 baud rate.',
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        diff_drive_controller,
    ])
