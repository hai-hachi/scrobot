import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = get_package_share_directory('scrobot_bringup')
    control_pkg = get_package_share_directory('scrobot_control')
    localization_pkg = get_package_share_directory('scrobot_localization')
    perception_pkg = get_package_share_directory('scrobot_perception')
    navigation_pkg = get_package_share_directory('scrobot_navigation')
    hardware_pkg = get_package_share_directory('scrobot_hardware')

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    launch_navigation = LaunchConfiguration('launch_navigation')
    launch_global_localization = LaunchConfiguration('launch_global_localization')

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

    # Jazzy controller_manager receives the URDF from /robot_description.
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

    control_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'control_stack.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'realsense.yaml'),
            {'use_sim_time': False},
        ],
    )

    magnetometer = Node(
        package='scrobot_hardware',
        executable='magnetometer_5883l.py',
        name='magnetometer_5883l',
        output='screen',
        parameters=[
            os.path.join(hardware_pkg, 'config', 'magnetometer.yaml'),
            {'use_sim_time': False},
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_pkg, 'launch', 'perception.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'cloud_topic': '/camera/camera/depth/color/points',
        }.items(),
    )

    global_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'global_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(launch_global_localization),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(launch_navigation),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyAMA0',
            description='Raspberry Pi hardware UART connected to STM32 USART6.',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='STM32 UART baud rate.',
        ),
        DeclareLaunchArgument(
            'launch_global_localization',
            default_value='true',
            choices=['true', 'false'],
            description='Launch AprilTag map->odom localization and approach action.',
        ),
        DeclareLaunchArgument(
            'launch_navigation',
            default_value='false',
            choices=['true', 'false'],
            description='Launch Nav2 servers. Mission movement is not autostarted.',
        ),

        robot_state_publisher,
        controller_manager,
        realsense,
        magnetometer,
        control_stack,
        localization,
        perception,
        global_localization,
        navigation,
    ])
