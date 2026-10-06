import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                'launch',
                launch_file,
            )
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    bringup_pkg = get_package_share_directory('scrobot_bringup')
    control_pkg = get_package_share_directory('scrobot_control')

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    camera_config = LaunchConfiguration('camera_config')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    enable_yolo = LaunchConfiguration('enable_yolo')

    launch_global_localization = LaunchConfiguration(
        'launch_global_localization'
    )
    launch_navigation = LaunchConfiguration('launch_navigation')
    launch_mission = LaunchConfiguration('launch_mission')

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

    control_stack = include(
        'scrobot_control',
        'control_stack.launch.py',
        {'use_sim_time': 'false'},
    )

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        parameters=[
            camera_config,
            {'use_sim_time': False},
        ],
    )

    localization = include(
        'scrobot_localization',
        'localization.launch.py',
        {
            'use_sim_time': 'false',
            'use_magnetometer': 'false',
            'use_sensor_qos_transformer': 'true',
        },
    )

    perception = include(
        'scrobot_perception',
        'perception.launch.py',
        {
            'use_sim_time': 'false',
            'enable_yolo': enable_yolo,
            'model_path': model_path,
            'device': device,
            'publish_debug_image': 'false',
        },
    )

    # These are useful for subsystem testing. The full sweep mission already
    # launches both global localization and Nav2 internally, so leave these
    # disabled when launch_mission:=true.
    global_localization = include(
        'scrobot_localization',
        'global_localization.launch.py',
        {'use_sim_time': 'false'},
        condition=IfCondition(launch_global_localization),
    )

    navigation = include(
        'scrobot_navigation',
        'navigation.launch.py',
        {'use_sim_time': 'false'},
        condition=IfCondition(launch_navigation),
    )

    mission = include(
        'scrobot_mission',
        'sweep_mission.launch.py',
        {'use_sim_time': 'false'},
        condition=IfCondition(launch_mission),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=EnvironmentVariable(
                'SCROBOT_SERIAL_PORT',
                default_value='/dev/ttyTHS1',
            ),
            description='STM32 protocol-v2 UART device.',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='STM32 UART baud rate.',
        ),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(
                bringup_pkg, 'config', 'realsense.yaml'
            ),
            description='D435i production RGB-D + IMU profile.',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=EnvironmentVariable(
                'SCROBOT_YOLO_MODEL',
                default_value='',
            ),
            description='Production shuttle YOLO model.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='Ultralytics inference device on the Orin.',
        ),
        DeclareLaunchArgument(
            'enable_yolo',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'launch_global_localization',
            default_value='false',
            choices=['true', 'false'],
            description='Standalone AprilTag global localization for testing.',
        ),
        DeclareLaunchArgument(
            'launch_navigation',
            default_value='false',
            choices=['true', 'false'],
            description='Standalone Nav2 bringup for testing.',
        ),
        DeclareLaunchArgument(
            'launch_mission',
            default_value='false',
            choices=['true', 'false'],
            description='Start the autonomous sweep mission. Disabled by default for safe hardware bringup.',
        ),
        robot_state_publisher,
        controller_manager,
        realsense,
        control_stack,
        localization,
        perception,
        global_localization,
        navigation,
        mission,
    ])
