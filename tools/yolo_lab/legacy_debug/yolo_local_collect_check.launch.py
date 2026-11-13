import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                'launch',
                filename,
            )
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    mission_pkg = get_package_share_directory('scrobot_mission')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    shuttle_x = LaunchConfiguration('shuttle_x')
    shuttle_y = LaunchConfiguration('shuttle_y')

    perception_delay = LaunchConfiguration('perception_delay')
    stack_delay = LaunchConfiguration('stack_delay')
    shuttle_delay = LaunchConfiguration('shuttle_delay')
    local_delay = LaunchConfiguration('local_delay')
    telemetry_delay = LaunchConfiguration('telemetry_delay')

    local_params = os.path.join(
        mission_pkg,
        'config',
        'local_collect.yaml',
    )

    simulation = include(
        'scrobot_simulation',
        'simulation.launch.py',
        {
            'use_sim_time': use_sim_time,
            'x': robot_x,
            'y': robot_y,
            'z': robot_z,
            'yaw': robot_yaw,
            'enable_magnetometer': 'false',
        },
    )

    perception = include(
        'scrobot_perception',
        'perception.launch.py',
        {
            'use_sim_time': use_sim_time,
            'enable_yolo': 'true',
            'model_path': model_path,
            'device': device,
            'publish_debug_image': 'true',
        },
    )

    control = include(
        'scrobot_control',
        'control_stack.launch.py',
        {'use_sim_time': use_sim_time},
    )

    localization = include(
        'scrobot_localization',
        'localization.launch.py',
        {
            'use_sim_time': use_sim_time,
            'use_magnetometer': 'false',
        },
    )

    # This isolated local-collection test does not run global localization.
    # Identity map->odom is sufficient for the collection filter's net-pole
    # exclusion check while EKF still owns odom->base_footprint.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='debug_map_to_odom',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    collection_filter = Node(
        package='scrobot_mission',
        executable='shuttle_collection_filter',
        name='shuttle_collection_filter',
        output='screen',
        parameters=[local_params, {'use_sim_time': use_sim_time}],
    )

    local_collect = Node(
        package='scrobot_mission',
        executable='local_collect_controller',
        name='local_collect_controller',
        output='screen',
        parameters=[local_params, {'use_sim_time': use_sim_time}],
    )

    spawn_shuttle = include(
        'scrobot_simulation',
        'spawn_shuttles.launch.py',
        {
            'mode': 'single',
            'batch': 'yolo_local_collect',
            'x': shuttle_x,
            'y': shuttle_y,
        },
    )

    telemetry = include(
        'scrobot_debug',
        'telemetry.launch.py',
        {
            'use_sim_time': use_sim_time,
            'summary_rate': '2.0',
            'min_rosout_level': '20',
        },
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

        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='0.003'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),

        # Slightly off-center so the SMC pose stage must correct heading/lateral
        # error instead of reducing to a straight drive.
        DeclareLaunchArgument('shuttle_x', default_value='1.20'),
        DeclareLaunchArgument('shuttle_y', default_value='0.20'),

        DeclareLaunchArgument('perception_delay', default_value='3.0'),
        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='5.0'),
        DeclareLaunchArgument('local_delay', default_value='5.5'),
        DeclareLaunchArgument('telemetry_delay', default_value='6.0'),

        simulation,
        TimerAction(
            period=perception_delay,
            actions=[perception],
        ),
        TimerAction(
            period=stack_delay,
            actions=[control, localization, map_to_odom],
        ),
        TimerAction(
            period=shuttle_delay,
            actions=[spawn_shuttle],
        ),
        TimerAction(
            period=local_delay,
            actions=[collection_filter, local_collect],
        ),
        TimerAction(
            period=telemetry_delay,
            actions=[telemetry],
        ),
    ])
