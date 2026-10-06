import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                'launch',
                launch_file,
            )
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    mission_pkg = get_package_share_directory('scrobot_mission')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    base_standoff = LaunchConfiguration('base_standoff')
    launch_rviz = LaunchConfiguration('launch_rviz')

    shuttle_x = LaunchConfiguration('shuttle_x')
    shuttle_y = LaunchConfiguration('shuttle_y')

    stack_delay = LaunchConfiguration('stack_delay')
    perception_delay = LaunchConfiguration('perception_delay')
    shuttle_delay = LaunchConfiguration('shuttle_delay')
    controller_delay = LaunchConfiguration('controller_delay')
    visualizer_delay = LaunchConfiguration('visualizer_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    action_delay = LaunchConfiguration('action_delay')

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
            'x': '0.0',
            'y': '0.0',
            'yaw': '0.0',
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

    localization = include(
        'scrobot_localization',
        'localization.launch.py',
        {
            'use_sim_time': use_sim_time,
            'use_magnetometer': 'false',
        },
    )

    control = include(
        'scrobot_control',
        'control_stack.launch.py',
        {'use_sim_time': use_sim_time},
    )

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
        parameters=[
            local_params,
            {
                'use_sim_time': use_sim_time,
                'base_standoff_distance': base_standoff,
            },
        ],
    )

    spawn_shuttle = include(
        'scrobot_simulation',
        'spawn_shuttles.launch.py',
        {
            'mode': 'single',
            'batch': 'smc_shuttle',
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

    shuttle_visualizer = Node(
        package='scrobot_debug',
        executable='smc_shuttle_visualizer',
        name='smc_shuttle_visualizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    collection_monitor = Node(
        package='scrobot_debug',
        executable='collection_test_monitor',
        name='smc_shuttle_collection_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'center_offset_z': 0.045,
            'base_frame': 'base_footprint',
            'collector_frame': 'collector_link',
            'pickup_half_length': 0.030,
            'pickup_half_width': 0.150,
            'report_rate': 2.0,
        }],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('scrobot_debug'),
                'launch',
                'rviz.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': os.path.join(
                get_package_share_directory('scrobot_debug'),
                'config',
                'smc_shuttle.rviz',
            ),
        }.items(),
        condition=IfCondition(launch_rviz),
    )

    collect_action = ExecuteProcess(
        cmd=[
            'ros2', 'action', 'send_goal',
            '/local_collect',
            'scrobot_interfaces/action/LocalCollect',
            '{timeout_sec: 30.0}',
            '--feedback',
        ],
        output='screen',
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
        DeclareLaunchArgument(
            'base_standoff',
            default_value='1.10',
            description='Desired planar base_link standoff from shuttle [m].',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Launch shuttle SMC RViz debug view.',
        ),

        # Explicit world-frame shuttle placement for SMC trajectory tests.
        # Keep the resulting detected base_link range inside about 0.50..1.80 m.
        DeclareLaunchArgument(
            'shuttle_x',
            default_value='1.50',
            description='Single shuttle world X position [m].',
        ),
        DeclareLaunchArgument(
            'shuttle_y',
            default_value='0.30',
            description='Single shuttle world Y position [m].',
        ),

        DeclareLaunchArgument('perception_delay', default_value='3.0'),
        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='5.0'),
        DeclareLaunchArgument('controller_delay', default_value='5.5'),
        DeclareLaunchArgument('visualizer_delay', default_value='5.8'),
        DeclareLaunchArgument('rviz_delay', default_value='6.2'),
        DeclareLaunchArgument('action_delay', default_value='8.0'),

        simulation,
        TimerAction(period=perception_delay, actions=[perception]),
        TimerAction(
            period=stack_delay,
            actions=[localization, control, map_to_odom],
        ),
        TimerAction(period=shuttle_delay, actions=[spawn_shuttle]),
        TimerAction(
            period=controller_delay,
            actions=[collection_filter, local_collect, telemetry],
        ),
        TimerAction(
            period=visualizer_delay,
            actions=[shuttle_visualizer, collection_monitor],
        ),
        TimerAction(period=rviz_delay, actions=[rviz]),
        TimerAction(period=action_delay, actions=[collect_action]),
    ])
