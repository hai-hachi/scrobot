import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
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


def single_shuttle(batch, x, y):
    return include(
        'scrobot_simulation',
        'spawn_shuttles.launch.py',
        {
            'mode': 'single',
            'batch': batch,
            'x': str(x),
            'y': str(y),
        },
    )


def generate_launch_description():
    mission_pkg = get_package_share_directory('scrobot_mission')
    debug_pkg = get_package_share_directory('scrobot_debug')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    launch_rviz = LaunchConfiguration('launch_rviz')

    perception_delay = LaunchConfiguration('perception_delay')
    stack_delay = LaunchConfiguration('stack_delay')
    shuttle_delay = LaunchConfiguration('shuttle_delay')
    controller_delay = LaunchConfiguration('controller_delay')
    monitor_delay = LaunchConfiguration('monitor_delay')
    action_delay = LaunchConfiguration('action_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')

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

    monitor = Node(
        package='scrobot_debug',
        executable='yolo_multi_shuttle_monitor',
        name='yolo_multi_shuttle_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'report_rate': 1.0,
            'association_radius': 0.25,
        }],
    )

    shuttle_visualizer = Node(
        package='scrobot_debug',
        executable='smc_shuttle_visualizer',
        name='smc_shuttle_visualizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                debug_pkg,
                'launch',
                'rviz.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': os.path.join(
                debug_pkg,
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
            '{timeout_sec: 90.0}',
            '--feedback',
        ],
        output='screen',
    )

    # Initial geometry, robot at world origin:
    # A: r ~= 1.346 m, bearing ~= -21.8 deg  -> valid
    # B: r ~= 1.458 m, bearing ~=  -5.9 deg  -> valid
    # C: r ~= 1.612 m, bearing ~=  +7.1 deg  -> valid
    # D: r ~= 1.850 m, bearing ~= +18.9 deg  -> initially filtered
    shuttle_a = single_shuttle('multi_A', 1.25, -0.50)
    shuttle_b = single_shuttle('multi_B', 1.45, -0.15)
    shuttle_c = single_shuttle('multi_C', 1.60, +0.20)
    shuttle_d = single_shuttle('multi_D', 1.75, +0.60)

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
            'launch_rviz',
            default_value='true',
            choices=['true', 'false'],
        ),

        DeclareLaunchArgument('perception_delay', default_value='3.0'),
        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('shuttle_delay', default_value='5.0'),
        DeclareLaunchArgument('controller_delay', default_value='6.0'),
        DeclareLaunchArgument('monitor_delay', default_value='6.5'),
        DeclareLaunchArgument(
            'action_delay',
            default_value='10.0',
            description=(
                'Leave several seconds to inspect initial raw/filter counts '
                'before starting collection.'
            ),
        ),
        DeclareLaunchArgument('rviz_delay', default_value='7.0'),

        simulation,
        TimerAction(period=perception_delay, actions=[perception]),
        TimerAction(
            period=stack_delay,
            actions=[control, localization, map_to_odom],
        ),
        TimerAction(
            period=shuttle_delay,
            actions=[shuttle_a, shuttle_b, shuttle_c, shuttle_d],
        ),
        TimerAction(
            period=controller_delay,
            actions=[collection_filter, local_collect],
        ),
        TimerAction(
            period=monitor_delay,
            actions=[monitor, shuttle_visualizer],
        ),
        TimerAction(period=rviz_delay, actions=[rviz]),
        TimerAction(period=action_delay, actions=[collect_action]),
    ])
