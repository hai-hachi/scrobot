import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_TESTS = {'raw', 'manual', 'mux', 'smoother', 'collision', 'full'}


def include(pkg, launch_file, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg),
                'launch',
                launch_file,
            )
        ),
        launch_arguments=(args or {}).items(),
        condition=condition,
    )


def launch_setup(context):
    test = LaunchConfiguration('test').perform(context)
    if test not in VALID_TESTS:
        raise RuntimeError(
            f'Unknown control test "{test}". '
            f'Choose one of: {", ".join(sorted(VALID_TESTS))}'
        )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')
    stack_delay = LaunchConfiguration('stack_delay')

    simulation = include(
        'scrobot_simulation',
        'simulation.launch.py',
        {
            'use_sim_time': use_sim_time,
            'x': robot_x,
            'y': robot_y,
            'yaw': robot_yaw,
            'enable_magnetometer': 'false',
        },
    )

    if test == 'raw':
        control = include(
            'scrobot_control',
            'controllers.launch.py',
            {'use_sim_time': use_sim_time},
        )
    else:
        control = include(
            'scrobot_control',
            'control_stack.launch.py',
            {'use_sim_time': use_sim_time},
        )

    actions = [
        LogInfo(msg=[f'CONTROL CHECK: test={test}']),
        simulation,
        TimerAction(period=stack_delay, actions=[control]),
    ]

    # The production command path ends at collision_monitor, which requires
    # /camera/camera/depth/scan. Provide only the depth point-cloud adapter here;
    # AprilTag detection is unrelated to scrobot_control testing.
    if test != 'raw':
        perception_pkg = get_package_share_directory('scrobot_perception')
        perception_params = os.path.join(
            perception_pkg,
            'config',
            'perception.yaml',
        )
        depth_scan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='depth_pointcloud_to_scan',
            output='screen',
            parameters=[
                perception_params,
                {
                    'use_sim_time': use_sim_time,
                    'min_height': 0.08,
                    'max_height': 0.70,
                },
            ],
            remappings=[
                ('cloud_in', '/camera/camera/depth/points'),
                ('scan', '/camera/camera/depth/scan'),
            ],
        )
        actions.append(
            TimerAction(period=stack_delay, actions=[depth_scan])
        )

    telemetry = include(
        'scrobot_debug',
        'telemetry.launch.py',
        {
            'use_sim_time': use_sim_time,
            'summary_rate': '1.0',
        },
    )
    actions.append(TimerAction(period=stack_delay, actions=[telemetry]))

    rviz_local = include(
        'scrobot_debug',
        'rviz_local.launch.py',
        {'use_sim_time': use_sim_time},
        condition=IfCondition(rviz),
    )
    actions.append(rviz_local)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'test',
            default_value='manual',
            choices=['raw', 'manual', 'mux', 'smoother', 'collision', 'full'],
            description='scrobot_control test scenario.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'stack_delay',
            default_value='4.0',
            description='Delay before starting control and debug nodes [s].',
        ),
        OpaqueFunction(function=launch_setup),
    ])
