import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    debug_pkg = get_package_share_directory('scrobot_debug')

    use_sim_time = LaunchConfiguration('use_sim_time')
    target_distance = LaunchConfiguration('target_distance')
    preferred_tag_id = LaunchConfiguration('preferred_tag_id')
    controller_strategy = LaunchConfiguration('controller_strategy')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    position_tolerance = LaunchConfiguration('position_tolerance')
    yaw_tolerance_deg = LaunchConfiguration('yaw_tolerance_deg')
    launch_rviz = LaunchConfiguration('launch_rviz')

    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    stack_delay = LaunchConfiguration('stack_delay')
    visualizer_delay = LaunchConfiguration('visualizer_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    approach_delay = LaunchConfiguration('approach_delay')
    approach_timeout = LaunchConfiguration('approach_timeout')

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
            'enable_yolo': 'false',
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

    global_localization = include(
        'scrobot_localization',
        'global_localization.launch.py',
        {
            'use_sim_time': use_sim_time,
            'tag_position_tolerance': position_tolerance,
            'tag_yaw_tolerance_deg': yaw_tolerance_deg,
            'tag_control_strategy': controller_strategy,
            'tag_max_linear_velocity': max_linear_velocity,
            'tag_max_angular_velocity': max_angular_velocity,
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

    visualizer = Node(
        package='scrobot_debug',
        executable='smc_tag_visualizer',
        name='smc_tag_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tag_id': ParameterValue(preferred_tag_id, value_type=int),
            'target_distance': ParameterValue(target_distance, value_type=float),
        }],
    )

    # Reuse the master debug RViz launcher so the existing court/net/pole
    # visualizer remains the single source for court geometry. Only the RViz
    # display config is replaced with the lightweight SMC-specific view.
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(debug_pkg, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': os.path.join(debug_pkg, 'config', 'smc_tag.rviz'),
        }.items(),
        condition=IfCondition(launch_rviz),
    )

    # The isolated tag-approach action intentionally does not run /relocalize,
    # so the global localizer has not committed map->odom yet. For RViz only,
    # anchor odom to the known Gazebo spawn pose. The SMC itself still controls
    # entirely in odom and does not consume this transform.
    debug_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='smc_tag_debug_map_to_odom',
        arguments=[
            '--x', robot_x,
            '--y', robot_y,
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', robot_yaw,
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    def send_approach(context):
        tag_id = preferred_tag_id.perform(context)
        distance = target_distance.perform(context)
        timeout = approach_timeout.perform(context)
        goal = (
            '{preferred_tag_id: ' + tag_id
            + ', target_distance: ' + distance
            + ', timeout_sec: ' + timeout + '}'
        )
        return [
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/approach_tag',
                    'scrobot_interfaces/action/ApproachTag',
                    goal,
                    '--feedback',
                ],
                output='screen',
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'target_distance',
            default_value='0.90',
            description='Desired planar base_link standoff from tag [m].',
        ),
        DeclareLaunchArgument(
            'preferred_tag_id',
            default_value='0',
            description='Court AprilTag ID used for the isolated controller test.',
        ),
        DeclareLaunchArgument(
            'controller_strategy',
            default_value='pure_smc',
            choices=[
                'pure_smc',
                'main_branch',
                'biarc_smc',
                'normal_ray_smc',
            ],
            description='Tag local approach strategy under test.',
        ),
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.45',
            description='Controller linear speed limit [m/s].',
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='0.75',
            description='Controller angular speed limit [rad/s].',
        ),
        DeclareLaunchArgument(
            'position_tolerance',
            default_value='0.05',
            description='SMC planar desired-pose tolerance [m].',
        ),
        DeclareLaunchArgument(
            'yaw_tolerance_deg',
            default_value='5.0',
            description='SMC desired-heading tolerance [deg].',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Launch the lightweight SMC tag RViz view.',
        ),

        # Near tag 0, intentionally displaced from its desired pose so the
        # controller must correct both position and heading.
        DeclareLaunchArgument(
            'robot_x',
            default_value='1.50',
            description='Robot initial court X [m].',
        ),
        DeclareLaunchArgument(
            'robot_y',
            default_value='1.80',
            description='Robot initial court Y [m].',
        ),
        DeclareLaunchArgument(
            'robot_z',
            default_value='0.003',
            description='Robot initial Z [m].',
        ),
        DeclareLaunchArgument(
            'robot_yaw',
            default_value='2.80',
            description='Robot initial yaw [rad].',
        ),

        DeclareLaunchArgument('stack_delay', default_value='4.0'),
        DeclareLaunchArgument('visualizer_delay', default_value='4.5'),
        DeclareLaunchArgument('rviz_delay', default_value='5.5'),
        DeclareLaunchArgument('approach_delay', default_value='8.0'),
        DeclareLaunchArgument(
            'approach_timeout',
            default_value='60.0',
            description='Approach action timeout [s].',
        ),

        simulation,
        TimerAction(
            period=stack_delay,
            actions=[
                perception,
                localization,
                control,
                global_localization,
                telemetry,
                debug_map_to_odom,
            ],
        ),
        TimerAction(
            period=visualizer_delay,
            actions=[visualizer],
        ),
        TimerAction(
            period=rviz_delay,
            actions=[rviz],
        ),
        TimerAction(
            period=approach_delay,
            actions=[OpaqueFunction(function=send_approach)],
        ),
    ])
