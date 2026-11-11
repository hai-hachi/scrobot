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
    enable_magnetometer = LaunchConfiguration('enable_magnetometer')
    enable_depth_camera = LaunchConfiguration('enable_depth_camera')
    robot_visual_mode = LaunchConfiguration('robot_visual_mode')
    wheel_joint_damping = LaunchConfiguration('wheel_joint_damping')
    wheel_collision_width = LaunchConfiguration('wheel_collision_width')
    drive_contact_mu = LaunchConfiguration('drive_contact_mu')
    caster_contact_mu = LaunchConfiguration('caster_contact_mu')
    drive_slip1 = LaunchConfiguration('drive_slip1')
    drive_slip2 = LaunchConfiguration('drive_slip2')
    controller_params = LaunchConfiguration('controller_params')

    # ==========================================================
    # Simulation robot Xacro
    # ==========================================================
    #
    # This wrapper contains:
    #   - scrobot_description
    #   - Gazebo sensors
    #   - ros2_control
    #   - gz_ros2_control
    #
    # The visual / physical wheel remains 30 mm wide, but Gazebo intentionally
    # uses a 1 mm collision width. This narrow line-contact approximation was
    # retained because the full-width collision produced excessive lateral
    # skid in the differential-drive simulation.
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
            ' wheel_collision_width:=',
            wheel_collision_width,
            ' wheel_joint_damping:=',
            wheel_joint_damping,
            ' enable_magnetometer:=',
            enable_magnetometer,
            ' enable_depth_camera:=',
            enable_depth_camera,
            ' robot_visual_mode:=',
            robot_visual_mode,
            ' drive_contact_mu:=',
            drive_contact_mu,
            ' caster_contact_mu:=',
            caster_contact_mu,
            ' drive_slip1:=',
            drive_slip1,
            ' drive_slip2:=',
            drive_slip2,
            ' controller_params:=',
            controller_params,
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

        DeclareLaunchArgument(
            'enable_magnetometer',
            default_value='true',
            choices=['true', 'false'],
            description='Enable the simulated 5883L magnetometer and its TF frame.',
        ),
        DeclareLaunchArgument(
            'enable_depth_camera', default_value='true', choices=['true', 'false'],
            description='Enable the real-resolution Gazebo depth sensor.',
        ),
        DeclareLaunchArgument(
            'robot_visual_mode', default_value='full', choices=['full', 'simple', 'none'],
            description='Visual rendering only: full / simple / none; joints and collisions stay.',
        ),
        DeclareLaunchArgument(
            'wheel_joint_damping', default_value='0.0',
            description='Wheel bearing viscous damping N*m*s/rad (lab calibrates, zero baseline).',
        ),
        DeclareLaunchArgument(
            'wheel_collision_width', default_value='0.001',
            description='Simulated wheel collision width [m]; production narrow contact 0.001 m.',
        ),
        DeclareLaunchArgument(
            'drive_contact_mu',
            default_value='5.0',
            description=(
                'Gazebo drive-wheel contact friction coefficient. '
                'Tuned value; not a measured material coefficient.'
            ),
        ),
        DeclareLaunchArgument(
            'caster_contact_mu',
            default_value='0.05',
            description=(
                'Gazebo passive-caster contact friction coefficient.'
            ),
        ),
        DeclareLaunchArgument('drive_slip1', default_value='0.0',
                              description='Drive tire rolling-direction force-dependent slip.'),
        DeclareLaunchArgument('drive_slip2', default_value='0.0',
                              description='Drive tire lateral force-dependent slip.'),
        DeclareLaunchArgument(
            'controller_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('scrobot_control'),
                'config',
                'controllers.yaml',
            ]),
            description='ros2_control controller YAML loaded by Gazebo.',
        ),

        robot_state_publisher,
        spawn_robot,
    ])
