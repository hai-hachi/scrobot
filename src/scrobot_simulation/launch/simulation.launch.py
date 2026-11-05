import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    simulation_pkg = get_package_share_directory(
        'scrobot_simulation'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    local_only = LaunchConfiguration('local_only')
    world_name = LaunchConfiguration('world_name')
    robot_name = LaunchConfiguration('robot_name')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    enable_magnetometer = LaunchConfiguration('enable_magnetometer')
    drive_contact_mu = LaunchConfiguration('drive_contact_mu')
    caster_contact_mu = LaunchConfiguration('caster_contact_mu')
    controller_params = LaunchConfiguration('controller_params')

    spawn_delay = LaunchConfiguration('spawn_delay')

    # gazebo.launch.py provides:
    #   RGB image   -> ros_gz_image -> /camera/camera/color/image_raw
    #   Depth image -> ros_gz_image -> /camera/camera/depth/image_raw
    #   Native Gazebo point cloud
    #       -> ros_gz_bridge -> /camera/camera/depth/points
    #
    # depth_image_proc::PointCloudXyzNode is intentionally not
    # launched in simulation anymore.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_pkg,
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={
            'world': world,
            'gz_verbosity': gz_verbosity,
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_pkg,
                'launch',
                'spawn_robot.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_name': world_name,
            'robot_name': robot_name,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'enable_magnetometer': enable_magnetometer,
            'drive_contact_mu': drive_contact_mu,
            'caster_contact_mu': caster_contact_mu,
            'controller_params': controller_params,
        }.items(),
    )

    delayed_spawn_robot = TimerAction(
        period=spawn_delay,
        actions=[spawn_robot],
    )

    depth_to_color_registration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_pkg,
                'launch',
                'depth_to_color_registration.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'local_only',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Keep Gazebo Transport and ROS 2 DDS on loopback for a '
                'single-PC simulation (helps when Cloudflare WARP/VPN is active). '
                'Use false for multi-host simulation.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                simulation_pkg,
                'worlds',
                'badminton_court.sdf',
            ),
            description='Absolute path to Gazebo world file.',
        ),
        DeclareLaunchArgument(
            'gz_verbosity',
            default_value='3',
            description='Gazebo verbosity level.',
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
            'spawn_delay',
            default_value='2.0',
            description='Delay before spawning robot [s].',
        ),
        DeclareLaunchArgument(
            'enable_magnetometer',
            default_value='false',
            choices=['true', 'false'],
            description='Enable the optional legacy HMC5883L simulation sensor.',
        ),
        DeclareLaunchArgument(
            'drive_contact_mu',
            default_value='5.0',
            description='Gazebo drive-wheel contact friction coefficient.',
        ),
        DeclareLaunchArgument(
            'caster_contact_mu',
            default_value='0.05',
            description='Gazebo passive-caster contact friction coefficient.',
        ),
        DeclareLaunchArgument(
            'controller_params',
            default_value=os.path.join(
                get_package_share_directory('scrobot_control'),
                'config',
                'controllers.yaml',
            ),
            description='ros2_control controller YAML loaded by Gazebo.',
        ),

        # Configure the network before starting Gazebo, bridges, RSP, or spawn.
        # ROS and Gazebo have independent discovery transports; both settings
        # are needed to avoid a VPN interface becoming the advertised address.
        # Applied only in simulation, not in physical robot bringup.
        SetEnvironmentVariable(
            name='ROS_AUTOMATIC_DISCOVERY_RANGE',
            value='LOCALHOST',
            condition=IfCondition(local_only),
        ),
        SetEnvironmentVariable(
            name='GZ_IP',
            value='127.0.0.1',
            condition=IfCondition(local_only),
        ),
        gazebo,
        delayed_spawn_robot,
        depth_to_color_registration,
    ])
