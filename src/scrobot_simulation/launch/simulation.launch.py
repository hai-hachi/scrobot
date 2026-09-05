import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    simulation_pkg = get_package_share_directory(
        'scrobot_simulation'
    )

    description_pkg = get_package_share_directory(
        'scrobot_description'
    )

    # ==========================================================
    # Launch arguments
    # ==========================================================

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    launch_rviz = LaunchConfiguration(
        'rviz'
    )

    world = LaunchConfiguration(
        'world'
    )

    gz_verbosity = LaunchConfiguration(
        'gz_verbosity'
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

    spawn_delay = LaunchConfiguration(
        'spawn_delay'
    )

    # ==========================================================
    # Resource paths
    # ==========================================================

    description_share_parent = os.path.dirname(
        description_pkg
    )

    simulation_models = os.path.join(
        simulation_pkg,
        'models'
    )

    add_description_resources = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=description_share_parent,
    )

    add_simulation_models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=simulation_models,
    )

    # ==========================================================
    # Gazebo
    # ==========================================================

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

    # ==========================================================
    # Robot spawn / robot_state_publisher
    # ==========================================================

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
        }.items(),
    )

    delayed_spawn_robot = TimerAction(
        period=spawn_delay,
        actions=[
            spawn_robot,
        ],
    )

    # ==========================================================
    # RealSense-like ROS processing
    # ==========================================================
    #
    # Creates:
    #   /camera/camera/aligned_depth_to_color/image_raw
    #   /camera/camera/aligned_depth_to_color/camera_info
    #   /camera/camera/depth/points
    #
    # The processing nodes may start before the camera topics exist;
    # they will simply wait for publishers and TF.
    # ==========================================================

    realsense_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_pkg,
                'launch',
                'depth_registration.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ==========================================================
    # RViz
    # ==========================================================

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_pkg,
                'launch',
                'rviz.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(
            launch_rviz
        ),
    )

    return LaunchDescription([
        # ------------------------------------------------------
        # Arguments
        # ------------------------------------------------------

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo simulation time.',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Launch RViz.',
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

        # ------------------------------------------------------
        # Environment
        # ------------------------------------------------------

        add_description_resources,
        add_simulation_models,

        # ------------------------------------------------------
        # Processes
        # ------------------------------------------------------

        gazebo,
        delayed_spawn_robot,
        realsense_processing,
        rviz,
    ])
