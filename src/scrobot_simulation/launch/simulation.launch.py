import os
import sys
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    simulation_pkg = get_package_share_directory(
        'scrobot_simulation'
    )
    # The Mobility Lab publishes a single simulation-only YAML. Launch
    # arguments still override these defaults for isolated experiment runs.
    calibration_path = Path(simulation_pkg) / 'config/mobility_calibration.yaml'
    with calibration_path.open('r', encoding='utf-8') as stream:
        calibration = yaml.safe_load(stream)
    if not isinstance(calibration, dict) or calibration.get('schema_version') != 1:
        raise RuntimeError(f'Invalid mobility calibration: {calibration_path}')
    physics = calibration['physics']
    camera_config_path = Path(simulation_pkg) / 'config/perception_calibration.yaml'
    with camera_config_path.open('r', encoding='utf-8') as stream:
        camera_config = yaml.safe_load(stream)
    if not isinstance(camera_config, dict) or camera_config.get('schema_version') != 1:
        raise RuntimeError(f'Invalid published perception calibration: {camera_config_path}')
    color_config = camera_config['color']
    depth_config = camera_config['depth']

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from odometry_rate_config import load_rates, prepare_configurations
    rate_config = Path(simulation_pkg) / 'config/odometry_calibration.yaml'
    published_rates = load_rates(rate_config)
    production_controller = (
        Path(get_package_share_directory('scrobot_control')) /
        'config/controllers.yaml'
    )
    production_ekf = (
        Path(get_package_share_directory('scrobot_localization')) /
        'config/ekf.yaml'
    )
    generated = prepare_configurations(
        published_rates, production_controller, production_ekf
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    world_name = LaunchConfiguration('world_name')
    robot_name = LaunchConfiguration('robot_name')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    enable_magnetometer = LaunchConfiguration('enable_magnetometer')
    imu_rate_hz = LaunchConfiguration('imu_rate_hz')
    magnetometer_rate_hz = LaunchConfiguration('magnetometer_rate_hz')
    ground_truth_rate_hz = LaunchConfiguration('ground_truth_rate_hz')
    color_width = LaunchConfiguration('color_width')
    color_height = LaunchConfiguration('color_height')
    color_fps = LaunchConfiguration('color_fps')
    depth_width = LaunchConfiguration('depth_width')
    depth_height = LaunchConfiguration('depth_height')
    depth_fps = LaunchConfiguration('depth_fps')
    enable_depth_camera = LaunchConfiguration('enable_depth_camera')
    enable_depth_registration = LaunchConfiguration('enable_depth_registration')
    robot_visual_mode = LaunchConfiguration('robot_visual_mode')
    wheel_joint_damping = LaunchConfiguration('wheel_joint_damping')
    wheel_collision_width = LaunchConfiguration('wheel_collision_width')
    drive_contact_mu = LaunchConfiguration('drive_contact_mu')
    caster_contact_mu = LaunchConfiguration('caster_contact_mu')
    drive_slip1 = LaunchConfiguration('drive_slip1')
    drive_slip2 = LaunchConfiguration('drive_slip2')
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
            'imu_rate_hz': imu_rate_hz,
            'magnetometer_rate_hz': magnetometer_rate_hz,
            'ground_truth_rate_hz': ground_truth_rate_hz,
            'color_width': color_width,
            'color_height': color_height,
            'color_fps': color_fps,
            'depth_width': depth_width,
            'depth_height': depth_height,
            'depth_fps': depth_fps,
            'enable_depth_camera': enable_depth_camera,
            'robot_visual_mode': robot_visual_mode,
            'wheel_joint_damping': wheel_joint_damping,
            'wheel_collision_width': wheel_collision_width,
            'drive_contact_mu': drive_contact_mu,
            'caster_contact_mu': caster_contact_mu,
            'drive_slip1': drive_slip1,
            'drive_slip2': drive_slip2,
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
        condition=IfCondition(enable_depth_registration),
    )

    return LaunchDescription([
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
        DeclareLaunchArgument('imu_rate_hz',
                              default_value=str(published_rates['imu_hz']),
                              description='Gazebo D435i IMU update rate in Hz.'),
        DeclareLaunchArgument('magnetometer_rate_hz',
                              default_value=str(published_rates['magnetometer_hz']),
                              description='Gazebo magnetic-field update rate in Hz.'),
        DeclareLaunchArgument('ground_truth_rate_hz',
                              default_value=str(published_rates['ground_truth_hz']),
                              description='Gazebo ground truth odometry rate in Hz.'),
        DeclareLaunchArgument(
            'enable_magnetometer',
            default_value='true',
            choices=['true', 'false'],
            description='Enable the simulated 5883L magnetometer on /imu/mag.',
        ),
        DeclareLaunchArgument('color_width', default_value=str(color_config['width']),
                              description='Gazebo RGB camera image width'),
        DeclareLaunchArgument('color_height', default_value=str(color_config['height']),
                              description='Gazebo RGB camera image height'),
        DeclareLaunchArgument('color_fps', default_value=str(color_config['fps']),
                              description='Requested Gazebo RGB FPS'),
        DeclareLaunchArgument('depth_width', default_value=str(depth_config['width']),
                              description='Gazebo depth camera image width'),
        DeclareLaunchArgument('depth_height', default_value=str(depth_config['height']),
                              description='Gazebo depth camera image height'),
        DeclareLaunchArgument('depth_fps', default_value=str(depth_config['fps']),
                              description='Requested Gazebo depth FPS'),
        DeclareLaunchArgument(
            'enable_depth_camera',
            default_value='true', choices=['true', 'false'],
            description='Keep the physical D435i depth sensor (false for lighting-only experiments).',
        ),
        DeclareLaunchArgument(
            'enable_depth_registration',
            default_value='true', choices=['true', 'false'],
            description='Start RGB-depth registration (false for isolated raw camera benchmarks).',
        ),
        DeclareLaunchArgument(
            'robot_visual_mode',
            default_value='full', choices=['full', 'simple', 'none'],
            description='Render full meshes, a lightweight proxy, or no robot visual; physics unchanged.',
        ),
        DeclareLaunchArgument(
            'wheel_joint_damping', default_value=str(physics['wheel_joint_damping']),
            description='Wheel joint damping N*m*s/rad; default keeps existing physics.',
        ),
        DeclareLaunchArgument(
            'wheel_collision_width', default_value=str(physics['wheel_collision_width']),
            description='Wheel collision width in meters; current simulation baseline 0.001 m.',
        ),
        DeclareLaunchArgument(
            'drive_contact_mu',
            default_value=str(physics['drive_contact_mu']),
            description='Gazebo drive-wheel contact friction coefficient.',
        ),
        DeclareLaunchArgument(
            'caster_contact_mu',
            default_value=str(physics['caster_contact_mu']),
            description='Gazebo passive-caster contact friction coefficient.',
        ),
        DeclareLaunchArgument('drive_slip1', default_value=str(physics['drive_slip1']),
                              description='Rolling tire contact slip in Gazebo.'),
        DeclareLaunchArgument('drive_slip2', default_value=str(physics['drive_slip2']),
                              description='Lateral tire contact slip in Gazebo.'),
        DeclareLaunchArgument(
            'controller_params',
            default_value=generated['controller_params'],
            description='Simulation-only controller YAML with published wheel odometry rate.',
        ),

        gazebo,
        delayed_spawn_robot,
        depth_to_color_registration,
    ])
