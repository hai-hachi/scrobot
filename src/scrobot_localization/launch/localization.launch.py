import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    localization_pkg = get_package_share_directory('scrobot_localization')

    imu_filter_config = os.path.join(
        localization_pkg, 'config', 'imu_filter.yaml'
    )
    ekf_config = os.path.join(
        localization_pkg, 'config', 'ekf.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_magnetometer = LaunchConfiguration('use_magnetometer')
    use_sensor_qos_transformer = LaunchConfiguration(
        'use_sensor_qos_transformer'
    )

    # Simulation can use the stock imu_transformer. RealSense on physical
    # hardware publishes SensorDataQoS, so hardware bringup selects the local
    # QoS-compatible transformer instead.
    stock_imu_transformer = Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        name='imu_transformer',
        output='screen',
        condition=UnlessCondition(use_sensor_qos_transformer),
        parameters=[{
            'target_frame': 'base_link',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('imu_in', '/camera/camera/imu'),
            ('imu_out', '/imu/data_raw'),
        ],
    )

    sensor_qos_imu_transformer = Node(
        package='scrobot_localization',
        executable='imu_transformer_sensor_qos',
        name='imu_transformer_sensor_qos',
        output='screen',
        condition=IfCondition(use_sensor_qos_transformer),
        parameters=[{
            'target_frame': 'base_link',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('imu_in', '/camera/camera/imu'),
            ('imu_out', '/imu/data_raw'),
        ],
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[
            imu_filter_config,
            {
                'use_sim_time': use_sim_time,
                'use_mag': ParameterValue(
                    use_magnetometer, value_type=bool
                ),
            },
        ],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data', '/imu/data'),
            ('imu/mag', '/imu/mag'),
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'use_magnetometer',
            default_value='false',
            choices=['true', 'false'],
            description='Optional Madgwick magnetometer input; disabled on the current robot.',
        ),
        DeclareLaunchArgument(
            'use_sensor_qos_transformer',
            default_value='false',
            choices=['true', 'false'],
            description='Use the SensorDataQoS-compatible D435i IMU transformer on physical hardware.',
        ),
        stock_imu_transformer,
        sensor_qos_imu_transformer,
        imu_filter,
        ekf_node,
    ])
