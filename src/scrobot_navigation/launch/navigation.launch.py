import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('scrobot_navigation')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    controller_server = Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[params, {'use_sim_time': use_sim_time}], remappings=[('cmd_vel', '/cmd_vel_nav')])
    planner_server = Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[params, {'use_sim_time': use_sim_time}])
    behavior_server = Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[params, {'use_sim_time': use_sim_time}], remappings=[('cmd_vel', '/cmd_vel_nav')])
    bt_navigator = Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[params, {'use_sim_time': use_sim_time}])

    # Important: mission manager starts Nav2 only after initial map -> odom exists.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': False,
            'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
