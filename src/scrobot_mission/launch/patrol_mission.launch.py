import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('scrobot_mission')
    params = os.path.join(pkg, 'config', 'patrol_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    patrol_points = Node(package='scrobot_mission', executable='patrol_points', name='patrol_points', output='screen', parameters=[params, {'use_sim_time': use_sim_time}])
    patrol_manager = Node(package='scrobot_mission', executable='patrol_manager', name='patrol_manager', output='screen', parameters=[params, {'use_sim_time': use_sim_time}])

    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='true'), patrol_points, patrol_manager])
