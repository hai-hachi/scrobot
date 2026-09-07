import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_pkg = get_package_share_directory('scrobot_mission')
    config = os.path.join(mission_pkg, 'config', 'patrol_params.yaml')

    patrol_points = Node(package='scrobot_mission', executable='patrol_points', name='patrol_points', parameters=[config, {'use_sim_time': True}], output='screen')
    patrol_manager = Node(package='scrobot_mission', executable='patrol_manager', name='patrol_manager', parameters=[config, {'use_sim_time': True}], output='screen')

    return LaunchDescription([patrol_points, patrol_manager])
