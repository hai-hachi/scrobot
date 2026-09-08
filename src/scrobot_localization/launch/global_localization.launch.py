import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('scrobot_localization')
    params = os.path.join(pkg, 'config', 'court_landmarks.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Exactly one node owns /relocalize and map -> odom.
    global_localizer = Node(package='scrobot_localization', executable='tag_global_localizer.py', name='tag_global_localizer', output='screen', parameters=[params, {'use_sim_time': use_sim_time}])

    # Tag search / approach owns /approach_tag and /cmd_vel_relocalization.
    approach_controller = Node(package='scrobot_localization', executable='tag_approach_controller.py', name='tag_approach_controller', output='screen', parameters=[params, {'use_sim_time': use_sim_time}])

    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='true'), global_localizer, approach_controller])
