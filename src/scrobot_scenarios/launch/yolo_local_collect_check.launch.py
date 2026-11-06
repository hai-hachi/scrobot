"""Regression scenario entrypoint; existing debug implementation retained.

All launch arguments remain supported by the included legacy scenario.
During migration this avoids duplicating test implementations and artifacts.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    legacy = os.path.join(
        get_package_share_directory('scrobot_debug'),
        'launch', 'yolo_local_collect_check.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(legacy)),
    ])
