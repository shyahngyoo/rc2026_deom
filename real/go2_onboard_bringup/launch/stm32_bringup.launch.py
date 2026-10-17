"""Explicit onboard controller-link entry, independent of real sensor diagnostics."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = Path(get_package_share_directory('quadruped_stm32_bridge'))/'launch/stm32_bringup.launch.py'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))])
