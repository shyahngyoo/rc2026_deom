"""Deprecated compatibility alias; build/install go2_sim_bringup to use it.

This optional alias is not a reverse package dependency: the new bringup
depends on the algorithms here, never the other way around.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    share = get_package_share_directory("go2_sim_bringup")
    return LaunchDescription([
        LogInfo(msg="Compatibility entry: use ros2 launch go2_sim_bringup navigation.launch.py"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, "launch", "navigation.launch.py"))),
    ])
