"""Compatibility FAST-LIO entry with the historical simulation profile."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    own = get_package_share_directory("go2_fastlio_localization")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(own, "config", "fast_lio_mid360_sim.yaml")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(own, "launch", "fastlio.launch.py")),
            launch_arguments={"fastlio_params_file": LaunchConfiguration("params_file"),
                              "use_sim_time": LaunchConfiguration("use_sim_time")}.items()),
    ])
