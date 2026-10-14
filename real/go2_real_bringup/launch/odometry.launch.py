"""Real FAST-LIO mapping/odometry; the sensor driver must already be running."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    own = get_package_share_directory("go2_real_bringup")
    algorithm = get_package_share_directory("go2_fastlio_localization")
    return LaunchDescription([
        DeclareLaunchArgument("show_rviz", default_value="true"),
        DeclareLaunchArgument("lio_params", default_value=os.path.join(own, "config", "fast_lio_mid360_real.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(algorithm, "launch", "fastlio.launch.py")),
            launch_arguments={
                "fastlio_params_file": LaunchConfiguration("lio_params"),
                "use_sim_time": "false", "node_name": "fastlio_mapping",
                "publish_map": "true", "save_pcd": "false",
            }.items(),
        ),
        Node(package="rviz2", executable="rviz2", name="real_mapping_rviz",
             arguments=["-d", os.path.join(own, "config", "real_mapping.rviz")],
             parameters=[{"use_sim_time": False}],
             condition=IfCondition(LaunchConfiguration("show_rviz"))),
    ])
