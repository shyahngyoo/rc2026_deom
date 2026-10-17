"""Only the Hurricane skill adapter; it never starts another Nav2 or controller."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    profile = str(Path(get_package_share_directory("robocon_mission")) /
                  "config/hurricane_skills.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("skills_file", default_value=profile),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="robocon_mission", executable="hurricane_skill_server",
             name="hurricane_skill_server", output="screen", parameters=[{
                 "skills_file": LaunchConfiguration("skills_file"),
                 "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
             }]),
    ])
