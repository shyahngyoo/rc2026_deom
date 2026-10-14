"""FAST-LIO only: caller supplies the sensor profile and clock."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    value = LaunchConfiguration
    boolean = lambda name: ParameterValue(value(name), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("fastlio_params_file", description="Required FAST-LIO sensor YAML"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("node_name", default_value="fastlio_odometry"),
        DeclareLaunchArgument("publish_map", default_value="false"),
        DeclareLaunchArgument("save_pcd", default_value="false"),
        DeclareLaunchArgument("map_output", default_value=""),
        Node(package="fast_lio", executable="fastlio_mapping", name=value("node_name"),
             output="screen", parameters=[value("fastlio_params_file"), {
                 "use_sim_time": boolean("use_sim_time"),
                 "publish.map_en": boolean("publish_map"),
                 "publish.path_en": boolean("publish_map"),
                 "publish.effect_map_en": False,
                 "pcd_save.pcd_save_en": boolean("save_pcd"),
                 "map_file_path": value("map_output"),
             }]),
    ])
