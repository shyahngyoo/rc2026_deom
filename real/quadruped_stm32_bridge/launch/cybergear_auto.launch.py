"""CyberGear-only entry: no v1 gateway, Nav2 or old Mission output."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('quadruped_stm32_bridge'))
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='', description='USB-TTL device path'),
        DeclareLaunchArgument('enable_motion', default_value='false', description='Allow key transmission'),
        DeclareLaunchArgument('plan_file', default_value='', description='CyberGear steps YAML; empty for manual commissioning'),
        DeclareLaunchArgument('params_file', default_value=str(share/'config/cybergear.yaml')),
        Node(package='quadruped_stm32_bridge', executable='cybergear_bridge', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                 'plan_file': ParameterValue(LaunchConfiguration('plan_file'), value_type=str),
                 'enable_motion': ParameterValue(LaunchConfiguration('enable_motion'), value_type=bool),
                 'use_sim_time': False}])])
