"""Real hardware only: Livox acquisition + local odometry + GICP. No Nav2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sensor = get_package_share_directory('go2_real_bringup')
    algorithm = get_package_share_directory('go2_fastlio_localization')
    own = get_package_share_directory('go2_real_localization')
    return LaunchDescription([
        DeclareLaunchArgument('map_path', description='Required real-room PCD absolute path'),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(own, 'config', 'gicp.yaml')),
        DeclareLaunchArgument('lio_params', default_value=os.path.join(sensor, 'config', 'fast_lio_mid360_real.yaml')),
        DeclareLaunchArgument('user_config_path', default_value=os.path.join(sensor, 'config', 'mid360_real.json')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sensor, 'launch', 'lidar.launch.py')),
                                 launch_arguments={'user_config_path': LaunchConfiguration('user_config_path'),
                                                   'xfer_format': '1'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(algorithm, 'launch', 'fastlio.launch.py')),
            launch_arguments={
                'fastlio_params_file': LaunchConfiguration('lio_params'),
                'use_sim_time': 'false', 'node_name': 'fastlio_mapping',
                'publish_map': 'false', 'save_pcd': 'false',
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(own, 'launch', 'localization.launch.py')),
                                 launch_arguments={'map_path': LaunchConfiguration('map_path'),
                                                   'params_file': LaunchConfiguration('params_file'),
                                                   'show_rviz': LaunchConfiguration('show_rviz'),
                                                   'use_sim_time': 'false'}.items()),
    ])
