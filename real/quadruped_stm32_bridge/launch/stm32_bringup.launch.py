"""Controller link only. Does not start sensors, Nav2 or Mission automatically."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(context, key):
    value = LaunchConfiguration(key).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(key+' must be true or false')
    return value == 'true'


def _nodes(context):
    enabled, test = _boolean(context, 'enable_motion'), _boolean(context, 'test_mode')
    hurricane = _boolean(context, 'start_hurricane')
    calibrated = _boolean(context, 'calibration_confirmed')
    if enabled and test:
        raise ValueError('enable_motion=true requires deliberate test_mode=false')
    if enabled and not calibrated:
        raise ValueError('enable_motion=true requires measured calibration_confirmed=true')
    if hurricane and not enabled:
        raise ValueError('start_hurricane=true requires enabled, calibrated firmware integration')
    config = LaunchConfiguration('params_file').perform(context)
    port = LaunchConfiguration('port').perform(context)
    if enabled and not port:
        raise ValueError('explicit port required for enable_motion')
    arbiter = str(Path(get_package_share_directory('go2_mission_backends'))/'config/arbiter_real.yaml')
    actions = [LogInfo(msg='STM32 protocol v1 DRAFT; opening UART is not motor enable. Mission remains manual.'),
        Node(package='quadruped_stm32_bridge', executable='serial_gateway', name='serial_gateway',
            output='screen', parameters=[config, dict(port=port, enable_motion=enabled, test_mode=test, calibration_confirmed=calibrated, use_sim_time=False)]),
        Node(package='quadruped_stm32_bridge', executable='nav_velocity_adapter', name='nav_velocity_adapter',
            output='screen', parameters=[config, dict(use_sim_time=False)]),
        Node(package='quadruped_stm32_bridge', executable='telemetry_adapter', name='stm32_telemetry_adapter',
            output='screen', parameters=[config, dict(use_sim_time=False)]),
        Node(package='go2_mission_backends', executable='motion_arbiter', name='motion_arbiter',
            output='screen', parameters=[arbiter, dict(enable_output=enabled, simulation_mode=False, use_sim_time=False)])]
    if hurricane:
        actions.append(Node(package='robocon_mission', executable='hurricane_skill_server', name='hurricane_skill_server',
            output='screen', parameters=[config, dict(dry_run=False, use_sim_time=False,
                skills_file=LaunchConfiguration('skills_file').perform(context))]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('quadruped_stm32_bridge'))
    mission = Path(get_package_share_directory('robocon_mission'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share/'config/stm32.yaml')),
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('test_mode', default_value='true'),
        DeclareLaunchArgument('start_hurricane', default_value='false'),
        DeclareLaunchArgument('calibration_confirmed', default_value='false'),
        DeclareLaunchArgument('skills_file', default_value=str(mission/'config/hurricane_skills.yaml')),
        OpaqueFunction(function=_nodes)])
