"""Onboard real inputs only. This entry point never authorizes robot motion."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from go2_onboard_bringup.profile import DEFAULT_CONFIG, MODES
from go2_onboard_bringup.launching import onboard_actions


def prepare(context):
    config_dir=LaunchConfiguration('config_dir').perform(context)
    mode=LaunchConfiguration('mode').perform(context)
    return onboard_actions(config_dir,mode)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir',default_value=DEFAULT_CONFIG,description='External measured configure_robot profile'),
        DeclareLaunchArgument('mode',default_value='localization',choices=list(MODES),
            description='navigation means isolated planning diagnostics, not motor control'),
        OpaqueFunction(function=prepare)])
