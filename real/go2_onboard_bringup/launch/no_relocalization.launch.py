"""Explicit odometry-only entry point; reject an accidental GICP profile."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from go2_onboard_bringup.profile import DEFAULT_CONFIG, load_profile
from go2_onboard_bringup.launching import onboard_actions


def prepare(context):
    folder = LaunchConfiguration('config_dir').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    profile, _ = load_profile(folder, mode)
    if profile.get('localization_source') != 'odometry':
        raise ValueError('no_relocalization requires --localization-source odometry profile')
    return onboard_actions(folder, mode)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=DEFAULT_CONFIG),
        DeclareLaunchArgument('mode', default_value='navigation', choices=['localization', 'navigation']),
        OpaqueFunction(function=prepare),
    ])
