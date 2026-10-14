"""Start only the independent Mission Manager and optional test adapters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("robocon_mission")
    route = LaunchConfiguration("route_file")
    params = LaunchConfiguration("params_file")
    simulation = LaunchConfiguration("simulation_mode")
    mocks = LaunchConfiguration("start_mock_servers")
    mock_condition = IfCondition(PythonExpression([
        "'", simulation, "'.lower() == 'true' and '", mocks,
        "'.lower() == 'true'",
    ]))
    return LaunchDescription([
        DeclareLaunchArgument("route_file", default_value=os.path.join(
            share, "config", "competition_route.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            share, "config", "mission_params.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("simulation_mode", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument("startup_check_service", default_value=""),
        DeclareLaunchArgument("start_mock_servers", default_value="false"),
        DeclareLaunchArgument("accepted_pose_topic", default_value=""),
        DeclareLaunchArgument("accepted_pose_type", default_value="covariance"),
        DeclareLaunchArgument("accepted_pose_body_frame", default_value="livox_frame"),
        DeclareLaunchArgument("accepted_pose_source_id", default_value=""),
        DeclareLaunchArgument("mission_time_limit", default_value="180.0"),
        Node(package="robocon_mission", executable="mission_manager",
             name="mission_manager", output="screen", parameters=[params, {
                 "route_file": route,
                 "accepted_pose_topic": ParameterValue(
                     LaunchConfiguration("accepted_pose_topic"), value_type=str),
                 "accepted_pose_type": ParameterValue(
                     LaunchConfiguration("accepted_pose_type"), value_type=str),
                 "accepted_pose_body_frame": ParameterValue(
                     LaunchConfiguration("accepted_pose_body_frame"), value_type=str),
                 "accepted_pose_source_id": ParameterValue(
                     LaunchConfiguration("accepted_pose_source_id"), value_type=str),
                 "use_sim_time": ParameterValue(
                     LaunchConfiguration("use_sim_time"), value_type=bool),
                 "simulation_mode": ParameterValue(simulation, value_type=bool),
                 "startup_check_service": ParameterValue(LaunchConfiguration("startup_check_service"), value_type=str),
                  "autostart": ParameterValue(LaunchConfiguration("autostart"),
                                               value_type=bool),
                 "mission_time_limit": ParameterValue(
                     LaunchConfiguration("mission_time_limit"), value_type=float),
             }]),
        Node(package="go2_nav_bringup", executable="mock_obstacle_skill",
             name="mock_obstacle_skill", output="screen",
             condition=mock_condition),
        Node(package="robocon_mission", executable="mock_motion_services",
             name="mock_motion_services", output="screen", condition=mock_condition,
             parameters=[{"simulation_mode": True,
                          "use_sim_time": ParameterValue(
                              LaunchConfiguration("use_sim_time"), value_type=bool)}]),
        Node(package="robocon_mission", executable="mock_localization_ok",
             name="mock_localization_ok", output="screen",
             condition=IfCondition(PythonExpression([
                 "'", simulation, "'.lower() == 'true' and '", mocks,
                 "'.lower() == 'true'",
             ]))),
    ])
