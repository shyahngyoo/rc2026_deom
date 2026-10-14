"""Nav2 and map server only; requires sensors and map-to-base TF."""
import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    go2_share = get_package_share_directory("go2_nav_bringup")
    champ_navigation_share = go2_share
    nav2_share = get_package_share_directory("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav_rviz = LaunchConfiguration("show_rviz")
    nav_map = LaunchConfiguration("nav_map")
    params_file = LaunchConfiguration("nav_params_file")
    transit_bt = os.path.join(go2_share, "behavior_trees", "nav_transit.xml")
    nav_params = RewrittenYaml(source_file=params_file, param_rewrites={
        "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": transit_bt}, convert_types=True)
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": nav_map}],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_composition": "False",
            "params_file": nav_params,
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_rviz",
        output="screen",
        arguments=["-d", os.path.join(champ_navigation_share, "rviz", "navigation.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(nav_rviz),
    )

    readiness_script = os.path.join(
        go2_share, "launch", "wait_for_nav_ready.py"
    )

    def wait_for(stage, timeout):
        return ExecuteProcess(
            cmd=["/usr/bin/python3", readiness_script, stage, str(timeout)],
            output="screen",
        )

    # Map transitions use bounded requests and actual-state recovery. The
    # standard Humble manager can wait forever for a dropped change_state reply.
    map_activate = wait_for("map_activate", 60)
    map_tf_wait = wait_for("map_tf", 120)
    nav_action_wait = wait_for("nav_action", 120)

    def on_ready(event, actions, stage):
        if event.returncode == 0:
            return actions
        return [LogInfo(
            msg=f"{stage} readiness failed (exit {event.returncode}); "
                "dependent nodes will not start."
        )]

    return LaunchDescription([
        # A terminal sourced before this BT plugin was built can resolve this
        # package's launch files while still lacking its lib directory.
        AppendEnvironmentVariable(
            "LD_LIBRARY_PATH",
            os.path.join(get_package_prefix("go2_nav_bringup"), "lib"),
            prepend=True,
        ),
        DeclareLaunchArgument("nav_map", description="Required navigation map YAML"),
        DeclareLaunchArgument("nav_params_file", description="Required Nav2 parameter YAML"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("show_rviz", default_value="true"),
        DeclareLaunchArgument("startup_delay", default_value="0.0"),
        RegisterEventHandler(OnProcessExit(target_action=map_activate,
            on_exit=lambda event, context: on_ready(event, [map_tf_wait], "map_server activation"))),
        RegisterEventHandler(OnProcessExit(target_action=map_tf_wait,
            on_exit=lambda event, context: on_ready(event, [navigation, nav_action_wait], "map and TF"))),
        RegisterEventHandler(OnProcessExit(target_action=nav_action_wait,
            on_exit=lambda event, context: on_ready(
                event, [LogInfo(msg="Nav2 Action ready: starting RViz if enabled."), rviz], "Nav2 Action"))),
        TimerAction(period=LaunchConfiguration("startup_delay"), actions=[map_server, map_activate]),
    ])
