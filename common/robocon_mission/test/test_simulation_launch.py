import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, SetLaunchConfiguration)
from launch.events.process import ProcessExited
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
import yaml
from launch_ros.actions import Node
from robocon_mission.route_manager import RouteManager


def launch_description():
    path = Path(get_package_share_directory("go2_sim_bringup")) / "launch" / "mission.launch.py"
    spec = importlib.util.spec_from_file_location("patrol_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def ready_exit_actions(description, context, returncode):
    process = next(action for action in description.entities
                   if type(action) is ExecuteProcess)
    handler = next(action.event_handler for action in description.entities
                   if isinstance(action, RegisterEventHandler))
    event = ProcessExited(action=process, name="nav_readiness", cmd=[],
                          cwd=None, env=None, pid=0, returncode=returncode)
    assert handler.matches(event)
    return handler.handle(event, context)


def test_patrol_controller_can_finish_yaw_at_transit_tolerance():
    context = LaunchContext()
    context.launch_configurations["nav_params_file"] = str(
        Path(get_package_share_directory("go2_nav_bringup")) /
        "config" / "autonomy" / "fast_lio_nav2.yaml")
    navigation = next(action for action in launch_description().entities
                      if isinstance(action, IncludeLaunchDescription))
    configured = dict(navigation.launch_arguments)["params_file"]
    path = Path(configured.perform(context))
    config = yaml.safe_load(path.read_text())
    controller = config["controller_server"]["ros__parameters"]
    assert controller["FollowPath"]["xy_goal_tolerance"] == \
        controller["transit_goal_checker"]["xy_goal_tolerance"]
    approach = controller["FollowPathApproach"]
    assert approach["PrecisionPose.class"] == "go2_nav_bringup::PrecisionPoseCritic"
    assert approach["min_vel_x"] < 0.0 and approach["min_vel_y"] < 0.0
    assert approach["max_vel_y"] > 0.0
    assert approach["PrecisionPose.max_xy_speed"] <= approach["max_speed_xy"]
    assert approach["PrecisionPose.max_yaw_speed"] <= approach["max_vel_theta"]
    assert "BaseObstacle" in approach["critics"] and "RotateToGoal" not in approach["critics"]
    assert config["velocity_smoother"]["ros__parameters"]["max_velocity"][1] >= approach["max_vel_y"]
    assert config["velocity_smoother"]["ros__parameters"]["min_velocity"][1] <= approach["min_vel_y"]
    assert controller["approach_goal_checker"]["xy_goal_tolerance"] < 0.10
    assert controller["approach_goal_checker"]["yaw_goal_tolerance"] < 0.10
    mission_config = yaml.safe_load((Path(get_package_share_directory("robocon_mission")) /
                                    "config" / "mission_params.yaml").read_text())
    mission = mission_config["mission_manager"]["ros__parameters"]
    assert mission["wait_at_waypoints"] and mission["precise_waypoints"]
    assert controller["approach_goal_checker"]["xy_goal_tolerance"] < mission["waypoint_xy_tolerance"]
    assert controller["approach_goal_checker"]["yaw_goal_tolerance"] < mission["waypoint_yaw_tolerance"]
    assert isinstance(mission["max_waypoint_alignment_retries"], int)
    assert 0 <= mission["max_waypoint_alignment_retries"] <= 1
    assert controller["approach_goal_checker"]["stateful"] is False
    assert controller["approach_goal_checker"]["plugin"] == "nav2_controller::StoppedGoalChecker"
    assert controller["approach_goal_checker"]["rot_stopped_velocity"] == 0.05
    assert config["planner_server"]["ros__parameters"]["GridBasedApproach"]["tolerance"] == 0.


def test_manual_mission_start_survives_nav2_autostart_override():
    context = LaunchContext()
    context.launch_configurations["autostart"] = "false"
    description = launch_description()
    capture = next(action for action in description.entities
                   if isinstance(action, SetLaunchConfiguration))
    capture.execute(context)
    # Nested Nav2 startup writes autostart=true to the launch context.
    context.launch_configurations["autostart"] = "true"
    mission = next(action for action in ready_exit_actions(description, context, 0)
                   if isinstance(action, IncludeLaunchDescription))
    autostart = dict(mission.launch_arguments)["autostart"]
    assert perform_substitutions(context, [autostart]) == "false"


def test_nav_readiness_failure_never_starts_mission():
    actions = ready_exit_actions(launch_description(), LaunchContext(), 1)
    assert not any(isinstance(action, IncludeLaunchDescription) for action in actions)


def test_default_launch_uses_state_only_actions_and_real_localization():
    context = LaunchContext()
    description = launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = [action for action in description.entities if isinstance(action, Node)]
    executables = [perform_substitutions(context, normalize_to_list_of_substitutions(
        action.node_executable)) for action in nodes]
    assert executables.count("mock_obstacle_skill") == 1
    assert executables.count("mock_motion_services") == 1
    assert executables.count("localization_health") == 1
    assert "mock_localization_ok" not in executables
    mission = next(action for action in ready_exit_actions(description, context, 0)
                   if isinstance(action, IncludeLaunchDescription))
    arguments = dict(mission.launch_arguments)
    assert arguments["start_mock_servers"] == "false"  # no duplicate mocks
    assert arguments["accepted_pose_topic"] == "/localization/pose"
    assert Path(context.launch_configurations["route_file"]).name == "patrol_mock_actions.yaml"


def test_state_only_route_preserves_patrol_geometry_and_checks_three_exits():
    directory = Path(get_package_share_directory("go2_sim_bringup")) / "config"
    old = RouteManager.load(directory / "patrol_route.yaml")
    new = RouteManager.load(directory / "patrol_mock_actions.yaml")
    assert len(new.tasks) == len(old.tasks) == 29
    assert new.return_pose == old.return_pose
    obstacles = []
    for before, after in zip(old.tasks, new.tasks):
        assert after.id == before.id
        if after.type == "obstacle":
            obstacles.append(after.id)
            assert after.transit_pose == after.approach_pose == after.end_pose == before.pose
            assert after.skill_id == "mock_state_only"
            assert after.end_xy_tolerance == .15 and after.end_yaw_tolerance == .20
        else:
            assert after.pose == before.pose
    assert obstacles == ["west_entry", "south_west", "north_center"]
