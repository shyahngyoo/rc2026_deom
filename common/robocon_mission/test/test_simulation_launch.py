"""Five retained contracts migrated from a non-shipped Gazebo wrapper.

This historical test filename remains for traceability. No simulator package
is needed: inspect the real Mission launch and the preserved route fixture.
"""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from robocon_mission.route_manager import RouteManager
import yaml


def launch_description():
    path = Path(get_package_share_directory("robocon_mission")) / "launch" / "mission.launch.py"
    spec = importlib.util.spec_from_file_location("real_mission_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def resolved(overrides=None):
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    description = launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = [action for action in description.entities if isinstance(action, Node)]
    manager = next(action for action in nodes if perform_substitutions(context,
        normalize_to_list_of_substitutions(action.node_executable)) == "mission_manager")
    values = evaluate_parameters(context, manager._Node__parameters)
    parameters = next(v for v in values if isinstance(v, dict))
    return description, context, nodes, parameters


def test_patrol_controller_can_finish_yaw_at_transit_tolerance():
    # Keep every original precision/stop/rotation assertion, using the real
    # onboard profile instead of the removed, external Gazebo wrapper.
    path = Path(get_package_share_directory("go2_onboard_bringup")) / "config" / "nav2.template.yaml"
    config = yaml.safe_load(path.read_text())
    controller = config["controller_server"]["ros__parameters"]
    assert controller["FollowPath"]["xy_goal_tolerance"] == \
        controller["transit_goal_checker"]["xy_goal_tolerance"]
    approach = controller["FollowPathApproach"]
    assert approach["PrecisionPose.class"] == "go2_nav_bringup::PrecisionPoseCritic"
    # Hardware contract changed: the custom STM32 quadruped cannot strafe.
    assert approach["min_vel_x"] < 0.0
    assert approach["min_vel_y"] == approach["max_vel_y"] == 0.0
    assert approach["acc_lim_y"] == approach["decel_lim_y"] == 0.0
    assert approach["vy_samples"] == 1 and approach["PrecisionPose.holonomic"] is False
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


def test_manual_mission_start_defaults_false_and_explicit_override_is_preserved():
    _, _, _, defaults = resolved()
    assert defaults["autostart"] is False
    _, _, _, explicit = resolved({"autostart": "true"})
    assert explicit["autostart"] is True  # Caller capability retained; no automatic caller exists.


def test_real_mission_forwards_required_preflight_and_atomic_localization():
    _, _, _, parameters = resolved({
        "startup_check_service": "/real/preflight", "accepted_pose_topic": "/localization/evidence",
        "accepted_pose_type": "evidence", "accepted_pose_body_frame": "livox_frame",
        "accepted_pose_source_id": "measured_source", "mission_time_limit": "90.0"})
    assert parameters["startup_check_service"] == "/real/preflight"
    assert parameters["accepted_pose_topic"] == "/localization/evidence"
    assert parameters["accepted_pose_type"] == "evidence"
    assert parameters["accepted_pose_body_frame"] == "livox_frame"
    assert parameters["accepted_pose_source_id"] == "measured_source"
    assert parameters["mission_time_limit"] == 90.0


def test_default_launch_has_only_real_manager_and_no_simulator_include():
    description, context, nodes, parameters = resolved()
    assert len(nodes) == 1
    assert perform_substitutions(context, normalize_to_list_of_substitutions(nodes[0].node_executable)) == "mission_manager"
    assert not any(isinstance(a, IncludeLaunchDescription) for a in description.entities)
    assert "start_mock_servers" not in context.launch_configurations
    assert parameters["use_sim_time"] is False
    assert parameters["simulation_mode"] is False
    assert parameters["autostart"] is False


def test_retained_patrol_fixture_preserves_geometry_and_checks_three_exits():
    path = Path(__file__).resolve().parent / "fixtures" / "patrol_mock_actions.yaml"
    raw = yaml.safe_load(path.read_text())
    route = RouteManager.load(path)
    assert len(route.tasks) == len(raw["route"]) == 29
    assert route.return_pose == tuple(raw["return"]["pose"])
    obstacles = []
    for before, after in zip(raw["route"], route.tasks):
        assert after.id == before["id"]
        if after.type == "obstacle":
            obstacles.append(after.id)
            assert after.transit_pose == after.approach_pose == after.end_pose == tuple(before["transit_pose"])
            assert after.skill_id == "mock_state_only"
            assert after.end_xy_tolerance == .15 and after.end_yaw_tolerance == .20
        else:
            assert after.pose == tuple(before["pose"])
    assert obstacles == ["west_entry", "south_west", "north_center"]
