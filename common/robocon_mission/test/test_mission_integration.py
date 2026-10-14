"""Real rclpy Action/TF/service integration against simulation-only servers."""

import json
import importlib.util
import math
import multiprocessing
import os
import subprocess
import threading
import time
import traceback
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from go2_obstacle_interfaces.action import ExecuteObstacle
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from robocon_mission.mission_fsm import State
from robocon_mission.mission_manager import MissionManager
from robocon_mission.mock_motion_services import MockMotionServices


ROUTE = """start_area: demo
frame_id: map
route:
  - id: first
    type: navigation
    pose: [0.0, 0.0, 0.0]
    max_retries: 1
  - id: obstacle
    type: obstacle
    transit_pose: [0.0, 0.0, 0.0]
    approach_pose: [0.0, 0.0, 0.0]
    skill_id: mock
    end_pose: [0.0, 0.0, 0.0]
    verify_timeout: 0.8
  - id: last
    type: navigation
    pose: [0.0, 0.0, 0.0]
return:
  pose: [0.0, 0.0, 0.0]
"""


# Probe ROS discovery and request/response across real processes before the
# three tests that depend on the separately launched mock skill Action server.
_TRIGGER_SERVER = """
import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

rclpy.init()
node = Node("mission_ipc_probe_server")
def respond(_request, response):
    response.success = True
    response.message = "ready"
    return response
service = node.create_service(Trigger, sys.argv[1], respond)
print("Trigger server created", flush=True)
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
"""

_TRIGGER_CLIENT = """
import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

rclpy.init()
node = Node("mission_ipc_probe_client")
try:
    client = node.create_client(Trigger, sys.argv[1])
    if not client.wait_for_service(timeout_sec=25.0):
        raise RuntimeError("Trigger server not discovered")
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None:
        raise RuntimeError("Trigger response timed out")
    response = future.result()
    if not response.success or response.message != "ready":
        raise RuntimeError("unexpected Trigger response")
finally:
    node.destroy_node()
    rclpy.shutdown()
"""


@pytest.fixture(scope="module")
def cross_process_trigger_ready(tmp_path_factory):
    """Skip only external mock Action tests when ROS IPC itself is unavailable."""
    directory = tmp_path_factory.mktemp("mission_ipc_probe")
    log_path = directory / "trigger_server.log"
    service_name = f"/mission/test_ipc_{os.getpid()}_{time.monotonic_ns()}"
    reason = None
    server_log = ""
    with log_path.open("w", encoding="utf-8") as log:
        server = subprocess.Popen(
            ["/usr/bin/python3", "-u", "-c", _TRIGGER_SERVER, service_name],
            stdout=log, stderr=subprocess.STDOUT,
        )
        try:
            try:
                probe = subprocess.run(
                    ["/usr/bin/python3", "-u", "-c", _TRIGGER_CLIENT,
                     service_name],
                    capture_output=True, text=True, timeout=33.0,
                    check=False,
                )
            except subprocess.TimeoutExpired:
                reason = "Trigger client process timed out"
            else:
                if probe.returncode != 0:
                    reason = (probe.stderr or probe.stdout).strip()[-1200:]
        finally:
            # Read diagnostics before SIGTERM produces shutdown traceback noise.
            server_log = log_path.read_text(encoding="utf-8")[-1200:].strip()
            if server.poll() is None:
                server.terminate()
            try:
                server.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=3.0)
    if reason is not None:
        message = (
            "cross-process ROS Trigger request/response unavailable: "
            f"{reason}; server log: {server_log or '(empty)'}"
        )
        if os.environ.get("ROBOCON_REQUIRE_ROS_IPC") == "1":
            pytest.fail(message)
        pytest.skip(message)


class FakeEnvironment(Node):
    def __init__(self, skill_mode="success", nav_mode="success",
                 health_fail_after=None, moving=False,
                 skill_server_enabled=True, use_sim_time=False):
        super().__init__("mission_fake_environment")
        self.skill_mode = skill_mode
        self.pose = (0.0, 0.0, 0.0)
        self.skill_end_pose = None
        self.health_failure_active = False
        self.nav_mode = nav_mode
        self.health_fail_after = health_fail_after
        self.moving = False
        self.moving_after_approach = moving
        self.nav_goals = []
        self.moving_after_nav = None
        self.freeze_accepted_after_nav = None
        self.follow_nav_goals = False
        self.nav_cancel_requests = 0
        self.skill_goals = []
        self.health_calls = 0
        self.statuses = []
        self.simulate_clock = use_sim_time
        self.clock_paused = False
        self.clock_start_wall = time.monotonic()
        self.latest_stamp = self.get_clock().now().to_msg()
        self.control_node = Node("mission_fake_motion_services")
        self.control = MockMotionServices(self.control_node)
        # The standalone mock arbiter stamps its own heartbeat independently
        # of sensor callbacks. Paused simulated time remains paused here too.
        self.control.stamp_provider = lambda: (
            self.latest_stamp if self.simulate_clock
            else self.control_node.get_clock().now().to_msg())
        self.clock_pub = (self.create_publisher(Clock, "/clock", 10)
                          if use_sim_time else None)
        group = ReentrantCallbackGroup()
        self.nav_server = ActionServer(
            self, NavigateToPose, "/navigate_to_pose", self._navigate,
            cancel_callback=self._cancel_navigation,
            callback_group=group,
        )
        self.skill_server = (ActionServer(
            self, ExecuteObstacle, "/execute_obstacle", self._skill,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        ) if skill_server_enabled else None)
        self.health_server = self.create_service(
            Trigger, "/mission/localization_ok", self._health,
            callback_group=group,
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.accepted_pub = self.create_publisher(PoseWithCovarianceStamped, "/test/accepted_pose", 10)
        self.freeze_accepted_after_approach = False
        self.accepted_frozen = False
        self.status_sub = self.create_subscription(
            String, "/mission/status", lambda msg: self.statuses.append(
                json.loads(msg.data)), 10
        )
        self.tf_pub = TransformBroadcaster(self)
        self.tick_timer = self.create_timer(0.05, self._publish_sensors,
                                            callback_group=group)

    def _publish_sensors(self):
        if self.clock_paused:
            return
        stamp = self.get_clock().now().to_msg()
        if self.simulate_clock:
            # This environment uses a wall timer, independent of the mission's
            # ROS clock, so the test can stop /clock like pausing Gazebo.
            sim_ns = int((10.0 + time.monotonic() - self.clock_start_wall) * 1e9)
            stamp.sec, stamp.nanosec = divmod(sim_ns, 1000000000)
            self.clock_pub.publish(Clock(clock=stamp))
        transform = TransformStamped()
        self.latest_stamp = stamp
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.pose[0]
        transform.transform.translation.y = self.pose[1]
        transform.transform.rotation.z = math.sin(self.pose[2] / 2)
        transform.transform.rotation.w = math.cos(self.pose[2] / 2)
        self.tf_pub.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.1 if self.moving else 0.0
        self.odom_pub.publish(odom)
        if not self.accepted_frozen:
            accepted = PoseWithCovarianceStamped()
            accepted.header.stamp, accepted.header.frame_id = stamp, "map"
            accepted.pose.pose.position.x, accepted.pose.pose.position.y = self.pose[:2]
            accepted.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
            accepted.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
            self.accepted_pub.publish(accepted)

    def _health(self, _request, response):
        self.health_calls += 1
        response.success = not self.health_failure_active
        response.message = "simulated health"
        return response

    def _navigate(self, handle):
        self.nav_goals.append(handle.request.behavior_tree)
        if self.follow_nav_goals:
            pose = handle.request.pose.pose
            q = pose.orientation
            self.pose = (pose.position.x, pose.position.y,
                         math.atan2(2*q.w*q.z, 1-2*q.z*q.z))
        if len(self.nav_goals) == self.moving_after_nav:
            self.moving = True
        if len(self.nav_goals) == self.freeze_accepted_after_nav:
            self.accepted_frozen = True
        if "nav_approach.xml" in handle.request.behavior_tree:
            self.moving = self.moving_after_approach
            self.accepted_frozen = self.freeze_accepted_after_approach
        if self.nav_mode == "wait_cancel":
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    handle.canceled()
                    return NavigateToPose.Result()
                time.sleep(0.02)
            handle.abort()
            return NavigateToPose.Result()
        if self.nav_mode == "hang":
            time.sleep(1.5)  # deliberately ignore cancel past its grace period
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
        if self.nav_mode == "fail_once" and len(self.nav_goals) == 1:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    def _cancel_navigation(self, _handle):
        self.nav_cancel_requests += 1
        return CancelResponse.ACCEPT

    def _skill(self, handle):
        self.skill_goals.append((handle.request.obstacle_id,
                                 handle.request.skill_id))
        if self.skill_mode == "hang":
            time.sleep(1.5)  # fault injection: ignore cancellation past the grace period
        if self.skill_mode == "slow":
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    handle.canceled()
                    return ExecuteObstacle.Result(
                        completed=False, control_released=True,
                        message="canceled")
                time.sleep(0.05)
        result = ExecuteObstacle.Result()
        result.control_released = self.skill_mode != "hold"
        result.completed = self.skill_mode != "fail"
        result.safe_stance = self.skill_mode != "unstable"
        result.message = "simulated"
        if self.skill_mode == "motion_unsafe":
            self.control.safe_stance = False
        if self.skill_mode == "release_fail":
            self.control.deny_owner = "SAFE_HOLD"
        if self.skill_mode == "restore_fail":
            self.control.deny_owner = "NAVIGATION"
        if self.skill_end_pose is not None:
            self.pose = self.skill_end_pose
        if self.health_fail_after is not None:
            self.health_failure_active = True
        if self.skill_mode == "fail":
            handle.abort()
        else:
            handle.succeed()
        return result


def _run_scenario_inner(tmp_path, skill_mode="success", nav_mode="success",
                        health_fail_after=None, navigation_timeout=2.0,
                        cancel_timeout=0.3, skill_timeout=2.0,
                        stop_timeout=2.0, moving=False,
                        existing_mock=False, server_timeout=1.0,
                        use_sim_time=False, pause_at=None,
                        localization_timeout=2.0, clock_timeout=1.0,
                        localization_loss_grace=1.0, end_pose=None,
                        skill_end_pose=None, relocalize_success=False,
                        deny_owner="", actual_pose=None, mission_time_limit=180.0,
                        simulation_mode=True, accepted_pose_topic="",
                        freeze_accepted_after_approach=False,
                        moving_after_nav=None, freeze_accepted_after_nav=None,
                        inprocess_mock=False, route_path=None, follow_nav_goals=False,
                        precise_waypoints=False):
    path = tmp_path / "route.yaml"
    route_text = Path(route_path).read_text() if route_path else ROUTE
    if end_pose is not None:
        route_text = route_text.replace("end_pose: [0.0, 0.0, 0.0]",
                                       f"end_pose: {list(end_pose)}")
    path.write_text(route_text, encoding="utf-8")
    mock_process = None
    mock_log = None
    if existing_mock:
        executable = (Path(get_package_prefix("go2_nav_bringup")) / "lib" /
                      "go2_nav_bringup" / "mock_obstacle_skill")
        args = [str(executable), "--duration", "0.1"]
        if skill_mode == "fail":
            args.append("--fail")
        elif skill_mode == "hold":
            args.append("--hold-control")
        mock_log = (tmp_path / "mock_skill.log").open("w", encoding="utf-8")
        mock_process = subprocess.Popen(["/usr/bin/python3", *args],
                                        stdout=mock_log, stderr=subprocess.STDOUT)
        time.sleep(2.0)
        assert mock_process.poll() is None, "existing mock server exited"
    rclpy.init()
    fake = FakeEnvironment(skill_mode, nav_mode, health_fail_after, moving,
                           skill_server_enabled=not (existing_mock or inprocess_mock),
                           use_sim_time=use_sim_time)
    fake.skill_end_pose = skill_end_pose
    fake.freeze_accepted_after_approach = freeze_accepted_after_approach
    fake.moving_after_nav = moving_after_nav
    fake.freeze_accepted_after_nav = freeze_accepted_after_nav
    fake.follow_nav_goals = follow_nav_goals
    fake.control.deny_owner = deny_owner
    fake.control.relocalize_success = relocalize_success
    fake.control.on_relocalize = lambda: setattr(fake, "health_failure_active", False)
    if actual_pose is not None:
        fake.pose = actual_pose
    if pause_at == State.IDLE:
        fake.clock_paused = True
    parameters = [Parameter(name, value=value) for name, value in {
        "route_file": str(path), "simulation_mode": simulation_mode, "autostart": True,
        "precise_waypoints": precise_waypoints,
        "use_sim_time": use_sim_time, "stable_duration": 0.15,
        "navigation_timeout": navigation_timeout,
        "approach_timeout": navigation_timeout,
        "skill_timeout": skill_timeout,
        "cancel_timeout": cancel_timeout,
        "server_timeout": server_timeout,
        "localization_timeout": localization_timeout,
        "clock_timeout": clock_timeout,
        "localization_loss_grace": localization_loss_grace,
        "stop_timeout": stop_timeout,
        "handoff_timeout": 1.0, "skill_safe_timeout": 0.7,
        "relocalize_timeout": 0.6, "max_relocalization_retries": 1,
        "verify_stable_duration": 0.15, "mission_time_limit": mission_time_limit,
        "accepted_pose_topic": accepted_pose_topic,
    }.items()]
    mission = MissionManager(parameter_overrides=parameters)
    # Match mission_manager: its FSM/adapters run serially. Mock servers have
    # a separate Executor so injected blocking Actions cannot starve Mission.
    executor = MultiThreadedExecutor(num_threads=3)
    mission_executor = SingleThreadedExecutor()
    control_executor = SingleThreadedExecutor()
    executor.add_node(fake)
    mission_executor.add_node(mission)
    control_executor.add_node(fake.control_node)
    mock_node = None
    if inprocess_mock:
        executable = (Path(get_package_prefix("go2_nav_bringup")) / "lib" /
                      "go2_nav_bringup" / "mock_obstacle_skill")
        loader = importlib.machinery.SourceFileLoader("tested_mock_skill", str(executable))
        spec = importlib.util.spec_from_loader(loader.name, loader)
        module = importlib.util.module_from_spec(spec)
        loader.exec_module(module)
        mock_node = module.MockObstacleSkill(0.3, skill_mode == "fail", skill_mode == "hold")
        executor.add_node(mock_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    mission_thread = threading.Thread(target=mission_executor.spin, daemon=True)
    control_thread = threading.Thread(target=control_executor.spin, daemon=True)
    thread.start()
    mission_thread.start()
    control_thread.start()
    try:
        paused_ros_ns = 0 if fake.clock_paused else None
        deadline = time.monotonic() + (38.0 if existing_mock or route_path else 9.0)
        while time.monotonic() < deadline and mission.fsm.state not in (
                State.FINISHED, State.ABORTED):
            if pause_at is not None and not fake.clock_paused and \
                    mission.fsm.state == pause_at:
                runner = (mission.obstacle if pause_at == State.EXECUTING_SKILL
                          else mission.navigation)
                if runner.active and runner.active["stage"] == "running":
                    fake.clock_paused = True
                    # Let any already queued /clock message be processed.
                    time.sleep(0.15)
                    paused_ros_ns = mission.get_clock().now().nanoseconds
            time.sleep(0.03)
        assert mission.fsm.state in (State.FINISHED, State.ABORTED), (
            mission.fsm.state, mission.fsm.error
        )
        time.sleep(0.1)  # allow the final status publication to reach the subscriber
        if pause_at is not None:
            assert paused_ros_ns is not None, "clock was never paused"
            assert mission.get_clock().now().nanoseconds == paused_ros_ns
            assert not mission.localization.tf_fresh()
            if pause_at == State.NAVIGATING:
                assert fake.nav_cancel_requests == 1
            # Require a periodic terminal status even with ROS time frozen;
            # an immediate _dispatch publication alone must not satisfy this.
            status_count = len(fake.statuses)
            time.sleep(0.65)
            assert len(fake.statuses) > status_count
            assert fake.statuses[-1]["state"] == mission.fsm.state.value
            assert mission.get_clock().now().nanoseconds == paused_ros_ns
        (tmp_path / "scenario_result.json").write_text(json.dumps({
            "state": mission.fsm.state.value, "error": mission.fsm.error,
            "completed": list(mission.route.completed),
            "nav_goals": list(fake.nav_goals), "statuses": list(fake.statuses),
        }, indent=2), encoding="utf-8")
        (tmp_path / "runtime_diagnostics.json").write_text(json.dumps({
            "handoff_failure": mission.handoff.failure,
            "handoff_state": str(mission.handoff.state),
            "heartbeat_age": time.monotonic() - mission.handoff.received,
            "sensor_age": time.monotonic() - (mission.stop_monitor.last_wall or 0),
            "motion_timer_age": fake.control.timer.time_since_last_call() / 1e9,
            "sensor_timer_age": fake.tick_timer.time_since_last_call() / 1e9,
            "mission_timer_age": mission.timer.time_since_last_call() / 1e9,
            "threads_alive": [thread.is_alive(), mission_thread.is_alive(),
                              control_thread.is_alive()],
        }, indent=2), encoding="utf-8")
        return (mission.fsm.state, mission.fsm.error,
                list(mission.route.completed), list(fake.nav_goals),
                list(fake.skill_goals), fake.health_calls,
                mission.navigation.unknown_terminal, list(fake.statuses))
    finally:
        mission_executor.shutdown(timeout_sec=3.0)
        control_executor.shutdown(timeout_sec=3.0)
        executor.shutdown(timeout_sec=3.0)
        mission_thread.join(timeout=3.0)
        control_thread.join(timeout=3.0)
        thread.join(timeout=3.0)
        fake.nav_server.destroy()
        fake.control.destroy()
        fake.control_node.destroy_node()
        if fake.skill_server is not None:
            fake.skill_server.destroy()
        if mock_node is not None:
            mock_node.server.destroy()
            mock_node.destroy_node()
        mission.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()
        if mock_process is not None:
            mock_process.terminate()
            try:
                mock_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                mock_process.kill()
                mock_process.wait(timeout=3.0)
            mock_log.close()


def _scenario_process(pipe, args, kwargs):
    try:
        pipe.send(("ok", _run_scenario_inner(*args, **kwargs)))
    except BaseException:
        pipe.send(("error", traceback.format_exc()))
    finally:
        pipe.close()


def run_scenario(*args, **kwargs):
    # rclpy/Humble native handles are isolated per scenario; repeated
    # init/shutdown of ActionServer and TF in one pytest process can crash.
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=_scenario_process,
                              args=(child, args, kwargs))
    process.start()
    child.close()
    try:
        assert parent.poll(48.0), f"scenario timed out; exit={process.exitcode}"
        status, payload = parent.recv()
        assert status == "ok", payload
        return payload
    finally:
        parent.close()
        process.join(timeout=2.0)
        if process.is_alive():
            process.terminate()
            process.join(timeout=2.0)


def test_complete_route_with_real_ros_actions(tmp_path):
    state, error, completed, nav, skill, checks, unknown, statuses = run_scenario(
        tmp_path)
    assert state == State.FINISHED and not error
    assert completed == ["first", "obstacle", "last"]
    assert ["nav_approach.xml" in item for item in nav] == [
        False, False, True, False, False]
    assert skill == [("obstacle", "mock")]
    assert checks >= 2 and not unknown
    assert any(item["schema_version"] == 1 and
               item["state"] == "FINISHED" and
               item["completed_task_ids"] == completed for item in statuses)
    waits = [item for item in statuses if item["state"] == "WAIT_WAYPOINT_STOP"]
    assert any(item["task_id"] == "first" and not item["completed_task_ids"]
               for item in waits)
    assert any(item["task_id"] == "last" and item["completed_task_ids"] ==
               ["first", "obstacle"] for item in waits)
    assert any(item["task_index"] == 3 for item in waits)  # return point


@pytest.mark.parametrize("mode", ["fail", "hold"])
def test_skill_failure_or_control_hold_stops_navigation(tmp_path, mode):
    state, error, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, skill_mode=mode
    )
    assert state == State.ABORTED and error
    assert completed == ["first"]
    assert len(nav) == 3 and len(skill) == 1


def test_localization_failure_after_skill_blocks_next_goal(tmp_path):
    state, error, completed, nav, skill, checks, _, _ = run_scenario(
        tmp_path, health_fail_after=2
    )
    assert state == State.ABORTED and "localization" in error
    assert completed == ["first"] and len(nav) == 3
    assert len(skill) == 1 and checks >= 2


def test_navigation_failure_retries_only_after_terminal(tmp_path):
    state, _, completed, nav, skill, checks, unknown, _ = run_scenario(
        tmp_path, nav_mode="fail_once"
    )
    assert state == State.FINISHED
    assert completed == ["first", "obstacle", "last"]
    assert len(nav) == 6 and len(skill) == 1
    assert checks >= 3 and not unknown


def test_cancel_unconfirmed_blocks_new_goal(tmp_path):
    state, error, completed, nav, skill, _, unknown, _ = run_scenario(
        tmp_path, nav_mode="hang", navigation_timeout=0.2,
        cancel_timeout=0.2,
    )
    assert state == State.ABORTED and "unknown" in error
    assert completed == [] and len(nav) == 1 and not skill
    assert unknown


def test_moving_robot_never_starts_skill(tmp_path):
    state, error, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, moving=True, stop_timeout=0.5
    )
    assert state == State.ABORTED and "did not stop" in error
    assert completed == ["first"] and len(nav) == 3 and not skill


def test_skill_timeout_cancel_stops_mission(tmp_path):
    state, error, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, skill_mode="slow", skill_timeout=0.2,
        cancel_timeout=0.5,
    )
    assert state == State.ABORTED and "skill" in error
    assert completed == ["first"] and len(nav) == 3 and len(skill) == 1


def test_missing_sim_clock_still_times_out_and_publishes_status(tmp_path):
    state, error, completed, nav, skill, checks, _, _ = run_scenario(
        tmp_path, use_sim_time=True, pause_at=State.IDLE,
        localization_timeout=0.6,
    )
    assert state == State.ABORTED and "localization" in error
    assert completed == [] and not nav and not skill
    # Health service can respond independently of /clock; it cannot approve stale TF.
    assert checks >= 1


@pytest.mark.parametrize("nav_mode,unknown", [
    ("wait_cancel", False), ("hang", True),
])
def test_paused_sim_clock_cancels_navigation_and_blocks_next_goal(
        tmp_path, nav_mode, unknown):
    state, error, completed, nav, skill, _, actual_unknown, _ = run_scenario(
        tmp_path, use_sim_time=True, pause_at=State.NAVIGATING,
        nav_mode=nav_mode, navigation_timeout=5.0, cancel_timeout=0.2,
        clock_timeout=0.3, localization_loss_grace=0.2,
        localization_timeout=0.6,
    )
    assert state == State.ABORTED
    assert ("unknown" if unknown else "localization") in error
    assert completed == [] and len(nav) == 1 and not skill
    assert actual_unknown == unknown


def test_paused_sim_clock_skill_timeout_still_cancels(tmp_path):
    state, error, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, use_sim_time=True, pause_at=State.EXECUTING_SKILL,
        skill_mode="slow", skill_timeout=0.4, cancel_timeout=0.5,
        clock_timeout=0.3,
    )
    assert state == State.ABORTED and "skill canceled" in error
    assert completed == ["first"] and len(nav) == 3 and len(skill) == 1


@pytest.mark.parametrize("mode,expected_state,nav_count", [
    ("success", State.FINISHED, 5),
    ("fail", State.ABORTED, 3),
    ("hold", State.ABORTED, 3),
])
def test_existing_mock_skill_contract(tmp_path, mode, expected_state,
                                      nav_count, cross_process_trigger_ready):
    state, error, completed, nav, _, _, _, _ = run_scenario(
        tmp_path, skill_mode=mode, existing_mock=True,
        server_timeout=25.0, skill_timeout=6.0,
    )
    assert state == expected_state and len(nav) == nav_count
    if mode == "fail":
        assert "skill aborted" in error
    elif mode == "hold":
        assert "control not released" in error
    else:
        assert not error
    assert completed == (["first", "obstacle", "last"]
                         if mode == "success" else ["first"])


@pytest.mark.parametrize("mode", ["unstable", "motion_unsafe", "release_fail", "restore_fail"])
def test_independent_stance_release_and_restore_confirmation(tmp_path, mode):
    state, error, completed, nav, skill, _, _, _ = run_scenario(tmp_path, skill_mode=mode)
    assert state == State.ABORTED and error
    assert completed == ["first"] and len(nav) == 3 and len(skill) == 1


@pytest.mark.parametrize("actual,reason", [
    ((0.0, 0.0, 0.0), "xy error"),  # Skill success but no robot movement
    ((1.0, 0.0, 1.0), "yaw error"),
])
def test_skill_success_requires_real_end_pose(tmp_path, actual, reason):
    state, error, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, end_pose=(1.0, 0.0, 0.0), skill_end_pose=actual)
    assert state == State.ABORTED and reason in error
    assert completed == ["first"] and len(nav) == 3 and len(skill) == 1


def test_simulated_pose_arrival_is_required_before_completion(tmp_path):
    state, error, completed, nav, skill, _, _, statuses = run_scenario(
        tmp_path, end_pose=(1.0, 0.0, 0.0), skill_end_pose=(1.0, 0.0, 0.0))
    assert state == State.FINISHED and not error
    assert completed == ["first", "obstacle", "last"] and len(skill) == 1
    assert len(nav) == 5
    assert any(s["state"] == "VERIFYING_END" and
               s["completed_task_ids"] == ["first"] for s in statuses)


def test_localization_loss_recovers_without_repeating_skill(tmp_path):
    state, error, completed, nav, skill, checks, _, statuses = run_scenario(
        tmp_path, health_fail_after=2, relocalize_success=True,
        end_pose=(1.0, 0.0, 0.0), skill_end_pose=(1.0, 0.0, 0.0))
    assert state == State.FINISHED and not error
    assert completed == ["first", "obstacle", "last"] and len(skill) == 1
    assert len(nav) == 5 and checks >= 3
    assert any(s["state"] == "RELOCALIZING" for s in statuses)


def test_control_acquisition_rejected_prevents_skill(tmp_path):
    state, error, completed, nav, skill, _, _, _ = run_scenario(tmp_path, deny_owner="SKILL")
    assert state == State.ABORTED and "handoff" in error
    assert completed == ["first"] and len(nav) == 3 and not skill


def test_nav2_success_with_inaccurate_entry_pose_prevents_skill(tmp_path):
    state, error, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, actual_pose=(0.4, 0.0, 0.0))
    assert state == State.ABORTED and "entry tolerance" in error
    assert completed == ["first"] and len(nav) == 3 and not skill


def test_real_mode_uses_confirmed_external_interfaces(tmp_path):
    state, error, completed, _, skill, _, _, _ = run_scenario(
        tmp_path, simulation_mode=False)
    assert state == State.FINISHED and not error
    assert completed == ["first", "obstacle", "last"] and len(skill) == 1


def test_competition_deadline_cancels_and_never_marks_task_complete(tmp_path):
    state, error, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, nav_mode="wait_cancel", mission_time_limit=0.7)
    assert state == State.ABORTED and "time budget" in error
    assert not completed and len(nav) == 1 and not skill


def test_new_accepted_poses_allow_the_complete_handoff_sequence(tmp_path):
    state, error, completed, _, skill, _, _, _ = run_scenario(
        tmp_path, accepted_pose_topic="/test/accepted_pose")
    assert state == State.FINISHED and not error
    assert completed == ["first", "obstacle", "last"] and len(skill) == 1


def test_fresh_tf_with_pre_approach_accepted_pose_never_starts_skill(tmp_path):
    state, _, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, accepted_pose_topic="/test/accepted_pose",
        freeze_accepted_after_approach=True, stop_timeout=.5)
    assert state == State.ABORTED
    assert completed == ["first"] and len(nav) == 3 and not skill


def test_skill_cancel_result_unknown_never_restores_navigation(tmp_path):
    state, error, completed, nav, skill, _, _, statuses = run_scenario(
        tmp_path, skill_mode="hang", skill_timeout=0.2, cancel_timeout=0.2)
    assert state == State.ABORTED and "skill result/cancellation unknown" in error
    assert completed == ["first"] and len(nav) == 3 and len(skill) == 1
    assert not any(s["state"] == "RESTORING_NAV" and s["task_id"] == "obstacle"
                   for s in statuses)


@pytest.mark.parametrize("nav_index,completed", [
    (1, []), (4, ["first", "obstacle"]),
    (5, ["first", "obstacle", "last"]),
])
def test_nav_success_still_waits_for_measured_stop(tmp_path, nav_index, completed):
    state, error, actual, nav, _, _, _, statuses = run_scenario(
        tmp_path, moving_after_nav=nav_index, stop_timeout=.4)
    assert state == State.ABORTED and "did not stop" in error
    assert actual == completed and len(nav) == nav_index
    assert any(s["state"] == "WAIT_WAYPOINT_STOP" for s in statuses)


def test_waypoint_requires_a_post_arrival_accepted_pose(tmp_path):
    state, error, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, accepted_pose_topic="/test/accepted_pose",
        freeze_accepted_after_nav=1, stop_timeout=.4)
    assert state == State.ABORTED and "no new accepted pose" in error
    assert not completed and len(nav) == 1 and not skill


@pytest.mark.parametrize("mode,expected,nav_count", [
    ("success", State.FINISHED, 5),
    ("fail", State.ABORTED, 3),
    ("hold", State.ABORTED, 3),
])
def test_shipped_state_only_action_inprocess(tmp_path, mode, expected, nav_count):
    state, error, completed, nav, _, _, _, statuses = run_scenario(
        tmp_path, skill_mode=mode, inprocess_mock=True,
        accepted_pose_topic="/test/accepted_pose")
    assert state == expected and len(nav) == nav_count
    assert completed == (["first", "obstacle", "last"] if mode == "success" else ["first"])
    if mode == "success":
        assert not error
        assert any(s["state"] == "VERIFYING_END" for s in statuses)
    else:
        assert error


def test_state_only_action_cannot_fake_arrival_at_a_different_exit(tmp_path):
    state, error, completed, nav, _, _, _, _ = run_scenario(
        tmp_path, inprocess_mock=True, end_pose=(1., 0., 0.))
    assert state == State.ABORTED and "xy error" in error
    assert completed == ["first"] and len(nav) == 3


def test_full_shipped_sim_route_with_stops_and_state_only_actions(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    from robocon_mission.route_manager import RouteManager
    route_path = Path(__file__).resolve().parent / "fixtures" / "patrol_mock_actions.yaml"
    route = RouteManager.load(route_path)
    state, error, completed, nav, _, _, _, statuses = run_scenario(
        tmp_path, inprocess_mock=True, route_path=str(route_path),
        follow_nav_goals=True, accepted_pose_topic="/test/accepted_pose",
        precise_waypoints=True)
    assert state == State.FINISHED and not error
    assert completed == [task.id for task in route.tasks]
    assert len(nav) == 60  # All 29 points and return have transit/approach pairs
    verified_ids = {s["task_id"] for s in statuses if s["state"] == "VERIFYING_END"}
    assert verified_ids == {task.id for task in route.tasks if task.type == "obstacle"}
    wait_ids = {s["task_id"] for s in statuses if s["state"] == "WAIT_WAYPOINT_STOP"}
    assert {task.id for task in route.tasks if task.type == "navigation"} <= wait_ids
    assert any(s["state"] == "WAIT_WAYPOINT_STOP" and s["task_index"] == 29 for s in statuses)




def test_precise_waypoints_and_return_use_real_ros_approach_actions(tmp_path):
    state, error, completed, nav, skill, _, unknown, statuses = run_scenario(
        tmp_path, precise_waypoints=True, follow_nav_goals=True,
        accepted_pose_topic="/test/accepted_pose")
    assert state == State.FINISHED and not error and not unknown
    assert completed == ["first", "obstacle", "last"] and len(skill) == 1
    assert ["nav_approach.xml" in path for path in nav] == [False, True] * 4
    assert any(s["state"] == "APPROACHING_RETURN" for s in statuses)
    assert any(s["state"] == "APPROACHING_WAYPOINT" for s in statuses)
    accepted_tasks = {s["task_id"] for s in statuses
                      if s["waypoint_errors"] is not None and
                      s["waypoint_errors"]["distance"] <= .10}
    assert {"first", "last", ""} <= accepted_tasks


def test_inaccurate_waypoint_realigns_once_and_never_completes(tmp_path):
    state, error, completed, nav, skill, _, unknown, statuses = run_scenario(
        tmp_path, precise_waypoints=True, actual_pose=(.4, 0., 0.),
        accepted_pose_topic="/test/accepted_pose")
    assert state == State.ABORTED and "alignment retries exhausted" in error
    assert not completed and not skill and not unknown
    assert len(nav) == 3  # transit, initial approach, one correction
    assert max(s["waypoint_alignment_attempts"] for s in statuses) == 1


def test_precise_waypoint_requires_new_accepted_pose_after_approach(tmp_path):
    state, error, completed, nav, skill, _, _, _ = run_scenario(
        tmp_path, precise_waypoints=True, accepted_pose_topic="/test/accepted_pose",
        freeze_accepted_after_approach=True, stop_timeout=.5)
    assert state == State.ABORTED and "no new accepted pose" in error
    assert not completed and not skill and len(nav) == 2
