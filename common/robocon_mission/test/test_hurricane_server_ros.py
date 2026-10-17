"""Exercise the skill Action over ROS with measured-feedback test publishers.

The controller below is test-only. It drives no robot, opens no serial port,
and supplies no independent Motion Manager evidence for a mission scheduler.
Every scenario runs in a fresh process and ROS context.
"""

import json
import multiprocessing
import threading
import time
import traceback
from pathlib import Path

import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from go2_obstacle_interfaces.action import ExecuteObstacle
from go2_obstacle_interfaces.msg import ControlState, MotionAuthority, MotionCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from robocon_mission.hurricane_protocol import DryRunTransport
from robocon_mission.hurricane_server import HurricaneServer


class TestController(Node):
    """Publish fresh TF, SKILL ownership, localization health, and step proof."""

    __test__ = False

    def __init__(self, mode):
        super().__init__("test_hurricane_action_controller")
        self.mode = mode
        self.latest = None
        self.observed_steps = set()
        group = ReentrantCallbackGroup()
        self.tf = TransformBroadcaster(self)
        self.authority = self.create_publisher(ControlState, "/motion/control_state", 10)
        self.feedback = self.create_publisher(String, "/hurricane/controller_feedback", 10)
        self.health = self.create_service(
            Trigger, "/mission/localization_ok", self._health, callback_group=group)
        self.status = self.create_subscription(
            String, "/hurricane/status", self._status, 10, callback_group=group)
        self.client = ActionClient(self, ExecuteObstacle, "/execute_obstacle",
                                   callback_group=group)
        self.timer = self.create_timer(0.02, self._tick, callback_group=group)

    @staticmethod
    def _health(_request, response):
        response.success = True
        response.message = "test localization is fresh"
        return response

    def _status(self, message):
        self.latest = json.loads(message.data)
        self.observed_steps.add(self.latest["step_type"])

    def _tick(self):
        stamp = self.get_clock().now()
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)
        self.authority.publish(ControlState(
            stamp=stamp.to_msg(), owner="SKILL", mission_id="test_hurricane_mission",
            healthy=True))
        phase = self.latest
        if phase is None or phase["status"] not in ("awaiting_feedback", "awaiting_safe_stance"):
            return
        # The server rejects any evidence from before the actual UART boundary.
        if phase["phase_started_ns"] <= 0 or stamp.nanoseconds < phase["phase_started_ns"]:
            return
        self.feedback.publish(String(data=json.dumps({
            "stamp_ns": stamp.nanoseconds,
            "session_id": ("another_session" if self.mode == "wrong_session"
                           else phase["session_id"]),
            "obstacle_id": phase["obstacle_id"],
            "step_index": phase["step_index"],
            "event": "completed",
            "completed": True,
            "safe_stance": True,
            "control_released": self.mode != "no_release",
        })))


def _wait_future(future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    if not future.done():
        raise TimeoutError("ROS Action future did not complete")
    return future.result()


def _scenario(directory, mode):
    rclpy.init()
    directory = Path(directory)
    profile = directory / "server_skills.yaml"
    profile.write_text(yaml.safe_dump({
        "version": 1,
        "coordinate_convention": "hurricane_reflected_y",
        "skills": {"test_skill": {
            "calibrated": True,
            "origin": [0.0, 0.0, 0.0],
            "steps": [
                {"type": "mode", "command": "正常模式", "min_wait": 0.02},
                {"type": "waypoint", "target": [0.0, 0.0, 0.0]},
                {"type": "jump", "command": "小跳1", "min_wait": 0.02},
            ],
        }},
    }, allow_unicode=True), encoding="utf-8")
    transport = DryRunTransport()
    server = HurricaneServer([
        Parameter("skills_file", value=str(profile)),
        # Exercise actual owner and feedback checks with a recording transport.
        Parameter("dry_run", value=False),
        Parameter("confirmation_timeout", value=0.6),
        Parameter("step_timeout", value=2.0),
    ], transport=transport)
    controller = TestController(mode)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(controller)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        if not controller.client.wait_for_server(timeout_sec=5.0):
            raise TimeoutError("Hurricane Action server was not discovered")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if server._authorized() and server._health_ready() and server._pose() is not None:
                break
            time.sleep(0.01)
        else:
            raise TimeoutError("fresh TF, localization health, or SKILL ownership missing")
        goal = _wait_future(controller.client.send_goal_async(ExecuteObstacle.Goal(
            obstacle_id="test_obstacle", skill_id="test_skill")), 3.0)
        if not goal.accepted:
            raise AssertionError("test goal unexpectedly rejected")
        response = _wait_future(goal.get_result_async(), 6.0)
        result = response.result
        return {
            "status": response.status,
            "completed": result.completed,
            "control_released": result.control_released,
            "safe_stance": result.safe_stance,
            "error_code": result.error_code,
            "message": result.message,
            "frames": [frame.decode("ascii") for frame in transport.sent],
            "observed_steps": sorted(controller.observed_steps),
        }
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        server.close()
        server.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


def _child(sender, directory, mode):
    try:
        sender.send((True, _scenario(directory, mode)))
    except BaseException:
        sender.send((False, traceback.format_exc()))
    finally:
        sender.close()


def _run_scenario(tmp_path, mode):
    context = multiprocessing.get_context("spawn")
    receiver, sender = context.Pipe(duplex=False)
    process = context.Process(target=_child, args=(sender, str(tmp_path), mode))
    process.start()
    sender.close()
    try:
        assert receiver.poll(20.0), "isolated Hurricane Action scenario timed out"
        okay, result = receiver.recv()
        assert okay, result
        return result
    finally:
        receiver.close()
        process.join(timeout=3.0)
        if process.is_alive():
            process.terminate()
            process.join(timeout=3.0)


def test_controller_confirmed_action_completes_without_serial_io(tmp_path):
    result = _run_scenario(tmp_path, "success")
    assert result["status"] == GoalStatus.STATUS_SUCCEEDED, result
    assert result["completed"] and result["control_released"] and result["safe_stance"], result
    assert result["error_code"] == "", result
    assert any("S6E" in frame for frame in result["frames"]), result
    assert "safe_stance" in result["observed_steps"], result


@pytest.mark.parametrize("mode", ["wrong_session", "no_release"])
def test_unconfirmed_controller_evidence_aborts_action(tmp_path, mode):
    result = _run_scenario(tmp_path, mode)
    assert result["status"] == GoalStatus.STATUS_ABORTED, result
    assert not result["completed"] and not result["control_released"] and not result["safe_stance"], result
    assert result["error_code"] == "HURRICANE_SKILL_FAILED", result
    if mode == "no_release":
        assert "safe_stance" in result["observed_steps"], result
        assert any("S6E" in frame for frame in result["frames"]), result


class GatewayController(TestController):
    """Test-only firmware facade; never a real controller state publisher."""
    __test__ = False

    def __init__(self, mode):
        super().__init__(mode)
        self.request = None
        self.request_at = 0.
        self.requests = []
        self.commands = []
        self.sequence = 0
        self.authority_details = self.create_publisher(MotionAuthority, "/motion/authority", 10)
        self.gateway_status = self.create_publisher(String, "/stm32/status", 10)
        self.gateway_event = self.create_publisher(String, "/stm32/event", 10)
        self.gateway_requests = self.create_subscription(
            String, "/stm32/skill_request", self._request, 10)
        self.skill_commands = self.create_subscription(
            MotionCommand, "/motion/skill_command", self.commands.append, 10)

    def _request(self, message):
        self.request = json.loads(message.data)
        self.request_at = time.monotonic()
        self.requests.append(self.request)
        self.gateway_event.publish(String(data=json.dumps(dict(self.request, accepted=True,
                                                             event="ack"))))

    def _tick(self):
        # Deliberately also emit the old feedback topic. The real server must
        # ignore it, including its forged successful completion in ACK-only mode.
        super()._tick()
        stamp = self.get_clock().now()
        authority = MotionAuthority(owner="SKILL", mission_id="test_hurricane_mission",
                                    generation=2, lease_remaining=2., healthy=True)
        authority.header.stamp = stamp.to_msg()
        self.authority_details.publish(authority)
        self.sequence += 1
        state = dict(known=True, mode="HOLD", stopped=True, safe_stance=True,
                     control_releasable=True, skill_active=False, obstacle_id="test_obstacle",
                     skill_state="idle", estop=False, human_enable=True, fault_code="",
                     capabilities=[] if self.mode == "unsupported" else [9, 6])
        if self.request:
            state.update({key: self.request[key] for key in
                          ("request_id", "runner_session", "step_index")})
            running = time.monotonic() - self.request_at < .04 or self.mode in ("ack_only", "cancel")
            state["skill_state"] = "running" if running else (
                "failed" if self.mode == "failed" else "succeeded")
            state["mode"] = "HOLD" if self.request["op"] == "hold" else "SKILL"
        self.gateway_status.publish(String(data=json.dumps(dict(
            connected=True, handshake=True, session=123, sequence=self.sequence,
            gateway_sequence=self.sequence, stamp_ns=stamp.nanoseconds,
            status=state, error_code=""))))


def _gateway_scenario(directory, mode):
    rclpy.init()
    profile = Path(directory) / "gateway_skills.yaml"
    profile.write_text(yaml.safe_dump({
        "version": 1, "coordinate_convention": "hurricane_reflected_y",
        "skills": {"test_skill": {"calibrated": True, "origin": [0., 0., 0.],
            "steps": [{"type": "mode", "command": "正常模式", "min_wait": .02},
                      {"type": "waypoint", "target": [0., 0., 0.]},
                      {"type": "jump", "command": "小跳1", "min_wait": .02}]}},
    }, allow_unicode=True), encoding="utf-8")
    server = HurricaneServer([
        Parameter("skills_file", value=str(profile)), Parameter("dry_run", value=False),
        Parameter("confirmation_timeout", value=.4), Parameter("step_timeout", value=2.)])
    controller = GatewayController(mode)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(controller)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert controller.client.wait_for_server(timeout_sec=5.)
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            if server._authorized() and server._health_ready() and server._pose() is not None:
                break
            time.sleep(.01)
        else:
            raise TimeoutError("gateway handshake, fresh status or SKILL authority missing")
        ready = server._skills_ready(Trigger.Request(), Trigger.Response())
        assert ready.success is (mode != "unsupported")
        assert json.loads(ready.message)["skills"] == ([] if mode == "unsupported" else ["test_skill"])
        goal = _wait_future(controller.client.send_goal_async(ExecuteObstacle.Goal(
            obstacle_id="test_obstacle", skill_id="test_skill")), 3.)
        assert goal.accepted
        if mode == "cancel":
            deadline = time.monotonic() + 2.
            while not controller.requests and time.monotonic() < deadline:
                time.sleep(.01)
            assert controller.requests
            _wait_future(goal.cancel_goal_async(), 3.)
        response = _wait_future(goal.get_result_async(), 6.)
        result = response.result
        return dict(status=response.status, completed=result.completed,
                    safe_stance=result.safe_stance, released=result.control_released,
                    error=result.error_code, message=result.message,
                    requests=controller.requests,
                    commands=[dict(owner=m.owner, frame=m.header.frame_id,
                                   vy=m.twist.linear.y) for m in controller.commands])
    finally:
        executor.shutdown(timeout_sec=2.)
        thread.join(timeout=2.)
        server.close()
        server.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


def _gateway_child(sender, directory, mode):
    try:
        sender.send((True, _gateway_scenario(directory, mode)))
    except BaseException:
        sender.send((False, traceback.format_exc()))
    finally:
        sender.close()


def _run_gateway_scenario(tmp_path, mode):
    context = multiprocessing.get_context("spawn")
    receiver, sender = context.Pipe(duplex=False)
    process = context.Process(target=_gateway_child, args=(sender, str(tmp_path), mode))
    process.start()
    sender.close()
    try:
        assert receiver.poll(20.), "gateway Action scenario timed out"
        okay, result = receiver.recv()
        assert okay, result
        return result
    finally:
        receiver.close()
        process.join(timeout=3.)
        if process.is_alive():
            process.terminate()
            process.join(timeout=3.)


def test_gateway_status_completes_existing_action_without_serial_or_legacy_feedback(tmp_path):
    result = _run_gateway_scenario(tmp_path, "success")
    assert result["status"] == GoalStatus.STATUS_SUCCEEDED, result
    assert result["completed"] and result["safe_stance"] and result["released"], result
    assert [request["command_id"] for request in result["requests"]] == [9, 5, 6, 5], result
    assert result["commands"] and all(command == dict(owner="SKILL", frame="base_footprint", vy=0.)
                                      for command in result["commands"]), result


@pytest.mark.parametrize("mode,error", [("ack_only", "HURRICANE_SKILL_FAILED"),
                                        ("failed", "HURRICANE_SKILL_FAILED"),
                                        ("unsupported", "UNSUPPORTED")])
def test_gateway_ack_timeout_failure_and_unsupported_cannot_complete_action(tmp_path, mode, error):
    result = _run_gateway_scenario(tmp_path, mode)
    assert result["status"] == GoalStatus.STATUS_ABORTED, result
    assert not result["completed"] and not result["safe_stance"] and not result["released"], result
    assert result["error"] == error, result


def test_gateway_action_cancel_sends_cancel_and_hold_without_claiming_safe_stance(tmp_path):
    result = _run_gateway_scenario(tmp_path, "cancel")
    assert result["status"] == GoalStatus.STATUS_CANCELED, result
    assert not result["completed"] and not result["safe_stance"] and not result["released"], result
    assert any(request["op"] == "cancel" for request in result["requests"]), result
    assert result["requests"][-1]["op"] == "hold", result
