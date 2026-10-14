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
from go2_obstacle_interfaces.msg import ControlState
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
