"""ExecuteObstacle server for transplanted Hurricane sequences and serial frames.

The legacy UART has no completion frames. An independent controller adapter must
publish matching, measured feedback; a successful write or elapsed wait is never
treated as completion. Default dry_run records frames and opens no serial port.
"""
import json
import math
import time
from functools import wraps
from threading import RLock

import rclpy
from ament_index_python.packages import get_package_share_directory
from go2_obstacle_interfaces.action import ExecuteObstacle
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from pathlib import Path

from .hurricane_protocol import DryRunTransport, SerialTransport
from .hurricane_skills import SkillRunner, load_skills


def serialized(function):
    @wraps(function)
    def call(self, *args, **kwargs):
        with self.lock:
            return function(self, *args, **kwargs)
    return call


class ObservedTransport:
    """Mark the actual send boundary before a fast controller can acknowledge it."""
    def __init__(self, transport, before_send):
        self.transport = transport
        self.before_send = before_send

    def send(self, command, repeat=1):
        self.before_send()
        return self.transport.send(command, repeat=repeat)


class HurricaneServer(Node):
    def __init__(self, parameter_overrides=None, transport=None):
        super().__init__("hurricane_skill_server", parameter_overrides=parameter_overrides)
        self.lock = RLock()
        defaults = {
            "skills_file": str(Path(get_package_share_directory("robocon_mission")) /
                               "config/hurricane_skills.yaml"),
            "dry_run": True, "serial_port": "", "baudrate": 115200,
            "action_name": "/execute_obstacle", "map_frame": "map",
            "base_frame": "base_footprint", "pose_timeout": 0.5,
            "step_timeout": 30.0, "confirmation_timeout": 10.0,
            "control_state_timeout": 0.5,
            "localization_ok_service": "/mission/localization_ok",
            "health_timeout": 0.5,
            "controller_feedback_topic": "/hurricane/controller_feedback",
            "control_state_topic": "/motion/control_state",
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        param = lambda key: self.get_parameter(key).value
        self.dry_run = param("dry_run")
        if not self.dry_run and not all(hasattr(ExecuteObstacle.Result(), name)
                                       for name in ("safe_stance", "error_code")):
            raise ValueError("rebuild go2_obstacle_interfaces with safe_stance/error_code")
        for key in ("pose_timeout", "step_timeout", "confirmation_timeout",
                    "control_state_timeout", "health_timeout"):
            if not math.isfinite(param(key)) or param(key) <= 0:
                raise ValueError(key + " must be finite and positive")
        if self.dry_run and transport is not None and not isinstance(transport, DryRunTransport):
            raise ValueError("dry_run only accepts DryRunTransport")
        profiles = load_skills(param("skills_file"))
        if not self.dry_run and not any(p["calibrated"] for p in profiles.values()):
            raise ValueError("hardware skills require site-calibrated profiles")
        self.transport = transport or (DryRunTransport() if self.dry_run else
                                       SerialTransport(param("serial_port"), param("baudrate")))
        self.runner = SkillRunner(profiles, ObservedTransport(self.transport, self._before_send),
                                  pose_timeout=param("pose_timeout"),
                                  step_timeout=param("step_timeout"),
                                  confirmation_timeout=param("confirmation_timeout"),
                                  allow_uncalibrated=self.dry_run)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.reserved = False
        self.goal = None
        self.future = None
        self.control_state = None
        self.control_received = 0.0
        self.control_mission_id = ""
        self.phase_key = None
        self.phase_started_ns = 0
        self.final_release = False
        self.execution_started = False
        self.last_event_key = None
        self.last_event_stamp = 0
        self.feedback_key = None
        self.feedback_stamp = 0
        self.feedback_received = 0.0
        self.last_clock = 0.0
        self.health_valid = False
        self.health_received = 0.0
        self.health_pending = None
        self.next_health_poll = 0.0
        self.health_client = (None if self.dry_run else self.create_client(
            Trigger, param("localization_ok_service")))
        self.use_sim_time = self.get_parameter("use_sim_time").value
        group = ReentrantCallbackGroup()
        self.feedback_sub = self.create_subscription(
            String, param("controller_feedback_topic"), self._feedback, 10,
            callback_group=group)
        self.status_pub = self.create_publisher(String, "/hurricane/status", 10)
        if not self.dry_run:
            from go2_obstacle_interfaces.msg import ControlState
            self.control_sub = self.create_subscription(
                ControlState, param("control_state_topic"), self._control_state, 10,
                callback_group=group)
        if self.use_sim_time:
            from rosgraph_msgs.msg import Clock as ClockMessage
            self.clock_sub = self.create_subscription(ClockMessage, "/clock",
                                                      self._clock, 10,
                                                      callback_group=group)
        self.server = ActionServer(
            self, ExecuteObstacle, param("action_name"), self._execute,
            goal_callback=self._request, cancel_callback=self._cancel,
            handle_accepted_callback=self._accepted, callback_group=group)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self._tick, clock=self.wall_clock,
                                      callback_group=group)
        self.get_logger().info(f"Hurricane adapter ready; dry_run={self.dry_run}")

    @serialized
    def _clock(self, _message):
        self.last_clock = time.monotonic()

    @serialized
    def _control_state(self, message):
        self.control_state = message
        self.control_received = time.monotonic()

    def _authorized(self):
        if self.dry_run:
            return True
        msg = self.control_state
        if msg is None:
            return False
        stamp = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        timeout = self.get_parameter("control_state_timeout").value
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return (msg.owner == "SKILL" and msg.healthy and bool(msg.mission_id)
                and (not self.control_mission_id or msg.mission_id == self.control_mission_id)
                and stamp > 0 and -0.1 <= age <= timeout
                and time.monotonic() - self.control_received <= timeout)

    def _health_ready(self):
        return self.dry_run or (self.health_valid and time.monotonic() - self.health_received <=
                               self.get_parameter("health_timeout").value)

    def _poll_health(self):
        if self.health_client is None:
            return
        now = time.monotonic()
        timeout = self.get_parameter("health_timeout").value
        if self.health_pending is not None:
            future, deadline = self.health_pending
            if future.done():
                try:
                    self.health_valid = bool(future.result().success)
                except Exception:
                    self.health_valid = False
                self.health_received = now
                self.health_pending = None
            elif now >= deadline:
                self.health_valid = False
                self.health_pending = None
        if self.health_pending is None and now >= self.next_health_poll:
            self.next_health_poll = now + timeout / 3
            if self.health_client.service_is_ready():
                self.health_pending = (self.health_client.call_async(Trigger.Request()), now + timeout)
            else:
                self.health_valid = False

    @serialized
    def _request(self, request):
        if (self.reserved or not self._authorized() or not self._health_ready()
                or not request.obstacle_id or self._pose() is None):
            return GoalResponse.REJECT
        skills = self.runner.skills
        if request.skill_id not in skills:
            return GoalResponse.REJECT
        profile = skills[request.skill_id]
        calibrated = (profile.get("calibrated", False) if isinstance(profile, dict)
                      else getattr(profile, "calibrated", False))
        if not self.dry_run and not calibrated:
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    @serialized
    def _accepted(self, goal):
        self.goal = goal
        self.future = Future()
        self.final_release = False
        self.execution_started = False
        self.last_event_key = None
        self.phase_key = None
        self.feedback_key = None
        self.control_mission_id = ("" if self.dry_run else self.control_state.mission_id)
        goal.execute()

    async def _execute(self, goal):
        with self.lock:
            future = self.future
            try:
                self.runner.start(goal.request.skill_id, time.monotonic(),
                                  obstacle_id=goal.request.obstacle_id)
                self.execution_started = True
            except Exception as error:
                self._finish(False, str(error))
        try:
            return await future
        finally:
            with self.lock:
                if self.goal is goal:
                    self.goal = None
                    self.future = None
                    self.reserved = False
                    self.control_mission_id = ""
                    self.execution_started = False

    @serialized
    def _cancel(self, _goal):
        return CancelResponse.ACCEPT

    def _pose(self):
        if self.use_sim_time and time.monotonic() - self.last_clock > 0.5:
            return None
        try:
            tf = self.buffer.lookup_transform(self.get_parameter("map_frame").value,
                                             self.get_parameter("base_frame").value, Time())
            stamp = tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp <= 0 or not -0.1 <= age <= self.get_parameter("pose_timeout").value:
                return None
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            return (tf.transform.translation.x, tf.transform.translation.y, yaw)
        except Exception:
            return None

    def _before_send(self):
        self.phase_key = (self.runner.session_id, self.runner.index)
        self.phase_started_ns = self.get_clock().now().nanoseconds

    @serialized
    def _feedback(self, message):
        if self.goal is None or self.future is None or self.future.done() or not self.execution_started:
            return
        try:
            data = json.loads(message.data)
            stamp = data["stamp_ns"]
            index = data["step_index"]
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            valid = (type(stamp) is int and type(index) is int
                     and data["session_id"] == self.runner.session_id
                     and data["obstacle_id"] == self.goal.request.obstacle_id
                     and index == self.runner.index and stamp >= self.phase_started_ns
                     and self.phase_key == (self.runner.session_id, index)
                     and self.phase_started_ns > 0
                     and -0.1 <= age <= self.get_parameter("pose_timeout").value
                     and type(data["completed"]) is bool and type(data["safe_stance"]) is bool
                     and self._authorized())
            if not valid:
                return
            key = (self.runner.session_id, index)
            if self.last_event_key == key and stamp <= self.last_event_stamp:
                return
            self.last_event_key, self.last_event_stamp = key, stamp
            event = data.get("event", "completed")
            if event == "progress":
                self.runner.clear_confirmation()
                self.final_release = False
                return
            if (event == "failed" or data.get("known") is False or data.get("error_code")
                    or data["completed"] is False or data["safe_stance"] is False):
                self.final_release = False
                stopped = self.runner.feedback(data["session_id"], index, False, False,
                                               time.monotonic())
                if not stopped:
                    self.runner.cancel(time.monotonic())
                self._finish(False, "controller reported failure or unsafe stance")
                return
            if event not in ("completed", "state"):
                return
            if self.runner.status == "awaiting_safe_stance":
                if data.get("control_released") is not True:
                    self.final_release = False
                    self.runner.clear_confirmation()
                    return
                self.final_release = True
            self.feedback_key = (self.runner.session_id, index)
            self.feedback_stamp = stamp
            self.feedback_received = time.monotonic()
            self.runner.feedback(data["session_id"], index, True, True,
                                 time.monotonic())
        except (ValueError, TypeError, KeyError):
            self.get_logger().warn("Invalid or stale controller feedback ignored")

    def _result(self, completed, error=""):
        result = ExecuteObstacle.Result()
        result.completed = completed
        result.control_released = completed and self.final_release
        result.message = error or "controller-confirmed Hurricane skill completed"
        if completed and self.dry_run:
            result.message = "DRY_RUN: sequence validated without hardware output"
        if hasattr(result, "safe_stance"):
            result.safe_stance = completed and not self.dry_run
        if hasattr(result, "error_code"):
            result.error_code = ("DRY_RUN" if completed and self.dry_run else
                                 "" if completed else "HURRICANE_SKILL_FAILED")
        return result

    @serialized
    def _finish(self, completed, error="", canceled=False):
        if self.future is None or self.future.done():
            return
        if canceled:
            self.goal.canceled()
        elif completed:
            self.goal.succeed()
        else:
            self.goal.abort()
        self.future.set_result(self._result(completed, error))

    @serialized
    def _tick(self):
        self._poll_health()
        if self.goal is None or self.future is None or self.future.done() or not self.execution_started:
            return
        try:
            if self.goal.is_cancel_requested:
                self.runner.cancel(time.monotonic())
                self._finish(False, "canceled; physical release remains unconfirmed", True)
            elif not self._authorized() or not self._health_ready():
                self.runner.cancel(time.monotonic())
                self._finish(False, "SKILL ownership or localization health lost")
            else:
                pose = self._pose()
                if pose is None:
                    self.runner.cancel(time.monotonic())
                    self._finish(False, "localization pose stale")
                    return
                now = time.monotonic()
                ack_age = (self.get_clock().now().nanoseconds - self.feedback_stamp) / 1e9
                ack_limit = self.get_parameter("pose_timeout").value
                if self.runner.status in ("awaiting_feedback", "awaiting_safe_stance") and (
                        self.feedback_key != (self.runner.session_id, self.runner.index)
                        or not -0.1 <= ack_age <= ack_limit
                        or now - self.feedback_received > ack_limit):
                    self.runner.clear_confirmation()
                    self.final_release = False
                status = self.runner.tick(pose, time.monotonic())
                if status == "completed":
                    self._finish(self.final_release, "" if self.final_release else
                                 "controller release unconfirmed")
                elif status in ("failed", "canceled"):
                    self._finish(False, self.runner.error or status)
        except Exception as error:
            try:
                self.runner.cancel(time.monotonic())
            except Exception:
                pass
            self._finish(False, str(error))
        self.status_pub.publish(String(data=json.dumps({
            "dry_run": self.dry_run, "session_id": self.runner.session_id,
            "obstacle_id": self.goal.request.obstacle_id, "skill_id": self.runner.skill_id,
            "step_index": self.runner.index, "status": self.runner.status,
            "step_type": self.runner.step_type, "target": self.runner.target,
            "phase_started_ns": (self.phase_started_ns if self.phase_key ==
                                 (self.runner.session_id, self.runner.index) else 0),
            "error": self.runner.error,
        }, ensure_ascii=False)))

    @serialized
    def close(self):
        try:
            if self.goal is not None and self.future is not None and not self.future.done():
                self.runner.cancel(time.monotonic())
                self._finish(False, "shutdown; physical release remains unconfirmed")
        finally:
            self.transport.close()
            self.server.destroy()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = HurricaneServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
