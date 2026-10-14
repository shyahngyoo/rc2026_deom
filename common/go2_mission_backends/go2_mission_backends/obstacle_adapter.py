"""ExecuteObstacle server with replaceable executor and measured safe completion.

The server never grants itself control and never switches SAFE_HOLD after a
successful skill. Existing Mission owns that transition. Result.release is only
true after executor output release AND independent GetMotionState confirmation.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger
from go2_obstacle_interfaces.action import ExecuteObstacle
from go2_obstacle_interfaces.msg import MotionAuthority, MotionCommand
from go2_obstacle_interfaces.srv import GetMotionState
from .skill_executor import load_skills, TestSkillExecutor


class ObstacleAdapter(Node):
    def __init__(self, executor_factories=None, **kwargs):
        super().__init__("obstacle_action_adapter", **kwargs)
        defaults = dict(skills_file="", simulation_mode=False, allowed_skills=[],
                        authority_topic="/motion/authority", command_topic="/motion/skill_command",
                        motion_state_service="/motion/get_state", action_name="/execute_obstacle",
                        ready_service="/skills/ready", emergency_service="/motion/emergency_hold",
                        base_frame="base_footprint", max_execution_time=30.0,
                        state_timeout=0.5, authority_timeout=0.5, release_timeout=2.0,
                        state_poll_period=0.05, timer_period=0.02)
        for key, value in defaults.items():
            # An empty list needs an explicit string-array type in Humble.
            if key == "allowed_skills":
                self.declare_parameter(key, value, ParameterDescriptor(dynamic_typing=True))
            else:
                self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        for key in ("max_execution_time", "state_timeout", "authority_timeout", "release_timeout",
                    "state_poll_period", "timer_period"):
            value = self.p(key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError("invalid " + key)
        if not self.p("base_frame"):
            raise ValueError("base_frame is required")
        self.factories = {"test": TestSkillExecutor}
        self.factories.update(executor_factories or {})
        self.skills = load_skills(self.p("skills_file"), self.p("allowed_skills") or [])
        self.authority = None
        self.authority_wall = None
        self.authority_stamp = 0
        self.authority_generation = 0
        self.op = None
        self.reserved = False
        self.lock = threading.RLock()
        self.command_pub = self.create_publisher(MotionCommand, self.p("command_topic"), 1)
        self.event_pub = self.create_publisher(String, "/backend_sim/skill_state", 10)
        self.status_pub = self.create_publisher(String, "/skills/status", 10)
        self.create_subscription(MotionAuthority, self.p("authority_topic"), self._authority, 10)
        self.motion_state = self.create_client(GetMotionState, self.p("motion_state_service"))
        self.emergency = self.create_client(Trigger, self.p("emergency_service"))
        self.create_service(Trigger, self.p("ready_service"), self._ready)
        self.group = ReentrantCallbackGroup()
        self.server = ActionServer(self, ExecuteObstacle, self.p("action_name"),
                                   self._execute, goal_callback=self._goal,
                                   cancel_callback=self._cancel, handle_accepted_callback=self._accepted,
                                   callback_group=self.group)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(self.p("timer_period"), self._tick, clock=self.steady)

    @staticmethod
    def _stamp(message):
        if message.header.stamp.sec < 0 or not 0 <= message.header.stamp.nanosec < 1000000000:
            return -1
        return message.header.stamp.sec*1000000000 + message.header.stamp.nanosec

    def _authority(self, message):
        with self.lock:
            stamp = self._stamp(message)
            now = self.get_clock().now().nanoseconds
            if (stamp <= 0 or not 0 <= (now-stamp)/1e9 <= self.p("authority_timeout") or
                    not math.isfinite(message.lease_remaining) or message.lease_remaining < 0):
                self.authority = None
                return
            previous = self.authority
            if message.generation < self.authority_generation:
                self.authority = None
                return
            if stamp == self.authority_stamp:
                if previous is not None and (message.owner, message.mission_id, message.generation, message.healthy) == (
                        previous.owner, previous.mission_id, previous.generation, previous.healthy):
                    return  # identical simulated-time sample never refreshes wall progress.
                if message.generation <= self.authority_generation:
                    self.authority = None
                    return
            elif stamp < self.authority_stamp or (previous is not None and message.generation < previous.generation):
                self.authority = None
                return
            self.authority_stamp = stamp
            self.authority_generation = message.generation
            self.authority, self.authority_wall = message, time.monotonic()

    def _authorized(self, op=None):
        a = self.authority
        if not a or self.authority_wall is None:
            return False
        wall_age = time.monotonic() - self.authority_wall
        ros_age = (self.get_clock().now().nanoseconds-self._stamp(a))/1e9
        return bool(a.owner == "SKILL" and a.healthy and a.mission_id and a.generation > 0 and
                    wall_age <= self.p("authority_timeout") and 0 <= ros_age <= self.p("authority_timeout") and
                    a.lease_remaining > wall_age and (op is None or
                    (a.generation == op["generation"] and a.mission_id == op["mission_id"])))

    def _available(self, skill):
        factory = self.factories.get(skill.executor)
        return bool(skill.enabled and factory and
                    (self.p("simulation_mode") or not getattr(factory, "simulation_only", False)))

    def _ready(self, _request, response):
        available = sorted(k for k, s in self.skills.items() if self._available(s))
        response.success = bool(available)
        response.message = json.dumps(dict(skills=available, simulation_mode=self.p("simulation_mode"),
                                           reason="READY" if available else "NOT_CONFIGURED"))
        return response

    def _goal(self, goal):
        with self.lock:
            skill = self.skills.get(goal.skill_id)
            reason = ("BUSY" if self.reserved else "INVALID_OBSTACLE_ID" if not goal.obstacle_id else
                      "NOT_CONFIGURED" if skill is None or not self._available(skill) else
                      "SKILL_AUTHORITY_UNCONFIRMED" if not self._authorized() else "")
            if reason:
                self.status_pub.publish(String(data=json.dumps(dict(event="goal_rejected", reason=reason,
                                                                    obstacle_id=goal.obstacle_id, skill_id=goal.skill_id))))
                return GoalResponse.REJECT
            self.reserved = True
            return GoalResponse.ACCEPT

    def _accepted(self, goal_handle):
        with self.lock:
            a = self.authority
            now = time.monotonic()
            self.op = dict(handle=goal_handle, request_id=bytes(goal_handle.goal_id.uuid).hex(),
                           obstacle_id=goal_handle.request.obstacle_id,
                           skill=self.skills[goal_handle.request.skill_id],
                           mission_id=a.mission_id if a else "", generation=a.generation if a else 0,
                           phase="PRECHECK", future=Future(), executor=None, started=now,
                           deadline=now+min(self.p("max_execution_time"), self.skills[goal_handle.request.skill_id].execution_timeout),
                           cancel_requested=False, terminal="", query=None, query_started=0., query_next=0.,
                           state=None, state_wall=0., last_valid_state=now, finishing_at=0., released_at=0.)
            goal_handle.execute()

    async def _execute(self, goal_handle):
        with self.lock:
            op = self.op
            if op is None or op["handle"] is not goal_handle:
                goal_handle.abort()
                return ExecuteObstacle.Result(error_code="INTERNAL_GOAL_MISMATCH", message="goal not reserved")
            future = op["future"]
        return await future

    def _cancel(self, goal_handle):
        with self.lock:
            op = self.op
            if op is None or op["handle"] is not goal_handle or op["phase"] == "RELEASED":
                return CancelResponse.REJECT
            op["cancel_requested"] = True
            op["cancel_at"] = time.monotonic()
            return CancelResponse.ACCEPT

    def _event(self, op, event, error=""):
        data = dict(event=event, obstacle_id=op["obstacle_id"], request_id=op["request_id"],
                    mission_id=op["mission_id"], generation=op["generation"], mode=op["skill"].mode,
                    skill_id=op["skill"].skill_id, error_code=error)
        self.status_pub.publish(String(data=json.dumps(data)))
        # Test state events are only connected to a simulator. Real executor
        # implementations provide independent physical telemetry themselves.
        if self.p("simulation_mode"):
            self.event_pub.publish(String(data=json.dumps(data)))

    def _command(self, op, velocity):
        m = MotionCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.p("base_frame")
        m.owner, m.mission_id, m.generation = "SKILL", op["mission_id"], op["generation"]
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = velocity
        self.command_pub.publish(m)

    def _query(self, op, now):
        pending = op["query"]
        if pending is not None:
            if pending.done():
                if now-op["query_started"] > self.p("state_timeout"):
                    op["state"] = None
                else:
                    try:
                        op["state"] = pending.result()
                        op["state_wall"] = now
                        op["state_requested"] = op["query_started"]
                    except Exception:
                        op["state"] = None
                op["query"] = None
            elif now-op["query_started"] > self.p("state_timeout"):
                pending.cancel()
                op["query"], op["state"] = None, None
        if op["query"] is None and now >= op["query_next"] and self.motion_state.service_is_ready():
            req = GetMotionState.Request(obstacle_id="" if op["phase"] in ("PRECHECK", "CANCEL_PRECHECK") else op["obstacle_id"])
            op["query"] = self.motion_state.call_async(req)
            op["query_started"] = now
            op["query_next"] = now + self.p("state_poll_period")

    def _measured(self, op, safe=False, after=0.):
        state = op["state"]
        return bool(state and time.monotonic()-op["state_wall"] <= self.p("state_timeout") and
                    op.get("state_requested", 0.) >= after and state.known and not state.error_code and
                    (op["phase"] in ("PRECHECK", "CANCEL_PRECHECK") or state.obstacle_id == op["obstacle_id"]) and
                    (not safe or (not state.skill_active and state.safe_stance and state.control_releasable)))

    def _finish(self, op, kind, error=""):
        if self.op is not op:
            return
        success = kind == "completed"
        released = kind in ("completed", "canceled")
        message = ("executor never started; independent measured safe state confirmed" if
                   released and op["executor"] is None else
                   "executor released; independent measured safe state confirmed" if released else error)
        result = ExecuteObstacle.Result(completed=success, control_released=released,
                                        safe_stance=released, error_code=error,
                                        message=message)
        if kind == "canceled":
            op["handle"].canceled()
        elif success:
            op["handle"].succeed()
        else:
            op["handle"].abort()
        if op["query"] is not None:
            op["query"].cancel()
        op["future"].set_result(result)
        self.op, self.reserved = None, False

    def _fault(self, op, error):
        self._command(op, (0., 0., 0.))
        self._event(op, "fault", error)
        if op["executor"] is not None:
            try:
                op["executor"].cancel(time.monotonic())
                op["executor"].close(time.monotonic())
            except Exception as cleanup_error:
                self.get_logger().error("executor cleanup failed: " + str(cleanup_error))
        if self.emergency.service_is_ready():
            self.emergency.call_async(Trigger.Request())
        self._finish(op, "failed", error)

    def _tick(self):
        with self.lock:
            op = self.op
            if op is None:
                return
            now = time.monotonic()
            if not self._authorized(op):
                self._fault(op, "SKILL_AUTHORITY_LOST")
                return
            if now >= op["deadline"]:
                self._fault(op, "SKILL_TIMEOUT")
                return
            if op["phase"] == "PRECHECK" and op["cancel_requested"]:
                # Cancellation won the race against physical executor start.
                # Confirm new independent support data before acknowledging it.
                op["phase"] = "CANCEL_PRECHECK"
                op["finishing_at"] = op.get("cancel_at", now)
                op["terminal"] = "canceled"
            if op["phase"] in ("PRECHECK", "CANCEL_PRECHECK"):
                # Keep an authorized idle command fresh while querying support;
                # the first active telemetry sample must not revive an expired
                # pre-handoff command heartbeat. This never authorizes movement.
                self._command(op, (0., 0., 0.))
            self._query(op, now)
            try:
                if op["phase"] == "CANCEL_PRECHECK":
                    if self._measured(op, safe=True, after=op["finishing_at"]):
                        self._finish(op, "canceled", "CANCELED_BEFORE_START")
                    elif now-op["finishing_at"] > self.p("release_timeout"):
                        self._fault(op, "CANCEL_SAFE_STATE_UNCONFIRMED")
                    return
                if op["phase"] == "PRECHECK":
                    if not self._measured(op, safe=True):
                        if now-op["started"] > self.p("state_timeout"):
                            self._fault(op, "INITIAL_SAFE_STATE_UNCONFIRMED")
                        return
                    op["executor"] = self.factories[op["skill"].executor](op["skill"], simulation_mode=self.p("simulation_mode"))
                    op["executor"].start(now)
                    op["phase"] = "RUNNING"
                    op["last_valid_state"] = now
                    self._event(op, "running")
                if op["cancel_requested"] and op["phase"] in ("RUNNING", "FINISHING"):
                    if not op["executor"].cancel(now):
                        self._fault(op, "CANCEL_UNCONFIRMED")
                        return
                    op["cancel_requested"] = False
                    op["terminal"] = "canceled"
                    if op["phase"] != "FINISHING":
                        op["phase"], op["finishing_at"] = "FINISHING", now
                        self._event(op, "finishing")
                if op["phase"] == "RUNNING":
                    if self._measured(op):
                        op["last_valid_state"] = now
                    elif now-op["last_valid_state"] > self.p("state_timeout"):
                        self._fault(op, "RUNNING_STATE_UNCONFIRMED")
                        return
                    update = op["executor"].tick(now)
                    if (len(update.velocity) != 3 or not all(math.isfinite(v) for v in update.velocity) or
                            not math.isfinite(update.progress) or not 0. <= update.progress <= 1. or
                            update.terminal not in ("", "completed", "canceled", "failed")):
                        raise RuntimeError("INVALID_EXECUTOR_UPDATE")
                    self._command(op, update.velocity)
                    feedback = ExecuteObstacle.Feedback(phase=update.phase, progress=float(update.progress))
                    op["handle"].publish_feedback(feedback)
                    if update.terminal == "failed":
                        self._fault(op, update.error_code or "SKILL_FAILED")
                        return
                    if update.terminal:
                        op["terminal"], op["phase"], op["finishing_at"] = update.terminal, "FINISHING", now
                        self._event(op, "finishing")
                if op["phase"] == "FINISHING":
                    self._command(op, (0., 0., 0.))
                    if now-op["finishing_at"] > self.p("release_timeout"):
                        self._fault(op, "CONTROL_RELEASE_UNCONFIRMED")
                        return
                    if self._measured(op, safe=True, after=op["finishing_at"]):
                        if not op["executor"].close(now):
                            self._fault(op, "EXECUTOR_RELEASE_FAILED")
                            return
                        op["phase"], op["released_at"] = "RELEASED", now
                        self._event(op, "released")
                if op["phase"] == "RELEASED":
                    self._command(op, (0., 0., 0.))
                    if self._measured(op, safe=True, after=op["released_at"]):
                        self._finish(op, op["terminal"], "CANCELED" if op["terminal"] == "canceled" else "")
                    elif now-op["released_at"] > self.p("release_timeout"):
                        self._fault(op, "RELEASED_STATE_UNCONFIRMED")
            except Exception as error:
                self.get_logger().error("executor fault: " + str(error))
                self._fault(op, "EXECUTOR_EXCEPTION")

    def destroy_node(self):
        with self.lock:
            if self.op is not None:
                self._fault(self.op, "SERVER_SHUTDOWN")
        self.server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ObstacleAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
