"""ROS 2 adapters execute FSM Commands without blocking the Executor."""
import json
import math
import time
from dataclasses import replace
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .async_service import AsyncServiceRunner
from .control_handoff import ControlHandoff
from .end_pose_verifier import EndPoseVerifier, pose_errors
from .localization_client import LocalizationClient
from .mission_fsm import MissionFSM, State
from .mission_status import MissionStatusPublisher
from .motion_state_client import MotionStateClient
from .navigation_client import NavigationClient, installed_trees
from .obstacle_client import ObstacleClient
from .relocalization_client import RelocalizationClient
from .route_manager import RouteManager
from .stop_monitor import StopMonitor


class MissionManager(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__("mission_manager", parameter_overrides=parameter_overrides, **kwargs)
        defaults = {
            "route_file": "", "transit_bt_file": "", "approach_bt_file": "",
            "nav_action_name": "/navigate_to_pose",
            "obstacle_action_name": "/execute_obstacle",
            "localization_ok_service": "/mission/localization_ok",
            "control_service": "/motion/set_control_mode",
            "control_state_topic": "/motion/control_state",
            "motion_state_service": "/motion/get_state",
            "enable_relocalization": False,
            "relocalization_action_name": "/localization/relocalize",
            "base_frame": "base_footprint",
            "startup_check_service": "", "startup_check_max_age": 1.0,
            "startup_check_timeout": 0.5, "startup_timeout": 20.0,
            "navigation_timeout": 120.0, "approach_timeout": 90.0,
            "skill_timeout": 20.0, "server_timeout": 20.0,
            "localization_timeout": 5.0, "cancel_timeout": 8.0,
            "stop_timeout": 8.0, "handoff_timeout": 5.0,
            "skill_safe_timeout": 5.0, "relocalize_timeout": 15.0,
            "max_navigation_retries": 1, "max_relocalization_retries": 1,
            "odom_timeout": 0.5, "tf_timeout": 0.5, "health_timeout": 1.0,
            "clock_timeout": 1.0, "localization_loss_grace": 1.0,
            "control_state_timeout": 0.5, "control_lease_duration": 2.0,
            "stable_linear_velocity": 0.03, "stable_angular_velocity": 0.05,
            "stable_duration": 1.0, "verify_stable_duration": 0.5,
            "verify_max_sample_gap": 0.4, "verify_minimum_samples": 3,
            "accepted_pose_topic": "", "accepted_pose_max_age": 4.0,
            "accepted_pose_type": "covariance",
            "accepted_pose_body_frame": "livox_frame", "accepted_pose_source_id": "",
            "approach_xy_tolerance": 0.10, "approach_yaw_tolerance": 0.10,
            "mission_time_limit": 180.0,
            "wait_at_waypoints": True, "precise_waypoints": True,
            "waypoint_xy_tolerance": 0.10, "waypoint_yaw_tolerance": 0.10,
            "max_waypoint_alignment_retries": 1,
            "simulation_mode": False, "autostart": False, "use_sim_time": False,
        }
        for name, value in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        for name, default in defaults.items():
            current = value(name)
            if isinstance(default, bool) and not isinstance(current, bool):
                raise ValueError(f"{name} must be a boolean")
            if isinstance(default, float):
                if (isinstance(current, bool) or not isinstance(current, (float, int)) or
                    not math.isfinite(current) or
                    (current < 0 if name == "mission_time_limit" else current <= 0)):
                    raise ValueError(f"{name} has invalid range/type")
            if name in ("max_navigation_retries", "max_relocalization_retries",
                        "max_waypoint_alignment_retries"):
                if isinstance(current, bool) or not isinstance(current, int) or current < 0:
                    raise ValueError(f"{name} must be a nonnegative integer")
        if value("control_lease_duration") < 3 * value("control_state_timeout"):
            raise ValueError("control_lease_duration must be >= 3 * control_state_timeout")
        if value("verify_max_sample_gap") > value("tf_timeout"):
            raise ValueError("verify_max_sample_gap must be <= tf_timeout")
        for name in ("approach_yaw_tolerance", "waypoint_yaw_tolerance"):
            if value(name) > math.pi:
                raise ValueError(f"{name} must be <= pi")
        self.route = RouteManager.load(value("route_file"))
        if any(task.type == "obstacle" and task.verify_timeout <=
               value("verify_stable_duration") for task in self.route.tasks):
            raise ValueError("verify_timeout must exceed verify_stable_duration")
        self.fsm = MissionFSM(
            self.route, value("navigation_timeout"), value("approach_timeout"),
            value("skill_timeout"), value("max_navigation_retries"),
            value("simulation_mode"), value("handoff_timeout"),
            value("skill_safe_timeout"), value("localization_timeout"),
            value("relocalize_timeout"), value("max_relocalization_retries"),
            wait_at_waypoints=value("wait_at_waypoints"),
            precise_waypoints=value("precise_waypoints"),
            max_waypoint_alignment_retries=value("max_waypoint_alignment_retries"),
            enable_relocalization=value("enable_relocalization"))
        self.stop_monitor = StopMonitor(
            self, value("odom_timeout"), value("stable_linear_velocity"),
            value("stable_angular_velocity"), value("stable_duration"))
        self.localization = LocalizationClient(
            self, value("localization_ok_service"), self.route.frame_id,
            value("base_frame"), value("tf_timeout"), value("clock_timeout"),
            value("use_sim_time"), value("health_timeout"),
            value("accepted_pose_topic"), value("accepted_pose_max_age"),
            value("accepted_pose_type"), value("accepted_pose_body_frame"),
            value("accepted_pose_source_id"))
        self.handoff = ControlHandoff(
            self, value("control_service"), value("control_state_topic"),
            value("control_state_timeout"), value("control_lease_duration"))
        self.navigation = NavigationClient(
            self, value("nav_action_name"), installed_trees(value("transit_bt_file"),
            value("approach_bt_file")), value("server_timeout"), value("cancel_timeout"))
        self.obstacle = ObstacleClient(self, value("obstacle_action_name"),
                                       value("server_timeout"), value("cancel_timeout"))
        self.relocalization = (RelocalizationClient(
            self, value("relocalization_action_name"), self.route.frame_id,
            value("server_timeout"), value("cancel_timeout"))
            if value("enable_relocalization") else None)
        self.action_runners = tuple(r for r in (self.navigation, self.obstacle, self.relocalization) if r is not None)
        self.motion_state = MotionStateClient(
            self, value("motion_state_service"), self.stop_monitor.ready,
            value("verify_stable_duration"))
        self.verifier = EndPoseVerifier(
            self.localization.sample, self.localization.healthy, self._end_stopped,
            value("verify_stable_duration"),
            max_sample_gap=value("verify_max_sample_gap"),
            minimum_samples=value("verify_minimum_samples"))
        self.status = MissionStatusPublisher(self)
        self.stop_timeout = value("stop_timeout")
        self.localization_loss_grace = value("localization_loss_grace")
        self.approach_xy_tolerance = value("approach_xy_tolerance")
        self.approach_yaw_tolerance = value("approach_yaw_tolerance")
        self.waypoint_xy_tolerance = value("waypoint_xy_tolerance")
        self.waypoint_yaw_tolerance = value("waypoint_yaw_tolerance")
        self.waypoint_errors = None
        self.mission_time_limit = value("mission_time_limit")
        self.mission_deadline = None
        self.stop_deadline = self.tf_bad_since = None
        self.closing = self.started = self.safety_cleanup = False
        self.autostart = value("autostart")
        self.startup_check_service = value("startup_check_service")
        self.startup_check_max_age = value("startup_check_max_age")
        self.startup_check_timeout = value("startup_check_timeout")
        self.startup_deadline = time.monotonic() + value("startup_timeout")
        self.startup_last_poll = self.startup_result_at = 0.0
        self.startup_result_positive = False
        self.startup_reason = "startup precheck not yet confirmed"
        self.startup_timed_out = False
        self.startup_last_ros = None
        self.startup_token = 0
        self.startup_runner = (AsyncServiceRunner(self.create_client(Trigger, self.startup_check_service))
                               if self.startup_check_service else None)
        self.startup_status = self.create_publisher(String, "/mission/startup_status", 10)
        self.start_service = self.create_service(Trigger, "/mission/start", self._start_service)
        self.supervision_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self._tick, clock=self.supervision_clock)
        self.status_timer = self.create_timer(0.5, self._publish, clock=self.supervision_clock)
        self._publish()
        self.get_logger().info(
            f"Mission ready: tasks={len(self.route.tasks)} base={value('base_frame')} "
            f"simulation_mode={value('simulation_mode')}; arbiter confirmation required")

    def _publish(self):
        feedback = (self.obstacle.last_feedback
                    if self.fsm.state == State.EXECUTING_SKILL
                    else self.navigation.last_feedback)
        self.status.publish(self.fsm, feedback, self.handoff.owner,
                            self.remaining_time(), self.verifier.last_errors,
                            self.waypoint_errors)

    def remaining_time(self):
        return (None if self.mission_deadline is None else
                max(0.0, self.mission_deadline - time.monotonic()))

    def _start_service(self, _request, response):
        if self.started:
            response.success, response.message = False, "mission already started"
        else:
            accepted = self._start()
            response.success = accepted
            response.message = "mission started" if accepted else self.startup_reason
        return response

    def _startup_ready(self):
        now = self.get_clock().now().nanoseconds
        return (not self.startup_check_service or
                (now > 0 and (self.startup_last_ros is None or now >= self.startup_last_ros)
                 and self.startup_result_positive and
                 time.monotonic() - self.startup_result_at <= self.startup_check_max_age))

    def _startup_result(self, _token, outcome, terminal, result):
        if self.started or self.closing:
            return
        self.startup_result_at = time.monotonic()
        self.startup_result_positive = bool(outcome == "succeeded" and terminal and
                                            result is not None and result.success)
        self.startup_reason = (result.message if result is not None else
                               "startup precheck " + outcome)
        self._publish_startup()

    def _publish_startup(self):
        self.startup_status.publish(String(data=json.dumps(dict(
            configured=bool(self.startup_check_service), ready=self._startup_ready(),
            started=self.started, autostart=self.autostart, timed_out=self.startup_timed_out, reason=self.startup_reason,
            remaining=max(0.0, self.startup_deadline-time.monotonic())))))

    def _poll_startup(self):
        if self.started or self.startup_runner is None:
            return
        now = self.get_clock().now().nanoseconds
        if self.startup_last_ros is not None and now < self.startup_last_ros:
            self.startup_runner.cancel()
            self.startup_result_positive = False
            self.startup_result_at = 0.0
            self.startup_reason = "startup precheck invalidated by clock rollback"
        self.startup_last_ros = now
        self.startup_runner.tick()
        wall = time.monotonic()
        if (self.startup_runner.pending is None and
                wall-self.startup_last_poll >= min(0.2,self.startup_check_max_age/3.0)):
            self.startup_last_poll = wall
            self.startup_token += 1
            self.startup_runner.start(self.startup_token, Trigger.Request(),
                self.startup_check_timeout, self._startup_result,
                lambda result: bool(result is not None and result.success))
            self.startup_runner.tick()
        self._publish_startup()

    def _start(self):
        if self.started or self.closing:
            return False
        if not self._startup_ready():
            if self.startup_result_positive:
                self.startup_reason = "startup precheck positive result expired"
            self._publish_startup()
            return False
        self.started = True
        self.startup_timed_out = False
        if self.startup_runner is not None:
            self.startup_runner.cancel()
        if self.mission_time_limit:
            self.mission_deadline = time.monotonic() + self.mission_time_limit
        self._dispatch(self.fsm.start())
        self._publish_startup()
        return True

    def _actions_terminal(self):
        return all(r.active is None and not r.unknown_terminal
                   for r in self.action_runners)

    def _pose_matches(self, target, xy, yaw):
        sample = self.localization.sample()
        if sample is None:
            return False
        distance, heading = pose_errors(target, sample.pose)
        return distance <= xy and heading <= yaw

    def _end_stopped(self):
        task = self.route.current
        return (task is not None and self.stop_monitor.ready() and
                self.motion_state.safe_recent(task.id))

    def _handoff_guard(self, command):
        if not self._actions_terminal():
            return False
        if command.mode == "SAFE_HOLD":
            return self.fsm.skill_safe or not self.fsm.skill_completed
        if not self.stop_monitor.ready() or not self.localization.healthy():
            return False
        task = self.route.current
        if command.mode == "SKILL":
            return (self._pose_matches(task.approach_pose, self.approach_xy_tolerance,
                                       self.approach_yaw_tolerance))
        if self.fsm.end_verified:
            return (self.fsm.skill_safe and self._end_stopped() and
                    self._pose_matches(task.end_pose, task.end_xy_tolerance,
                                       task.end_yaw_tolerance))
        return True

    def _abort(self, reason):
        if self.fsm.state != State.ABORTED:
            self.get_logger().error(
                f"task={self.route.current.id if self.route.current else 'return'} "
                f"state={self.fsm.state.value} command={self.fsm.pending} reason={reason}")
            self.fsm.abort(reason)
        self._cleanup()
        self._publish()

    def _cleanup(self):
        if self.safety_cleanup:
            return
        self.safety_cleanup = True
        for runner in self.action_runners:
            runner.request_cancel("mission_stopped")
        self.localization.cancel()
        self.verifier.cancel()
        self.motion_state.cancel()
        self.handoff.safe_hold()

    def _dispatch(self, command):
        if self.closing:
            return
        if self.fsm.state in (State.FINISHED, State.ABORTED):
            self._cleanup()
        if command is None:
            self._publish()
            return
        remaining = self.remaining_time()
        if remaining is not None:
            if remaining <= 0:
                self._abort("competition time budget exhausted")
                return
            command = replace(command, timeout=min(command.timeout, remaining))
            self.fsm.pending = command
        self.get_logger().info(
            f"state={self.fsm.state.value} command={command.kind} task={command.task_id} "
            f"mode={command.mode} token={command.token} timeout={command.timeout:.3f}")
        try:
            if command.kind == "navigation":
                if command.mode == "transit":
                    self.waypoint_errors = None
                if not self._actions_terminal() or not self.handoff.confirmed("NAVIGATION"):
                    self._abort("navigation ownership or Action terminal state unconfirmed")
                elif not self.localization.healthy():
                    self._on_action(command.token, "localization_lost", True, None)
                else:
                    self.navigation.navigate(command, self.route.frame_id, self._on_action)
            elif command.kind == "handoff":
                if command.mode == "SKILL" and not self.handoff.confirmed("NAVIGATION"):
                    self._abort("SKILL handoff requires confirmed NAVIGATION owner")
                else:
                    self.handoff.switch(command, self._on_action,
                                        lambda: self._handoff_guard(command))
            elif command.kind == "skill":
                if (not self._actions_terminal() or not self.handoff.confirmed("SKILL") or
                    not self.stop_monitor.ready() or not self.localization.healthy()):
                    self._abort("skill preconditions/ownership unconfirmed")
                else:
                    self.obstacle.execute(command, self._on_action)
            elif command.kind == "skill_safe":
                self.stop_monitor.reset()
                self.motion_state.start(command, self._on_action)
            elif command.kind == "localization":
                self.localization.require_new_accepted_pose()
                self.localization.check(command.token, command.timeout, self._on_action)
            elif command.kind == "verify_end":
                self.verifier.start(command, self._on_action)
            elif command.kind == "relocalize":
                if self.relocalization is None:
                    self._abort("relocalization disabled")
                elif not self._actions_terminal() or not self.handoff.confirmed("SAFE_HOLD"):
                    self._abort("relocalization requires confirmed SAFE_HOLD")
                else:
                    self.relocalization.execute(command, self._on_action)
            else:
                raise RuntimeError(f"unknown command: {command.kind}")
        except Exception as error:
            self._abort(f"adapter exception: {error}")
        self._publish()

    def _on_action(self, token, outcome, terminal, result):
        if self.closing or self.fsm.pending is None or self.fsm.pending.token != token:
            return
        if self.remaining_time() == 0:
            self._abort("competition time budget exhausted before result acceptance")
            return
        command = self.fsm.pending
        self.get_logger().info(
            f"state={self.fsm.state.value} task={command.task_id} command={command.kind} "
            f"token={token} outcome={outcome} terminal={terminal} result={result}")
        if command.kind == "relocalize" and outcome == "succeeded":
            if not getattr(result, "localized", False) or getattr(result, "error_code", ""):
                outcome = "localization_failed"
            else:
                self.localization.invalidate(require_new_tf=True)
        if command.kind == "navigation" and outcome == "succeeded" and not self.localization.healthy():
            outcome = "localization_lost"
        next_command = self.fsm.result(
            token, outcome, terminal, bool(getattr(result, "completed", False)),
            bool(getattr(result, "control_released", False)),
            bool(getattr(result, "safe_stance", False)), getattr(result, "error_code", ""),
            getattr(result, "owner", ""), result)
        if self.fsm.state in (State.WAIT_STOP, State.WAIT_WAYPOINT_STOP):
            self.stop_deadline = None
            self.stop_monitor.reset()
            self.localization.require_new_accepted_pose()
            self.get_logger().info(
                f"state={self.fsm.state.value} task={command.task_id} "
                "waiting for fresh localization and continuous measured stop")
        self._dispatch(next_command)

    def _wait_for_stop(self):
        self.stop_deadline = self.stop_deadline or time.monotonic() + self.stop_timeout
        if not self.localization.health_ok or not self.localization.tf_fresh():
            self._abort("localization untrusted while waiting for stop")
        elif self.stop_monitor.ready() and self.localization.healthy():
            if not self._actions_terminal():
                self._abort("Action terminal state unconfirmed while waiting for stop")
            elif (self.fsm.state == State.WAIT_STOP and
                  not self._pose_matches(self.route.current.approach_pose,
                                         self.approach_xy_tolerance,
                                         self.approach_yaw_tolerance)):
                self._abort("approach pose outside entry tolerance")
            elif (self.fsm.state == State.WAIT_WAYPOINT_STOP and
                  self.fsm.precise_waypoints):
                sample = self.localization.sample()
                if sample is None:
                    self._abort("waypoint pose sample unavailable")
                    return
                distance, heading = pose_errors(self.fsm.waypoint_target(), sample.pose)
                if not all(math.isfinite(v) for v in (distance, heading)):
                    self._abort("waypoint pose error is non-finite")
                    return
                self.waypoint_errors = {"distance": distance, "yaw_error": heading}
                self.stop_deadline = None
                if (distance > self.waypoint_xy_tolerance or
                        heading > self.waypoint_yaw_tolerance):
                    reason = f"waypoint pose outside tolerance: xy={distance:.4f} yaw={heading:.4f}"
                    self.get_logger().warning(reason)
                    self._dispatch(self.fsm.retry_waypoint_alignment(reason))
                else:
                    self.get_logger().info(
                        f"waypoint pose accepted task={self.route.current.id if self.route.current else 'return'} "
                        f"xy={distance:.4f} yaw={heading:.4f} "
                        f"realignment_retries={self.fsm.waypoint_alignment_attempts}")
                    self._publish()  # Publish this point's measured errors before advancing.
                    self._dispatch(self.fsm.stopped())
            else:
                self.stop_deadline = None
                self._dispatch(self.fsm.stopped())
        elif time.monotonic() >= self.stop_deadline:
            self._abort("odometry stale, robot did not stop or no new accepted pose")

    def _tick(self):
        if self.closing:
            return
        self._poll_startup()
        if self.autostart and not self.started:
            if self._startup_ready():
                self._start()
            elif time.monotonic() >= self.startup_deadline:
                self.autostart = False
                self.startup_timed_out = True
                self.startup_reason = "startup_timeout: " + self.startup_reason
                self.get_logger().error(self.startup_reason)
                self._publish_startup()
        if (self.started and self.remaining_time() == 0 and
            self.fsm.state not in (State.FINISHED, State.ABORTED)):
            self._abort("competition time budget exhausted")
        self.handoff.tick()
        self.navigation.tick()
        self.obstacle.tick()
        if self.relocalization is not None:
            self.relocalization.tick()
        self.localization.tick()
        self.motion_state.tick()
        self.verifier.tick()
        if self.fsm.state in (State.FINISHED, State.ABORTED):
            self._cleanup()
            return
        if self.fsm.state in (State.NAVIGATING, State.APPROACHING, State.RETURNING,
                              State.APPROACHING_WAYPOINT, State.APPROACHING_RETURN,
                              State.WAIT_STOP, State.WAIT_WAYPOINT_STOP,
                              State.EXECUTING_SKILL, State.WAIT_SKILL_SAFE,
                              State.VERIFYING_END, State.RELOCALIZING):
            owner = ("SKILL" if self.fsm.state in (State.EXECUTING_SKILL,
                     State.WAIT_SKILL_SAFE) else "SAFE_HOLD" if self.fsm.state in
                     (State.VERIFYING_END, State.RELOCALIZING) else "NAVIGATION")
            if self.handoff.failure or not self.handoff.confirmed(owner):
                self._abort(f"control ownership {owner} stale, lost or unknown")
                return
        if self.fsm.state in (State.NAVIGATING, State.APPROACHING, State.RETURNING,
                              State.APPROACHING_WAYPOINT, State.APPROACHING_RETURN,
                              State.VERIFYING_END):
            if self.localization.healthy():
                self.tf_bad_since = None
            else:
                self.tf_bad_since = self.tf_bad_since or time.monotonic()
                if time.monotonic() - self.tf_bad_since >= self.localization_loss_grace:
                    if self.fsm.state == State.VERIFYING_END:
                        command = self.fsm.localization_lost()
                        self.verifier.cancel()  # stale token is now ignored
                        self._dispatch(command)
                    else:
                        self.navigation.request_cancel("localization_lost")
        else:
            self.tf_bad_since = None
        if self.fsm.state in (State.WAIT_STOP, State.WAIT_WAYPOINT_STOP):
            self._wait_for_stop()
        else:
            self.stop_deadline = None

    def close_safely(self):
        self.closing = True
        if self.startup_runner is not None:
            self.startup_runner.cancel()
        self._cleanup()
        deadline = time.monotonic() + float(self.get_parameter("cancel_timeout").value)
        # Only shutdown drains callbacks; task execution itself never waits/spins.
        while rclpy.ok() and time.monotonic() < deadline:
            active = any(r.active for r in self.action_runners)
            if not active and self.handoff.emergency.pending is None:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
            for runner in self.action_runners:
                runner.tick()
            self.handoff.tick()
        if not self._actions_terminal():
            self.get_logger().error("shutdown: Action terminal state unconfirmed")


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = MissionManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_safely()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

