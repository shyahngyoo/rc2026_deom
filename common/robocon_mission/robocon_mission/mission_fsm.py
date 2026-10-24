"""Pure FSM: one outstanding Command/token, explicit physical safety boundaries."""
from dataclasses import dataclass
from enum import Enum
from typing import Optional
from .mission_recovery import RecoveryPolicy
from .rule_validation import RuleEvidence, UnconfiguredRuleValidator


class State(str, Enum):
    IDLE = "IDLE"
    INITIALIZING = "INITIALIZING"
    WAIT_LOCALIZATION = "WAIT_LOCALIZATION"
    RELOCALIZING = "RELOCALIZING"
    SELECT_TASK = "SELECT_TASK"
    NAVIGATING = "NAVIGATING"
    APPROACHING = "APPROACHING"
    APPROACHING_WAYPOINT = "APPROACHING_WAYPOINT"
    APPROACHING_RETURN = "APPROACHING_RETURN"
    WAIT_STOP = "WAIT_STOP"
    WAIT_WAYPOINT_STOP = "WAIT_WAYPOINT_STOP"
    ACQUIRING_CONTROL = "ACQUIRING_CONTROL"
    EXECUTING_SKILL = "EXECUTING_SKILL"
    WAIT_SKILL_SAFE = "WAIT_SKILL_SAFE"
    RELEASING_CONTROL = "RELEASING_CONTROL"
    VERIFYING_END = "VERIFYING_END"
    VERIFYING = "VERIFYING_END"  # old name remains an alias
    RESTORING_NAV = "RESTORING_NAV"
    RETURNING = "RETURNING"
    RECOVERY = "RECOVERY"
    FINISHED = "FINISHED"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class Command:
    kind: str
    task_id: str = ""
    mode: str = ""
    pose: tuple = ()
    skill_id: str = ""
    timeout: float = 0.0
    token: int = 0
    xy_tolerance: float = 0.15
    yaw_tolerance: float = 0.20


class MissionFSM:
    def __init__(self, route, navigation_timeout, approach_timeout,
                 skill_timeout, max_navigation_retries, simulation_mode,
                 handoff_timeout=5.0, skill_safe_timeout=5.0,
                 localization_timeout=5.0, relocalize_timeout=15.0,
                 max_relocalization_retries=1, rule_validator=None,
                 wait_at_waypoints=False, precise_waypoints=False,
                 max_waypoint_alignment_retries=1, enable_relocalization=False):
        self.route, self.state = route, State.IDLE
        self.error = self.nav_mode = ""
        self.simulation_mode = simulation_mode  # never bypasses confirmation
        if precise_waypoints and not wait_at_waypoints:
            raise ValueError("precise_waypoints requires wait_at_waypoints")
        if (isinstance(max_waypoint_alignment_retries, bool) or
                not isinstance(max_waypoint_alignment_retries, int) or
                max_waypoint_alignment_retries < 0):
            raise ValueError("max_waypoint_alignment_retries must be a nonnegative integer")
        self.wait_at_waypoints = wait_at_waypoints
        self.precise_waypoints = precise_waypoints
        self.max_waypoint_alignment_retries = max_waypoint_alignment_retries
        self.waypoint_alignment_attempts = 0
        self.waiting_return_stop = False
        self.navigation_timeout = navigation_timeout
        self.approach_timeout = approach_timeout
        self.skill_timeout = skill_timeout
        self.handoff_timeout = handoff_timeout
        self.skill_safe_timeout = skill_safe_timeout
        self.localization_timeout = localization_timeout
        if not isinstance(enable_relocalization, bool):
            raise ValueError('enable_relocalization must be a boolean')
        self.enable_relocalization = enable_relocalization
        self.relocalize_timeout = relocalize_timeout
        self.max_relocalization_retries = max_relocalization_retries
        self.relocalization_attempts = 0
        self.recovery = RecoveryPolicy(max_navigation_retries)
        self.pending: Optional[Command] = None
        self.sequence = 0
        self.resume_state = State.SELECT_TASK
        self.control_owner = "UNKNOWN"
        self.skill_completed = self.skill_safe = self.end_verified = False
        self.rule_validator = rule_validator or UnconfiguredRuleValidator()
        self.rule_verdicts = {}

    def _emit(self, kind, **kwargs) -> Command:
        if self.pending is not None:
            raise RuntimeError("a command is already active")
        self.sequence += 1
        self.pending = Command(kind, token=self.sequence, **kwargs)
        return self.pending

    def abort(self, reason: str) -> None:
        self.pending = None
        self.sequence += 1
        self.error, self.state = reason, State.ABORTED

    def _handoff(self, owner, state) -> Command:
        self.state = state
        return self._emit("handoff", mode=owner,
                          task_id=self.route.current.id if self.route.current else "",
                          timeout=self.handoff_timeout)

    def _health(self, continuation) -> Command:
        self.resume_state, self.state = continuation, State.WAIT_LOCALIZATION
        return self._emit("localization", timeout=self.localization_timeout)

    def start(self) -> Command:
        if self.state != State.IDLE:
            raise RuntimeError("mission already started")
        return self._health(State.SELECT_TASK)

    def _navigate(self, mode, pose, state, timeout) -> Command:
        self.state, self.nav_mode = state, mode
        return self._emit("navigation", task_id=self.route.current.id
                          if self.route.current else "return",
                          mode=mode, pose=pose, timeout=timeout)

    def _select(self) -> Command:
        self.state, self.error = State.SELECT_TASK, ""
        self.skill_completed = self.skill_safe = self.end_verified = False
        self.relocalization_attempts = 0
        self.waypoint_alignment_attempts = 0
        return self._resume_navigation(State.NAVIGATING)

    def _resume_navigation(self, state) -> Command:
        task = self.route.current
        if state == State.APPROACHING_RETURN:
            return self._navigate("approach", self.route.return_pose, state,
                                  self.approach_timeout)
        if state == State.APPROACHING_WAYPOINT:
            return self._navigate("approach", task.pose, state, self.approach_timeout)
        if task is None or state == State.RETURNING:
            return self._navigate("transit", self.route.return_pose,
                                  State.RETURNING, self.navigation_timeout)
        if state == State.APPROACHING:
            return self._navigate("approach", task.approach_pose, state,
                                  task.approach_timeout or self.approach_timeout)
        return self._navigate("transit", task.pose if task.type == "navigation"
                              else task.transit_pose, State.NAVIGATING,
                              task.transit_timeout or task.timeout or
                              self.navigation_timeout)

    def _continue_localized(self) -> Command:
        if self.resume_state == State.VERIFYING_END:
            task = self.route.current
            self.state = State.VERIFYING_END
            return self._emit("verify_end", task_id=task.id, pose=task.end_pose,
                              timeout=task.verify_timeout,
                              xy_tolerance=task.end_xy_tolerance,
                              yaw_tolerance=task.end_yaw_tolerance)
        if self.control_owner != "NAVIGATION":
            return self._handoff("NAVIGATION", State.RESTORING_NAV)
        return (self._select() if self.resume_state == State.SELECT_TASK else
                self._resume_navigation(self.resume_state))

    def _relocalize(self) -> Optional[Command]:
        if not self.enable_relocalization:
            self.abort('localization unavailable; relocalization disabled; check fixed start and restart')
            return None
        if self.relocalization_attempts > self.max_relocalization_retries:
            self.abort("localization: autonomous relocalization retries exhausted")
            return None
        self.relocalization_attempts += 1
        self.state = State.RELOCALIZING
        return self._emit("relocalize", timeout=self.relocalize_timeout)

    def result(self, token: int, outcome: str, terminal: bool = True,
               completed: bool = False, control_released: bool = False,
               safe_stance: bool = False, error_code: str = "",
               confirmed_owner: str = "", result=None) -> Optional[Command]:
        """Old/duplicate tokens cannot advance a new task."""
        if self.pending is None or token != self.pending.token:
            return None
        cmd, self.pending = self.pending, None
        if not terminal:
            self.abort(f"{cmd.kind} result/cancellation unknown: {outcome}; {self.error}")
            return None
        if cmd.kind == "localization":
            if outcome != "succeeded":
                self.error = f"localization {outcome}"
                return (self._relocalize() if self.control_owner == "SAFE_HOLD" else
                        self._handoff("SAFE_HOLD", State.RELEASING_CONTROL))
            return self._continue_localized()
        if cmd.kind == "relocalize":
            if outcome != "succeeded":
                if outcome in ("server_unavailable", "rejected"):
                    self.abort(f"localization: relocalization not ready ({outcome})")
                    return None
                return self._relocalize()
            return self._health(self.resume_state)
        if cmd.kind == "handoff":
            if outcome != "succeeded" or confirmed_owner != cmd.mode:
                self.abort(f"control handoff {cmd.mode} {outcome}: unconfirmed owner; {self.error}")
                return None
            self.control_owner = confirmed_owner
            if cmd.mode == "SKILL":
                task = self.route.current
                self.state = State.EXECUTING_SKILL
                return self._emit("skill", task_id=task.id, skill_id=task.skill_id,
                                  timeout=task.skill_timeout or task.timeout or
                                  self.skill_timeout)
            if cmd.mode == "SAFE_HOLD":
                return (self._health(State.VERIFYING_END) if self.skill_safe else
                        self._relocalize())
            if self.end_verified:
                if not (self.skill_completed and self.skill_safe):
                    self.abort("incomplete obstacle safety evidence")
                    return None
                task_id = self.route.current.id
                try:
                    self.rule_verdicts[task_id] = self.rule_validator.validate(
                        RuleEvidence(task_id, True, True, True))
                except Exception as error:
                    self.abort(f"rule validator exception: {error}")
                    return None
                self.route.complete_current()
                return self._select()
            return (self._select() if self.resume_state == State.SELECT_TASK else
                    self._resume_navigation(self.resume_state))
        if cmd.kind == "navigation":
            navigation_state = self.state
            if outcome != "succeeded":
                self.error, self.state = f"{cmd.mode} navigation {outcome}", State.RECOVERY
                task = self.route.current
                if task and self.recovery.retry_navigation(task, cmd.mode):
                    check = self._health(navigation_state)
                    self.state = State.RECOVERY
                    return check
                self.abort(self.error)
                return None
            self.error = ""
            if self.state == State.RETURNING:
                if self.precise_waypoints:
                    return self._resume_navigation(State.APPROACHING_RETURN)
                if self.wait_at_waypoints:
                    self.waiting_return_stop = True
                    self.state, self.nav_mode = State.WAIT_WAYPOINT_STOP, ""
                    return None
                self.state = State.FINISHED
                return None
            if self.state in (State.APPROACHING_WAYPOINT, State.APPROACHING_RETURN):
                self.waiting_return_stop = self.state == State.APPROACHING_RETURN
                self.state, self.nav_mode = State.WAIT_WAYPOINT_STOP, ""
                return None
            task = self.route.current
            if cmd.mode == "transit" and task.type == "navigation" and self.precise_waypoints:
                return self._resume_navigation(State.APPROACHING_WAYPOINT)
            if cmd.mode == "transit" and task.type == "obstacle":
                return self._resume_navigation(State.APPROACHING)
            if cmd.mode == "approach":
                self.state, self.nav_mode = State.WAIT_STOP, ""
                return None
            if self.wait_at_waypoints:
                self.waiting_return_stop = False
                self.state, self.nav_mode = State.WAIT_WAYPOINT_STOP, ""
                return None
            self.route.complete_current()
            return self._select()
        if cmd.kind == "skill":
            if outcome != "succeeded" or not completed or error_code:
                self.abort(f"obstacle skill {outcome}: {error_code or 'not completed'}")
                return None
            if not control_released:
                self.abort("obstacle control not released by Motion Manager")
                return None
            if not safe_stance:
                self.abort("obstacle skill did not confirm safe stance")
                return None
            self.skill_completed, self.state = True, State.WAIT_SKILL_SAFE
            return self._emit("skill_safe", task_id=cmd.task_id,
                              timeout=self.skill_safe_timeout)
        if cmd.kind == "skill_safe":
            if outcome != "succeeded":
                self.abort(f"skill safe stance {outcome}")
                return None
            self.skill_safe = True
            return self._handoff("SAFE_HOLD", State.RELEASING_CONTROL)
        if cmd.kind == "verify_end":
            if outcome != "succeeded":
                self.abort(f"end pose verification {outcome}")
                return None
            self.end_verified = True
            return self._handoff("NAVIGATION", State.RESTORING_NAV)
        raise RuntimeError(f"unknown command: {cmd.kind}")

    def waypoint_target(self):
        """Original route pose, never a planner-adjusted endpoint."""
        if self.state != State.WAIT_WAYPOINT_STOP:
            return None
        return (self.route.return_pose if self.waiting_return_stop else
                self.route.current.pose)

    def retry_waypoint_alignment(self, reason: str) -> Optional[Command]:
        if self.state != State.WAIT_WAYPOINT_STOP or self.pending is not None:
            return None
        if (not self.precise_waypoints or
                self.waypoint_alignment_attempts >= self.max_waypoint_alignment_retries):
            self.abort(f"waypoint alignment retries exhausted: {reason}")
            return None
        self.waypoint_alignment_attempts += 1
        self.error = reason
        return self._resume_navigation(State.APPROACHING_RETURN if
                                       self.waiting_return_stop else State.APPROACHING_WAYPOINT)

    def stopped(self) -> Optional[Command]:
        if self.state == State.WAIT_WAYPOINT_STOP and self.pending is None:
            if self.waiting_return_stop:
                self.state = State.FINISHED
                return None
            self.route.complete_current()
            return self._select()
        if self.state != State.WAIT_STOP or self.pending is not None:
            return None
        if self.route.current.end_pose is None:
            self.abort("obstacle end_pose missing")
            return None
        return self._handoff("SKILL", State.ACQUIRING_CONTROL)

    def stop_failed(self, reason: str) -> None:
        if self.state in (State.WAIT_STOP, State.WAIT_WAYPOINT_STOP):
            self.abort(reason)

    def localization_lost(self) -> Optional[Command]:
        if self.state != State.VERIFYING_END:
            return None
        self.pending = None
        self.sequence += 1
        return self._health(State.VERIFYING_END)

