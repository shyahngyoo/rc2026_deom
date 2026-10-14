"""Non-blocking, hardware-independent port of Hurricane obstacle designs.

Positions in the YAML remain in Hurricane's course coordinates.  ``trans.py``
uses the right normal of the course +x axis for +y; ``info_transmiting.py``
negates its yaw before calling the original handlers.  Consequently both
source y and source yaw must be reflected into a ROS right-handed frame:
``local_ros = (source_x, -source_y, -radians(source_yaw))``.  The calibrated
``origin`` is a ROS map pose whose yaw defines the *course +x axis*, not the
robot's heading at entry.  A null target component is captured once when its
waypoint starts, rather than following the robot on every pose callback.
``require_yaw: false`` explicitly disables final heading acceptance, as used
by Hurricane's first bridge-A waypoint whose yaw target is None.  Other null
yaw targets retain the new, deliberate entry-heading lock.

This module never treats a serial write or an elapsed jump delay as success.
Mode changes and jumps have separate, numbered feedback gates.  Terminal
index ``len(steps)`` has its own stand command and safe-stance confirmation.
Every jump also has a distinct preceding mode preparation/confirmation step,
including repeated modes.  Some original handlers immediately sent stand
after jump; that unverified wire sequence is replaced by trusted completion
and landing feedback, rather than assuming stand cannot interrupt a jump.
Control ownership/release and telemetry authenticity belong to the ROS
adapter, which must also verify them before reporting Action success.
"""

from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Any
import uuid

import yaml

from .hurricane_protocol import COMMAND_MAP


TERMINAL_STATUSES = frozenset({"completed", "failed", "canceled"})
ACTIVE_STATUSES = frozenset({"running", "awaiting_feedback", "awaiting_safe_stance"})


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be a finite number")
    return result


def _positive(value: Any, label: str, *, zero: bool = False) -> float:
    result = _finite(value, label)
    if result < 0.0 or (result == 0.0 and not zero):
        raise ValueError(f"{label} must be {'nonnegative' if zero else 'positive'}")
    return result


def _pose(value: Any, label: str, *, nullable: bool = False) -> list:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError(f"{label} must contain x, y and yaw")
    return [None if item is None and nullable else _finite(item, label)
            for item in value]


def _repeat(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 100:
        raise ValueError(f"{label} must be an integer in [1, 100]")
    return value


def _normalize_skill(skill_id: str, source: Any) -> dict:
    if not isinstance(source, dict):
        raise ValueError(f"skill {skill_id} must be a mapping")
    skill = copy.deepcopy(source)
    if not isinstance(skill.get("calibrated"), bool):
        raise ValueError(f"skill {skill_id} requires calibrated: true/false")
    skill["origin"] = _pose(skill.get("origin"), f"{skill_id}.origin")
    steps = skill.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError(f"skill {skill_id} requires nonempty steps")
    for index, step in enumerate(steps):
        label = f"{skill_id}.steps[{index}]"
        if not isinstance(step, dict):
            raise ValueError(f"{label} must be a mapping")
        kind = step.get("type")
        if kind not in {"mode", "waypoint", "jump"}:
            raise ValueError(f"{label}.type must be mode, waypoint or jump")
        if "timeout" in step:
            step["timeout"] = _positive(step["timeout"], f"{label}.timeout")
        if kind == "waypoint":
            step["target"] = _pose(step.get("target"), f"{label}.target", nullable=True)
            if not isinstance(step.get("require_yaw", True), bool):
                raise ValueError(f"{label}.require_yaw must be a bool")
            step["require_yaw"] = step.get("require_yaw", True)
            step["position_tolerance"] = _positive(
                step.get("position_tolerance", 0.05), f"{label}.position_tolerance")
            for key, default in (("yaw_tolerance_deg", 4.0), ("heading_tolerance_deg", 8.0)):
                step[key] = _positive(step.get(key, default), f"{label}.{key}")
                if step[key] >= 180.0:
                    raise ValueError(f"{label}.{key} must be below 180 degrees")
            if step.get("drive_direction", "forward") not in {"forward", "reverse"}:
                raise ValueError(f"{label}.drive_direction must be forward or reverse")
            step["drive_direction"] = step.get("drive_direction", "forward")
            step["repeat"] = _repeat(step.get("repeat", 4), f"{label}.repeat")
        else:
            command = step.get("command")
            if not isinstance(command, str) or command not in COMMAND_MAP:
                raise ValueError(f"{label}.command is not in Hurricane command map")
            if kind == "mode" and not command.endswith("模式"):
                raise ValueError(f"{label}.command must be a mode")
            if kind == "jump" and command not in {"小跳1", "小跳2"}:
                raise ValueError(f"{label}.command must be 小跳1 or 小跳2")
            step["min_wait"] = _positive(
                step.get("min_wait", 1.0 if kind == "mode" else 5.0),
                f"{label}.min_wait", zero=True)
            step["repeat"] = _repeat(step.get("repeat", 2 if kind == "mode" else 1),
                                     f"{label}.repeat")
    # Extra calibration/provenance fields are intentionally preserved.
    return skill


def load_skills(path: str | Path) -> dict:
    """Read and validate YAML; return the skill-id -> skill mapping."""
    with Path(path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if (not isinstance(document, dict) or type(document.get("version")) is not int
            or document.get("version") != 1):
        raise ValueError("skills document requires version: 1")
    if document.get("coordinate_convention") != "hurricane_reflected_y":
        raise ValueError("unsupported coordinate_convention")
    skills = document.get("skills")
    if not isinstance(skills, dict) or not skills:
        raise ValueError("skills document requires nonempty skills")
    result = {}
    for skill_id, skill in skills.items():
        if not isinstance(skill_id, str) or not skill_id.strip():
            raise ValueError("skill ids must be nonempty strings")
        result[skill_id] = _normalize_skill(skill_id, skill)
    return result


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class SkillRunner:
    """Tick-driven skill execution using ``transport.send(command, repeat)``.

    ``pose`` is a newly received ROS map ``(x, y, yaw_radians)``.  Pass None
    when no fresh observation is available; after pose_timeout all active
    phases stop and fail, including jumps and confirmation waits.  Cached
    observations must not be repeatedly presented as fresh by an adapter.

    ``feedback`` only accepts the active session and exact current step.
    Every mode/jump is a distinct step; the final stand gate uses index
    len(steps).  A jump also requires safe_stance.  Terminal confirmation is
    independent from any preceding feedback and never releases ownership
    itself.  The adapter must supply trusted, fresh hardware feedback.
    """

    def __init__(self, skills: dict, transport: Any, pose_timeout: float = 0.5,
                 step_timeout: float = 30.0, confirmation_timeout: float = 10.0,
                 allow_uncalibrated: bool = False):
        if not isinstance(skills, dict) or not skills:
            raise ValueError("skills must be a nonempty mapping")
        self.skills = {key: _normalize_skill(key, value) for key, value in skills.items()}
        if not callable(getattr(transport, "send", None)):
            raise ValueError("transport requires send(command, repeat)")
        self.transport = transport
        self.pose_timeout = _positive(pose_timeout, "pose_timeout")
        self.step_timeout = _positive(step_timeout, "step_timeout")
        self.confirmation_timeout = _positive(confirmation_timeout, "confirmation_timeout")
        if not isinstance(allow_uncalibrated, bool):
            raise ValueError("allow_uncalibrated must be a bool")
        self.allow_uncalibrated = allow_uncalibrated
        self.status = "idle"
        self.index = 0
        self.session_id = ""
        self.obstacle_id = ""
        self.skill_id = ""
        self.error = ""
        self._skill: dict | None = None
        self._started_at = 0.0
        self._last_tick = 0.0
        self._last_pose_at: float | None = None
        self._last_pose: tuple | None = None
        self._phase_started: float | None = None
        self._confirmation: tuple | None = None
        self._target: tuple | None = None
        self._last_command: str | None = None

    @property
    def progress(self) -> float:
        if self.status == "completed":
            return 1.0
        return self.index / (len(self._skill["steps"]) + 1) if self._skill else 0.0

    @property
    def step_type(self) -> str:
        if self._skill is None:
            return ""
        if self.index >= len(self._skill["steps"]):
            return "safe_stance"
        return self._skill["steps"][self.index]["type"]

    @property
    def target(self) -> tuple | None:
        """Resolved current waypoint in ROS map coordinates."""
        return self._target

    def start(self, skill_id: str, now: float, session_id: str = "",
              obstacle_id: str = "") -> None:
        now = _finite(now, "now")
        if self.status in ACTIVE_STATUSES:
            raise RuntimeError("a skill is already running")
        if skill_id not in self.skills:
            raise ValueError(f"unknown skill: {skill_id}")
        skill = self.skills[skill_id]
        if not skill["calibrated"] and not self.allow_uncalibrated:
            raise ValueError(f"skill {skill_id} requires field calibration")
        if not isinstance(session_id, str) or not isinstance(obstacle_id, str):
            raise ValueError("session_id and obstacle_id must be strings")
        self._skill = skill
        self.skill_id = skill_id
        self.obstacle_id = obstacle_id
        self.session_id = session_id or uuid.uuid4().hex
        self.status = "running"
        self.index = 0
        self.error = ""
        self._started_at = now
        self._last_tick = now
        self._last_pose_at = None
        self._last_pose = None
        self._phase_started = None
        self._confirmation = None
        self._target = None
        self._last_command = None

    def _stop(self) -> None:
        try:
            self.transport.send("站立", repeat=4)
        except Exception as exc:
            if not self.error:
                self.error = f"transport_error: {exc}"
        self._last_command = "站立"

    def _fail(self, reason: str) -> str:
        self.error = reason
        self.status = "failed"
        self._confirmation = None
        self._stop()
        return self.status

    def _send(self, command: str, repeat: int = 1, *, force: bool = False) -> bool:
        if command == self._last_command and not force:
            return True
        try:
            self.transport.send(command, repeat=repeat)
        except Exception as exc:
            self._fail(f"transport_error: {exc}")
            return False
        self._last_command = command
        return True

    def cancel(self, now: float) -> None:
        _finite(now, "now")
        self._stop()
        self.status = "canceled"
        self._confirmation = None

    def feedback(self, session_id: str, step_index: int, completed: bool,
                 safe_stance: bool, now: float) -> bool:
        try:
            now = _finite(now, "feedback.now")
        except ValueError:
            return False
        if (self.status not in {"awaiting_feedback", "awaiting_safe_stance"}
                or session_id != self.session_id or isinstance(step_index, bool)
                or not isinstance(step_index, int) or step_index != self.index
                or self._phase_started is None or now < self._phase_started
                or not isinstance(completed, bool) or not isinstance(safe_stance, bool)):
            return False
        if self._confirmation is not None and now < self._confirmation[2]:
            return False
        if not completed:
            self._fail(f"step_{self.index}_rejected")
            return True
        self._confirmation = (completed, safe_stance, now)
        return True

    def clear_confirmation(self) -> None:
        """Drop an expired adapter observation without inventing a new result."""
        self._confirmation = None

    def _resolve_target(self, pose: tuple, source: list) -> tuple:
        ox, oy, oyaw = self._skill["origin"]
        cosine, sine = math.cos(oyaw), math.sin(oyaw)
        dx, dy = pose[0] - ox, pose[1] - oy
        current_source = (cosine * dx + sine * dy,
                          sine * dx - cosine * dy,
                          -math.degrees(_wrap(pose[2] - oyaw)))
        sx, sy, syaw = [current_source[i] if value is None else value
                        for i, value in enumerate(source)]
        # Source +y and source +yaw are both right-handed ROS reflections.
        return (ox + cosine * sx + sine * sy,
                oy + sine * sx - cosine * sy,
                _wrap(oyaw - math.radians(syaw)))

    def _advance(self, now: float) -> None:
        self.index += 1
        self._target = None
        self._confirmation = None
        self._phase_started = None
        self.status = "running"
        if self.index == len(self._skill["steps"]):
            self._phase_started = now
            if self._send("站立", repeat=4, force=True):
                self.status = "awaiting_safe_stance"

    def tick(self, pose: tuple | None, now: float) -> str:
        now = _finite(now, "now")
        if self.status not in ACTIVE_STATUSES:
            return self.status
        if now < self._last_tick:
            return self._fail("clock_regression")
        self._last_tick = now
        if pose is not None:
            try:
                self._last_pose = tuple(_pose(pose, "pose"))
            except ValueError:
                return self._fail("pose_invalid")
            self._last_pose_at = now
        freshness_origin = self._started_at if self._last_pose_at is None else self._last_pose_at
        if now - freshness_origin > self.pose_timeout:
            return self._fail("pose_stale")
        if self._last_pose is None:
            return self.status

        if self.status == "awaiting_safe_stance":
            if now - self._phase_started > self.confirmation_timeout:
                return self._fail("safe_stance_confirmation_timeout")
            if (self._confirmation is not None and self._confirmation[1]
                    and self._confirmation[2] <= now):
                self.status = "completed"
            return self.status

        # One tick can complete at most one configured step.  In particular,
        # it never processes a second waypoint using the first one's pose.
        step = self._skill["steps"][self.index]
        kind = step["type"]
        if self._phase_started is None:
            self._phase_started = now
            self._confirmation = None
            if kind == "waypoint":
                self._target = self._resolve_target(self._last_pose, step["target"])
            else:
                if not self._send(step["command"], repeat=step["repeat"], force=True):
                    return self.status
                self.status = "awaiting_feedback"
                return self.status

        elapsed = now - self._phase_started
        if elapsed > step.get("timeout", self.step_timeout):
            return self._fail(f"step_{self.index}_timeout")
        if kind in {"mode", "jump"}:
            if elapsed > step["min_wait"] + self.confirmation_timeout:
                return self._fail(f"step_{self.index}_confirmation_timeout")
            landed = kind != "jump" or (self._confirmation is not None
                                         and self._confirmation[1])
            if (self._confirmation is not None and self._confirmation[2] <= now
                    and landed and elapsed >= step["min_wait"]):
                self._advance(now)
            return self.status

        x, y, yaw = self._last_pose
        target_x, target_y, target_yaw = self._target
        distance = math.hypot(target_x - x, target_y - y)
        if distance > step["position_tolerance"]:
            desired_yaw = math.atan2(target_y - y, target_x - x)
            reverse = step["drive_direction"] == "reverse"
            if reverse:
                desired_yaw = _wrap(desired_yaw + math.pi)
            difference = _wrap(desired_yaw - yaw)
            if abs(difference) > math.radians(step["heading_tolerance_deg"]):
                self._send("左转" if difference > 0.0 else "右转", step["repeat"])
            else:
                self._send("后退" if reverse else "前进", step["repeat"])
            return self.status

        difference = _wrap(target_yaw - yaw)
        if step["require_yaw"] and abs(difference) > math.radians(step["yaw_tolerance_deg"]):
            self._send("左转" if difference > 0.0 else "右转", step["repeat"])
        elif self._send("站立", repeat=step["repeat"]):
            self._advance(now)
        return self.status
