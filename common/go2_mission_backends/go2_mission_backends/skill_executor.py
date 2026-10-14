"""Replaceable skill execution contract and deterministic hardware-free executor."""
from dataclasses import dataclass
from typing import Protocol
import math
import yaml
from robocon_mission.route_manager import StrictLoader

MODES = {"success", "fail", "timeout", "cancel_unconfirmed", "crash", "release_failure", "wrong_exit"}


@dataclass(frozen=True)
class SkillConfig:
    skill_id: str
    executor: str
    mode: str = "success"
    duration: float = 1.0
    velocity: tuple = (0., 0., 0.)  # body vx, vy, wz
    execution_timeout: float = 5.0
    enabled: bool = True


@dataclass(frozen=True)
class SkillUpdate:
    phase: str
    progress: float
    velocity: tuple
    terminal: str = ""  # completed, canceled, failed; independently verify safe release.
    error_code: str = ""


class SkillExecutor(Protocol):
    """Real implementations must confirm cancel/close from their own backend.

    close=True means this executor stopped/released its output, not proof of
    physical stopping, stable support, control release or exit arrival.
    """
    def start(self, now: float) -> None: ...
    def tick(self, now: float) -> SkillUpdate: ...
    def cancel(self, now: float) -> bool: ...
    def close(self, now: float) -> bool: ...


def load_skills(path, allowed=()):
    if not path:
        return {}
    with open(path, encoding="utf-8") as handle:
        document = yaml.load(handle, Loader=StrictLoader)
    if not isinstance(document, dict) or not isinstance(document.get("skills"), dict):
        raise ValueError("skills_file must contain skills mapping")
    skills = {}
    for skill_id, item in document["skills"].items():
        if not isinstance(skill_id, str) or not skill_id or not isinstance(item, dict):
            raise ValueError("invalid skill mapping")
        executor = item.get("executor", "")
        mode = item.get("mode", "success")
        duration = item.get("duration", 1.0)
        timeout = item.get("execution_timeout", 5.0)
        velocity = item.get("velocity", [0., 0., 0.])
        enabled = item.get("enabled", True)
        if (not isinstance(executor, str) or not executor or mode not in MODES or
                not isinstance(enabled, bool) or not isinstance(velocity, (list, tuple)) or len(velocity) != 3):
            raise ValueError("invalid skill configuration: " + skill_id)
        if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v)
               for v in (duration, timeout, *velocity)) or duration <= 0 or timeout <= 0:
            raise ValueError("invalid skill numeric configuration: " + skill_id)
        if allowed and skill_id not in allowed:
            enabled = False
        skills[skill_id] = SkillConfig(skill_id, executor, mode, float(duration),
                                      tuple(float(v) for v in velocity), float(timeout), enabled)
    return skills


class TestSkillExecutor:
    """Explicitly simulation-only; it never sends to a hardware topic itself."""
    __test__ = False
    simulation_only = True

    def __init__(self, config, simulation_mode=False):
        if not simulation_mode:
            raise ValueError("TEST_EXECUTOR_FORBIDDEN_OUTSIDE_SIMULATION")
        self.config = config
        self.started = None
        self.cancel_confirmed = False
        self.closed = False

    def start(self, now):
        if self.started is not None:
            raise RuntimeError("executor already started")
        self.started = now

    def tick(self, now):
        if self.started is None or self.closed:
            raise RuntimeError("executor not running")
        if self.cancel_confirmed:
            return SkillUpdate("canceling", 0., (0., 0., 0.), "canceled")
        elapsed = max(0., now - self.started)
        progress = min(1., elapsed/self.config.duration)
        mode = self.config.mode
        if mode == "crash" and elapsed >= self.config.duration/2:
            raise RuntimeError("TEST_EXECUTOR_CRASH")
        if mode == "fail" and elapsed >= self.config.duration/2:
            return SkillUpdate("failed", progress, (0., 0., 0.), "failed", "SKILL_EXECUTION_FAILED")
        if mode not in ("timeout", "cancel_unconfirmed") and elapsed >= self.config.duration:
            return SkillUpdate("finishing", 1., (0., 0., 0.), "completed")
        velocity = (0., 0., 0.) if mode == "wrong_exit" else self.config.velocity
        return SkillUpdate("running", progress, velocity)

    def cancel(self, now):
        if self.config.mode == "cancel_unconfirmed":
            return False
        self.cancel_confirmed = True
        return True

    def close(self, now):
        if self.config.mode == "release_failure":
            return False
        self.closed = True
        return True
