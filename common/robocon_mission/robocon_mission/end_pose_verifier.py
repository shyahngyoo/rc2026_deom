"""ROS-free, nonblocking pose verifier driven by fresh trusted samples."""
import math
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple


@dataclass(frozen=True)
class PoseSample:
    pose: Tuple[float, float, float]
    stamp_ns: int


def pose_errors(target, current) -> Tuple[float, float]:
    distance = math.hypot(target[0] - current[0], target[1] - current[1])
    delta = target[2] - current[2]
    return distance, abs(math.atan2(math.sin(delta), math.cos(delta)))


class EndPoseVerifier:
    def __init__(self, sample: Callable, healthy: Callable, stopped: Callable,
                  stable_duration=0.5, clock: Callable=time.monotonic,
                  max_sample_gap=0.4, minimum_samples=3):
        if not all(math.isfinite(v) and v > 0 for v in
                   (stable_duration, max_sample_gap)):
            raise ValueError("stable_duration and max_sample_gap must be positive")
        if isinstance(minimum_samples, bool) or not isinstance(minimum_samples, int) or minimum_samples < 2:
            raise ValueError("minimum_samples must be an integer >= 2")
        self.sample, self.healthy, self.stopped = sample, healthy, stopped
        self.stable_duration, self.clock = stable_duration, clock
        self.max_sample_gap, self.minimum_samples = max_sample_gap, minimum_samples
        self.pending: Optional[dict] = None
        self.last_reason = ""
        self.last_errors = None

    def start(self, command, done) -> None:
        if self.pending is not None:
            raise RuntimeError("pose verification already active")
        self.pending = dict(command=command, done=done,
                            deadline=self.clock() + command.timeout,
                             stable_since=None, stamp=None, last_new=None, samples=0)
        self.last_reason, self.last_errors = "no trusted pose", None

    def _finish(self, op, outcome):
        if self.pending is not op:
            return
        self.pending = None
        op["done"](op["command"].token, outcome, True, self.last_errors)

    def tick(self) -> None:
        op = self.pending
        if op is None:
            return
        now, cmd = self.clock(), op["command"]
        if now >= op["deadline"]:
            self._finish(op, f"timeout:{self.last_reason}")
            return
        try:
            sample = self.sample()
            if not self.healthy():
                reason = "localization untrusted"
            elif not self.stopped():
                reason = "robot not stopped"
            elif sample is None:
                reason = "TF missing, invalid or stale"
            else:
                distance, yaw = pose_errors(cmd.pose, sample.pose)
                self.last_errors = {"distance": distance, "yaw_error": yaw}
                if not all(math.isfinite(v) for v in (*sample.pose, distance, yaw)):
                    reason = "nonfinite TF"
                elif distance > cmd.xy_tolerance:
                    reason = f"xy error {distance:.4f} > {cmd.xy_tolerance}"
                elif yaw > cmd.yaw_tolerance:
                    reason = f"yaw error {yaw:.4f} > {cmd.yaw_tolerance}"
                elif op["stamp"] is not None and sample.stamp_ns < op["stamp"]:
                    reason = "TF time regressed"
                elif sample.stamp_ns == op["stamp"]:
                    # Repeated cached TF cannot satisfy a stable observation window.
                    if now - op["last_new"] > self.max_sample_gap:
                        op["stable_since"] = None
                        op["samples"] = 0
                        self.last_reason = "TF timestamp not advancing"
                    return
                else:
                    reason = ""
                    if op["last_new"] is not None and now - op["last_new"] > self.max_sample_gap:
                        op["stable_since"], op["samples"] = None, 0
                    op["stamp"], op["last_new"] = sample.stamp_ns, now
        except Exception as error:
            reason = f"sample exception:{error}"
        self.last_reason = reason or "observing stability"
        if reason:
            op["stable_since"] = None
            op["samples"] = 0
        else:
            op["samples"] += 1
            if op["stable_since"] is None:
                op["stable_since"] = now
            if (now - op["stable_since"] >= self.stable_duration and
                    op["samples"] >= self.minimum_samples):
                self._finish(op, "succeeded")

    def cancel(self) -> None:
        if self.pending is not None:
            self._finish(self.pending, "canceled")

