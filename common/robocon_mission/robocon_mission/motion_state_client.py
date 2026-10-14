"""Independently poll Motion Manager until safe stance is continuously confirmed."""
import time
from typing import Callable
from go2_obstacle_interfaces.srv import GetMotionState
from .async_service import AsyncServiceRunner


class MotionStateClient:
    def __init__(self, node, service_name, stopped, stable_duration=0.5):
        self.runner = AsyncServiceRunner(node.create_client(GetMotionState, service_name))
        self.stopped = stopped
        self.stable_duration = stable_duration
        self.pending = None
        self.last_result = None
        self.last_result_at = 0.0
        self.watch_task_id = ""
        self.next_watch = 0.0

    def safe_recent(self, task_id: str) -> bool:
        result = self.last_result
        return (result is not None and time.monotonic() - self.last_result_at <= 0.3
                and result.known and result.obstacle_id == task_id
                and not result.skill_active and result.safe_stance
                and result.control_releasable and not result.error_code
                and self.stopped())

    def start(self, command, done: Callable) -> None:
        if self.pending is not None:
            raise RuntimeError("motion check already active")
        self.pending = dict(command=command, done=done,
                            deadline=time.monotonic() + command.timeout,
                            stable_since=None, last_confirmed=0.0,
                            next_poll=0.0, reason="motion status unavailable")
        self.last_result = None
        self.watch_task_id = command.task_id

    def _finish(self, op, outcome):
        if self.pending is not op:
            return
        self.pending = None
        self.runner.cancel()
        op["done"](op["command"].token, outcome, True, self.last_result)

    def tick(self) -> None:
        self.runner.tick()
        op = self.pending
        if op is None:
            if (self.watch_task_id and self.runner.pending is None and
                    time.monotonic() >= self.next_watch):
                task_id = self.watch_task_id
                self.next_watch = time.monotonic() + 0.1
                def watched(_token, outcome, _terminal, result):
                    if self.watch_task_id == task_id:
                        self.last_result = result if outcome == "succeeded" else None
                        self.last_result_at = time.monotonic()
                self.runner.start(0, GetMotionState.Request(obstacle_id=task_id),
                                  0.3, watched, lambda r: r is not None)
            return
        now = time.monotonic()
        if not self.stopped() or now - op["last_confirmed"] > 0.3:
            op["stable_since"] = None
        if now >= op["deadline"]:
            self._finish(op, f"timeout:{op['reason']}")
        elif self.runner.pending is None and now >= op["next_poll"]:
            cmd = op["command"]
            op["next_poll"] = now + 0.1
            def observed(_token, outcome, _terminal, result):
                if self.pending is not op:
                    return
                self.last_result = result
                self.last_result_at = time.monotonic()
                valid = (outcome == "succeeded" and result.known and
                         result.obstacle_id == cmd.task_id and not result.skill_active and
                         result.safe_stance and result.control_releasable and
                         not result.error_code and self.stopped())
                if not valid:
                    op["stable_since"] = None
                    op["reason"] = (getattr(result, "error_code", "") or
                                    "unsafe stance, active skill or unknown motion state")
                    return
                observed_at = time.monotonic()
                op["last_confirmed"] = observed_at
                if op["stable_since"] is None:
                    op["stable_since"] = observed_at
                if observed_at - op["stable_since"] >= self.stable_duration:
                    self._finish(op, "succeeded")
            self.runner.start(cmd.token, GetMotionState.Request(obstacle_id=cmd.task_id),
                              min(0.3, op["deadline"] - now), observed,
                              lambda r: r is not None)

    def cancel(self) -> None:
        self.watch_task_id = ""
        if self.pending is not None:
            self._finish(self.pending, "canceled")

