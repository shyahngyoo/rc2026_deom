"""Nonblocking NavigateToPose client with cancellation confirmation."""

import math
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def installed_trees(transit_override="", approach_override=""):
    share = Path(get_package_share_directory("go2_nav_bringup"))
    paths = {
        "transit": Path(transit_override) if transit_override else
                   share / "behavior_trees" / "nav_transit.xml",
        "approach": Path(approach_override) if approach_override else
                    share / "behavior_trees" / "nav_approach.xml",
    }
    for mode, path in paths.items():
        if not path.is_file():
            raise FileNotFoundError(f"{mode} BT XML missing: {path}")
    return {mode: str(path.resolve()) for mode, path in paths.items()}

#把yaml转化成Nav2 认识的
def pose_goal(node, pose, frame, tree):
    x, y, yaw = pose
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    goal.behavior_tree = tree
    return goal


class AsyncActionRunner:
    """One goal at a time; unknown terminal status blocks all later goals."""

    def __init__(self, node, action_type, action_name, server_timeout,
                 cancel_timeout):
        self.client = ActionClient(node, action_type, action_name)
        self.server_timeout = server_timeout
        self.cancel_timeout = cancel_timeout
        self.active = None
        self.unknown_terminal = False
        self.last_feedback = ""

    def start(self, token, goal, timeout, done):
        if self.active is not None or self.unknown_terminal:
            raise RuntimeError("previous Action has not been confirmed terminal")
        now = time.monotonic()
        self.active = {
            "token": token, "goal": goal, "done": done,
            "stage": "server", "server_deadline": now + self.server_timeout,
            "deadline": now + timeout,
            "handle": None, "result_future": None, "cancel_reason": "",
            "cancel_deadline": None,
        }

    def _finish(self, op, outcome, terminal, result=None):
        if self.active is not op:
            return
        self.active = None
        if not terminal:
            self.unknown_terminal = True
        op["done"](op["token"], outcome, terminal, result)

    def _feedback(self, op, message):
        if self.active is op and op["stage"] == "running":
            feedback = message.feedback
            self.last_feedback = (str(getattr(feedback, "phase", "")) or
                                  str(getattr(feedback, "distance_remaining", "")))
            if hasattr(feedback, "progress"):
                self.last_feedback += f" progress={feedback.progress:.3f}"

    def _accepted(self, op, future):
        if self.active is not op:
            try:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                pass
            return
        try:
            handle = future.result()
        except Exception:
            self._finish(op, "acceptance_unknown", False)
            return
        if not handle.accepted:
            self._finish(op, "rejected", True)
            return
        op["handle"] = handle
        op["stage"] = "running"
        try:
            result_future = handle.get_result_async()
        except Exception:
            self._finish(op, "result_request_unknown", False)
            return
        op["result_future"] = result_future
        result_future.add_done_callback(lambda result: self._result(op, result))
        if op["cancel_reason"]:
            self._cancel(op)

    def _result(self, op, future):
        if self.active is not op:
            return
        try:
            response = future.result()
        except Exception:
            self._finish(op, "result_unknown", False)
            return
        status = response.status
        outcome = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled",
        }.get(status, f"status_{status}")
        if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                          GoalStatus.STATUS_CANCELED):
            self._finish(op, outcome, False, response.result)
            return
        if time.monotonic() >= op["deadline"] and not op["cancel_reason"]:
            op["cancel_reason"] = "timeout"
        # A cancel request must never be converted into task success.
        if op["cancel_reason"] and outcome == "succeeded":
            outcome = op["cancel_reason"]
        self._finish(op, outcome, True, response.result)

    def _cancel(self, op):
        if self.active is not op or op["stage"] == "canceling":
            return
        if op["handle"] is None:
            return
        op["stage"] = "canceling"
        op["cancel_deadline"] = time.monotonic() + self.cancel_timeout
        try:
            op["handle"].cancel_goal_async()
        except Exception:
            # Still wait for a terminal result; otherwise status is unknown.
            pass

    def request_cancel(self, reason="canceled"):
        op = self.active
        if op is None:
            return
        if op["stage"] == "server":
            self._finish(op, reason, True)
            return
        op["cancel_reason"] = reason
        if op["stage"] == "accepting":
            op["cancel_deadline"] = time.monotonic() + self.cancel_timeout
        self._cancel(op)

    def tick(self):
        op = self.active
        if op is None:
            return
        now = time.monotonic()
        if op["stage"] == "server":
            if now >= min(op["server_deadline"], op["deadline"]):
                self._finish(op, "server_unavailable", True)
            elif self.client.server_is_ready():
                op["stage"] = "accepting"
                try:
                    future = self.client.send_goal_async(
                        op["goal"], feedback_callback=lambda msg: self._feedback(op, msg)
                    )
                except Exception:
                    self._finish(op, "send_goal_unknown", False)
                    return
                future.add_done_callback(lambda accepted: self._accepted(op, accepted))
        elif op["stage"] == "accepting" and now >= min(
                op["deadline"], op["cancel_deadline"] or op["deadline"]):
            # A late accepted goal could still be running; fail closed.
            self._finish(op, "acceptance_timeout", False)
        elif op["stage"] == "running" and now >= op["deadline"]:
            op["cancel_reason"] = "timeout"
            self._cancel(op)
        elif op["stage"] == "canceling" and now >= op["cancel_deadline"]:
            self._finish(op, "cancel_unconfirmed", False)


class NavigationClient(AsyncActionRunner):
    def __init__(self, node, action_name, trees, server_timeout, cancel_timeout):
        super().__init__(node, NavigateToPose, action_name, server_timeout,
                         cancel_timeout)
        self.node = node
        self.trees = trees

    def navigate(self, command, frame, done):
        goal = pose_goal(self.node, command.pose, frame,
                         self.trees[command.mode])
        self.start(command.token, goal, command.timeout, done)
