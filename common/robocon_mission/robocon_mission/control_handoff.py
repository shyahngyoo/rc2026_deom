"""Confirmed arbiter ownership, authoritative heartbeat and expiring leases."""
import time
import uuid
from typing import Callable
from go2_obstacle_interfaces.msg import ControlState
from go2_obstacle_interfaces.srv import SetControlMode
from .async_service import AsyncServiceRunner


class ControlHandoff:
    OWNERS = {"NAVIGATION", "SKILL", "SAFE_HOLD"}

    def __init__(self, node, service_name, state_topic, state_timeout=0.5,
                 lease_duration=2.0):
        self.node = node
        self.mission_id = uuid.uuid4().hex
        self.runner = AsyncServiceRunner(node.create_client(SetControlMode, service_name))
        self.emergency = AsyncServiceRunner(node.create_client(SetControlMode, service_name))
        self.owner = "UNKNOWN"
        self.state = None
        self.received = 0.0
        self.state_timeout = state_timeout
        self.lease_duration = lease_duration
        self.next_renewal = 0.0
        self.confirmation = None
        self.subscription = node.create_subscription(ControlState, state_topic,
                                                      self._state, 10)
        self.failure = ""

    def _state(self, msg):
        self.state = msg
        self.received = time.monotonic()

    def confirmed(self, owner: str) -> bool:
        msg = self.state
        if msg is None:
            return False
        stamp = msg.stamp.sec * 1000000000 + msg.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        return (self.owner == owner and msg.owner == owner and msg.healthy and
                msg.mission_id == self.mission_id and stamp > 0 and
                -0.1 <= age <= self.state_timeout and
                time.monotonic() - self.received <= self.state_timeout)

    def switch(self, command, done: Callable, precondition: Callable) -> None:
        if self.confirmation is not None or self.runner.unknown_terminal:
            raise RuntimeError("previous ownership switch remains active/unknown")
        request = SetControlMode.Request(
            owner=command.mode, expected_owner=("SAFE_HOLD" if self.owner == "UNKNOWN"
                                               else self.owner),
            mission_id=self.mission_id, request_id=uuid.uuid4().hex,
            lease_duration=self.lease_duration)
        self.confirmation = dict(command=command, done=done, guard=precondition,
                                 request=request,
                                 deadline=time.monotonic() + command.timeout,
                                 sent=False, response=None)
        # Owner is not updated until both RPC acknowledgment and topic agree.

    def _ack(self, op, _token, outcome, terminal, result):
        if self.confirmation is not op:
            return
        if time.monotonic() >= op["deadline"]:
            outcome = "ownership_confirmation_timeout"
        if outcome != "succeeded":
            self.confirmation = None
            self.owner = "UNKNOWN"
            op["done"](op["command"].token, outcome, terminal, result)
            return
        self.owner = result.owner
        op["response"] = result
        self.next_renewal = time.monotonic() + self.lease_duration / 3.0

    @staticmethod
    def _valid(request, result):
        return (result is not None and result.accepted and
                result.owner == request.owner and
                result.mission_id == request.mission_id and
                result.request_id == request.request_id)

    def tick(self) -> None:
        self.runner.tick()
        self.emergency.tick()
        op = self.confirmation
        if op is not None:
            if time.monotonic() >= op["deadline"]:
                self.confirmation = None
                self.owner = "UNKNOWN"
                self.runner.cancel()
                op["done"](op["command"].token, "ownership_confirmation_timeout",
                           not op["sent"], None)
            elif (not op["sent"] and self.runner.pending is None and
                  not self.runner.unknown_terminal and op["guard"]()):
                op["sent"] = True
                remaining = op["deadline"] - time.monotonic()
                self.runner.start(op["command"].token, op["request"], remaining,
                                  lambda *args: self._ack(op, *args),
                                  lambda r: self._valid(op["request"], r), True)
            elif (op["response"] is not None and
                  self.confirmed(op["command"].mode) and op["guard"]()):
                self.confirmation = None
                op["done"](op["command"].token, "succeeded", True, op["response"])
        elif (self.owner in self.OWNERS and not self.failure and
              self.runner.pending is None and
              time.monotonic() >= self.next_renewal):
            request = SetControlMode.Request(
                owner=self.owner, expected_owner=self.owner,
                mission_id=self.mission_id, request_id=uuid.uuid4().hex,
                lease_duration=self.lease_duration)
            self.next_renewal = time.monotonic() + self.lease_duration / 3.0

            def renewed(_token, outcome, terminal, _result):
                if outcome != "succeeded" or not terminal:
                    self.failure = "control lease renewal unconfirmed"
                    self.owner = "UNKNOWN"

            self.runner.start(0, request, self.lease_duration / 3.0, renewed,
                              lambda r: self._valid(request, r), True)

    def safe_hold(self) -> None:
        """Best-effort independent safety RPC; local success is never assumed."""
        self.failure = "mission stopped"
        self.confirmation = None
        self.runner.cancel()
        self.owner = "UNKNOWN"
        if self.emergency.pending is not None or self.emergency.unknown_terminal:
            return
        request = SetControlMode.Request(
            owner="SAFE_HOLD", expected_owner="UNKNOWN",
            mission_id=self.mission_id, request_id=uuid.uuid4().hex,
            lease_duration=self.lease_duration)
        def held(_token, outcome, terminal, result):
            if outcome == "succeeded" and terminal:
                self.owner = result.owner
            else:
                self.node.get_logger().error("SAFE_HOLD acknowledgment unconfirmed")
        self.emergency.start(0, request, self.lease_duration / 2.0, held,
                             lambda r: self._valid(request, r), True)

