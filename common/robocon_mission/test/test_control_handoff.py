"""Arbiter acknowledgment is insufficient without matching fresh ownership state."""
from types import SimpleNamespace
from go2_obstacle_interfaces.msg import ControlState
from go2_obstacle_interfaces.srv import SetControlMode
from robocon_mission.control_handoff import ControlHandoff
from robocon_mission.mission_fsm import Command
from test_async_service import Client


class Node:
    def __init__(self, clock):
        self.clock = clock
        self.clients = []
    def create_client(self, *args):
        client = Client()
        self.clients.append(client)
        return client
    def create_subscription(self, *args):
        return object()
    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(self.clock[0] * 1e9)))
    def get_logger(self):
        return SimpleNamespace(error=lambda message: None)


def make_handoff(monkeypatch):
    clock, outcomes = [10.0], []
    monkeypatch.setattr("robocon_mission.control_handoff.time.monotonic", lambda: clock[0])
    node = Node(clock)
    obj = ControlHandoff(node, "/mock", "/state")
    obj.switch(Command("handoff", mode="NAVIGATION", token=4, timeout=1.0),
               lambda *args: outcomes.append(args), lambda: True)
    obj.tick()
    obj.tick()
    return obj, node, clock, outcomes


def acknowledge(obj, node, request_id=None):
    req = obj.runner.pending["request"]
    node.clients[0].future.resolve(SetControlMode.Response(
        accepted=True, owner=req.owner, mission_id=req.mission_id,
        request_id=req.request_id if request_id is None else request_id))
    return req


def test_rpc_success_requires_authoritative_state_and_session(monkeypatch):
    obj, node, clock, outcomes = make_handoff(monkeypatch)
    acknowledge(obj, node)
    obj.tick()
    assert not outcomes and not obj.confirmed("NAVIGATION")
    msg = ControlState(owner="NAVIGATION", mission_id="other", healthy=True)
    msg.stamp.sec = 10
    obj._state(msg)
    obj.tick()
    assert not outcomes
    msg.mission_id = obj.mission_id
    obj._state(msg)
    obj.tick()
    assert outcomes[0][:3] == (4, "succeeded", True)
    clock[0] += .6
    assert not obj.confirmed("NAVIGATION")


def test_mismatched_rpc_request_id_never_confirms_handoff(monkeypatch):
    obj, node, _, outcomes = make_handoff(monkeypatch)
    acknowledge(obj, node, "obsolete_request")
    assert outcomes[0][1] == "service_rejected"
    assert obj.owner == "UNKNOWN"


def test_sent_lease_renewal_timeout_loses_owner(monkeypatch):
    obj, node, clock, outcomes = make_handoff(monkeypatch)
    acknowledge(obj, node)
    msg = ControlState(owner="NAVIGATION", mission_id=obj.mission_id, healthy=True)
    msg.stamp.sec = 10
    obj._state(msg)
    obj.tick()
    assert outcomes
    clock[0] += .7
    obj.tick()  # create lease renewal
    obj.tick()  # send it, no acknowledgment
    clock[0] += .7
    obj.tick()
    assert obj.failure and obj.owner == "UNKNOWN" and obj.runner.unknown_terminal

