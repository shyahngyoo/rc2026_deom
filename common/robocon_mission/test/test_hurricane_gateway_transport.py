"""Gateway integration regressions, with no serial port or hardware motion."""
import copy
import json
from threading import RLock
from types import SimpleNamespace

import pytest
from go2_obstacle_interfaces.msg import MotionAuthority
from rclpy.time import Time
from std_msgs.msg import String

from robocon_mission.hurricane_gateway_transport import GatewayTransport


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, value):
        self.messages.append(value)


class Node:
    def __init__(self):
        self.lock = RLock()
        self.now = 10_000_000_000
        self.publishers = {}

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=self.now))

    def get_logger(self):
        return SimpleNamespace(warn=lambda _: None)

    def create_publisher(self, _type, name, _depth):
        publisher = Publisher()
        self.publishers[name] = publisher
        return publisher

    def create_subscription(self, *_args, **_kwargs):
        return SimpleNamespace()


def fixture(capabilities=None):
    node = Node()
    context = dict(runner_session="runner", obstacle_id="wall", step_index=0,
                   mission_id="mission", active=True)
    feedback = []
    transport = GatewayTransport(node, lambda: context,
        lambda message: feedback.append(json.loads(message.data)), health_ready=lambda: True)
    authority = MotionAuthority(owner="SKILL", mission_id="mission", generation=2,
                                lease_remaining=2.0, healthy=True)
    authority.header.stamp = Time(nanoseconds=node.now).to_msg()
    transport._authority(authority)
    state = dict(known=True, mode="HOLD", stopped=True, safe_stance=True,
                 control_releasable=True, skill_active=False, obstacle_id="wall",
                 skill_state="idle", estop=False, human_enable=True, fault_code="",
                 capabilities=[] if capabilities is None else capabilities)
    envelope = dict(connected=True, handshake=True, session=100, sequence=1,
                    gateway_sequence=1, stamp_ns=node.now, status=state, error_code="")
    transport._status(String(data=json.dumps(envelope)))
    return node, context, transport, feedback, envelope


def status(node, transport, envelope, **changes):
    update = copy.deepcopy(envelope)
    update["gateway_sequence"] = transport.gateway_sequence + 1
    update["stamp_ns"] = node.now
    update["status"].update(changes)
    transport._status(String(data=json.dumps(update)))
    return update


def correlate(transport):
    return {key: transport.pending[key] for key in
            ("request_id", "runner_session", "step_index", "obstacle_id")}


@pytest.mark.parametrize("command,expected", [(1, (0., .25)), (2, (.08, 0.)),
                                                (3, (0., -.25)), (4, (-.08, 0.))])
def test_directions_only_use_typed_arbiter_channel(command, expected):
    node, _, transport, _, _ = fixture()
    transport.send(command, repeat=4)
    request = json.loads(transport.requests.messages[-1].data)
    assert request["op"] == "command" and request["command_id"] == command
    message = transport.commands.messages[-1]
    assert (message.owner, message.mission_id, message.generation) == ("SKILL", "mission", 2)
    assert message.header.frame_id == "base_footprint"
    assert (message.twist.linear.x, message.twist.angular.z) == expected
    assert message.twist.linear.y == message.twist.linear.z == 0.
    transport.tick()
    assert len(transport.requests.messages) == 1  # No repeated skill start.
    assert len(transport.commands.messages) == 2  # Fresh producer heartbeat.


def test_direction_command_is_accepted_by_actual_motion_arbiter():
    from go2_mission_backends.arbiter_core import Arbiter, Switch
    from go2_mission_backends.motion_arbiter import MotionArbiter
    node, _, transport, _, _ = fixture()
    arbiter = object.__new__(MotionArbiter)
    arbiter.core = Arbiter()
    assert arbiter.core.switch(Switch("SKILL", "SAFE_HOLD", "mission", "grant", 2.),
                               10., node.now, True).accepted
    arbiter.base_frame = "base_footprint"
    arbiter.fresh = lambda: True
    arbiter.active = lambda: True
    arbiter._now = lambda: node.now
    arbiter.get_logger = lambda: SimpleNamespace(debug=lambda _: None)
    # Arbiter uses monotonic seconds; its deadline must share that same clock.
    import time
    arbiter.core.deadline = time.monotonic() + 2.
    arbiter.core.last_command_wall = time.monotonic()
    node.now += 10_000_000
    transport.send(2)
    arbiter._command("SKILL", transport.commands.messages[-1])
    assert arbiter.core.reason == "COMMAND_ACCEPTED"
    assert arbiter.core.command == (.08, 0., 0., 0., 0., 0.)
    assert arbiter.core.owner == "SKILL"
    # An unauthorized navigation producer cannot overwrite the accepted skill.
    arbiter._command("NAVIGATION", transport.commands.messages[-1])
    assert arbiter.core.reason == "UNAUTHORIZED_SOURCE"
    assert arbiter.core.command == (.08, 0., 0., 0., 0., 0.)


@pytest.mark.parametrize("command", [6, 7, 8, 9, 18, 19, 20])
def test_unknown_capability_or_lateral_skill_is_explicitly_unsupported(command):
    _, _, transport, feedback, _ = fixture()
    with pytest.raises(RuntimeError, match="UNSUPPORTED"):
        transport.send(command)
    assert not transport.requests.messages and not feedback
    assert all(m.twist.linear.x == m.twist.angular.z == 0. for m in transport.commands.messages)


def test_ack_is_not_completion_and_only_matching_fresh_status_completes():
    node, _, transport, feedback, envelope = fixture([6])
    transport.send(6, repeat=4)
    assert len(transport.requests.messages) == 1
    correlation = correlate(transport)
    transport._event(String(data=json.dumps(dict(correlation, accepted=True, event="ack"))))
    assert feedback == []
    node.now += 10_000_000
    running = status(node, transport, envelope, **correlation, skill_state="running",
                     skill_active=True, control_releasable=False)
    assert feedback[-1]["event"] == "progress" and not feedback[-1]["completed"]
    node.now += 10_000_000
    done = status(node, transport, running, **correlation, skill_state="succeeded",
                  skill_active=False, stopped=True, safe_stance=True, control_releasable=True)
    assert feedback[-1]["completed"] and feedback[-1]["control_released"]
    assert feedback[-1]["stamp_ns"] == done["stamp_ns"]
    count = len(feedback)
    transport._status(String(data=json.dumps(done)))
    node.now += 10_000_000
    status(node, transport, done, **dict(correlation, runner_session="old"), skill_state="succeeded")
    assert len(feedback) == count  # Neither replay nor a foreign runner is evidence.


def test_unsupported_ack_fails_instead_of_succeeding():
    _, _, transport, feedback, _ = fixture([9])
    transport.send(9)
    transport._event(String(data=json.dumps(dict(correlate(transport), accepted=False,
                                                error_code="UNSUPPORTED"))))
    assert feedback[-1]["event"] == "failed"
    assert not feedback[-1]["completed"] and feedback[-1]["error_code"] == "UNSUPPORTED"


@pytest.mark.parametrize("terminal", ["failed", "canceled"])
def test_real_failure_and_cancellation_are_never_reported_success(terminal):
    node, _, transport, feedback, envelope = fixture([6])
    transport.send(6)
    node.now += 10_000_000
    status(node, transport, envelope, **correlate(transport), skill_state=terminal)
    assert feedback[-1]["event"] == "failed" and not feedback[-1]["completed"]


def test_hold_requires_measured_support_and_release_not_an_ack():
    node, _, transport, feedback, envelope = fixture()
    transport.send(5)
    correlation = correlate(transport)
    transport._event(String(data=json.dumps(dict(correlation, accepted=True))))
    assert feedback == []
    node.now += 10_000_000
    status(node, transport, envelope, **correlation, skill_state="succeeded",
           stopped=False, control_releasable=False)
    assert not feedback[-1]["completed"] and not feedback[-1]["control_released"]
    transport.cancel()
    assert json.loads(transport.requests.messages[-1].data)["op"] == "cancel"
    assert transport.direction == (0., 0.)


def test_stale_status_stops_heartbeat_and_does_not_replay_old_direction():
    node, _, transport, _, envelope = fixture()
    transport.send(2)
    node.now += 400_000_000
    transport.tick()
    assert transport.direction == (0., 0.)
    assert transport.commands.messages[-1].twist.linear.x == 0.
    status(node, transport, envelope)
    transport.tick()
    assert transport.commands.messages[-1].twist.linear.x == 0.


def test_reconnect_or_new_authority_fences_an_active_runner():
    node, _, transport, _, envelope = fixture()
    transport.send(2)
    update = copy.deepcopy(envelope)
    update.update(session=101, gateway_sequence=2)
    transport._status(String(data=json.dumps(update)))
    assert not transport.ready() and transport.direction == (0., 0.)
    with pytest.raises(RuntimeError, match="not ready"):
        transport.send(2)


def test_lease_expiry_replay_and_wrong_owner_never_authorize_motion(monkeypatch):
    node, _, transport, _, _ = fixture()
    received = transport.authority_received
    monkeypatch.setattr("robocon_mission.hurricane_gateway_transport.time.monotonic",
                        lambda: received + 3.)
    transport._authority(transport.authority)  # Exact replay cannot extend receipt.
    assert transport.authority_received == received and not transport.ready()
    with pytest.raises(RuntimeError, match="not ready"):
        transport.send(2)


def test_ready_service_capabilities_are_per_calibrated_profile():
    _, _, transport, _, _ = fixture([9, 6])
    profiles = {"ready": dict(calibrated=True, steps=[dict(type="mode", command="正常模式"),
                                                       dict(type="jump", command="小跳1")]),
                "uncalibrated": dict(calibrated=False, steps=[dict(type="jump", command="小跳1")]),
                "missing": dict(calibrated=True, steps=[dict(type="jump", command="小跳2")])}
    assert transport.available_skills(profiles) == ["ready"]


def test_real_server_has_no_direct_serial_constructor():
    import inspect
    from robocon_mission.hurricane_server import HurricaneServer
    source = inspect.getsource(HurricaneServer)
    assert "SerialTransport(" not in source
    assert "GatewayTransport(" in source
    assert '"/skills/ready"' in source


def test_unsupported_action_result_has_explicit_machine_readable_error():
    from robocon_mission.hurricane_server import HurricaneServer
    server = object.__new__(HurricaneServer)
    server.dry_run = False
    server.final_release = False
    result = server._result(False, "transport_error: UNSUPPORTED command_id=9")
    assert result.error_code == "UNSUPPORTED" and not result.completed


def test_runner_is_bound_to_generation_captured_at_action_acceptance():
    node, context, transport, _, _ = fixture()
    context["generation"] = 2
    transport.send(5)  # Even a stationary first waypoint binds the active lease.
    node.now += 10_000_000
    authority = MotionAuthority(owner="SKILL", mission_id="mission", generation=3,
                                lease_remaining=2., healthy=True)
    authority.header.stamp = Time(nanoseconds=node.now).to_msg()
    transport._authority(authority)
    assert not transport.ready()
    with pytest.raises(RuntimeError, match="not ready"):
        transport.send(2)


@pytest.mark.parametrize("changes", [dict(connected=False), dict(handshake=False),
                                     dict(error_code="SERIAL_DISCONNECTED"), dict(status=None)])
def test_invalid_new_snapshot_immediately_revokes_previous_ready_evidence(changes):
    node, _, transport, feedback, envelope = fixture([6])
    transport.send(6)
    node.now += 10_000_000
    update = dict(envelope, gateway_sequence=transport.gateway_sequence + 1,
                  stamp_ns=node.now, **changes)
    transport._status(String(data=json.dumps(update)))
    assert transport.snapshot is None and not transport.ready()
    assert transport.available_skills({"test": dict(calibrated=True, steps=[])}) == []
    assert transport.revoked and transport.direction == (0., 0.)
    assert feedback == []


def test_matching_request_from_wrong_obstacle_cannot_confirm_completion():
    node, _, transport, feedback, envelope = fixture([6])
    transport.send(6)
    node.now += 10_000_000
    wrong = dict(correlate(transport), obstacle_id="another_obstacle")
    status(node, transport, envelope, **wrong, skill_state="succeeded")
    assert feedback == []
    transport._event(String(data=json.dumps(dict(wrong, accepted=False, error_code="UNSUPPORTED"))))
    assert feedback == []
    node.now += 10_000_000
    status(node, transport, envelope, **correlate(transport), skill_state="succeeded")
    assert feedback[-1]["completed"] and feedback[-1]["obstacle_id"] == "wall"
