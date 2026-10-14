"""Regression for feedback accepted before the real send or after a fault."""
import json
from threading import RLock
from types import SimpleNamespace

from robocon_mission.hurricane_server import HurricaneServer, ObservedTransport
from robocon_mission.hurricane_protocol import DryRunTransport


def adapter():
    node = object.__new__(HurricaneServer)
    node.lock = RLock()
    node.goal = SimpleNamespace(request=SimpleNamespace(obstacle_id="bridge"))
    node.future = SimpleNamespace(done=lambda: False)
    node.dry_run = True
    node.health_client = None
    node.execution_started = True
    node.last_event_key = None
    node.last_event_stamp = 0
    node.runner = SimpleNamespace(session_id="session", index=2,
                                  status="awaiting_safe_stance")
    node.phase_key = ("session", 2)
    node.phase_started_ns = 900_000_000
    node.final_release = False
    node.feedback_key = None
    node.calls = []
    node.results = []
    node.runner.feedback = lambda *args: node.calls.append(args) or True
    node.runner.clear_confirmation = lambda: node.calls.append("cleared")
    node.runner.cancel = lambda now: node.calls.append("stop")
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))
    node.get_parameter = lambda name: SimpleNamespace(value=0.5)
    node.get_logger = lambda: SimpleNamespace(warn=lambda text: None)
    node._finish = lambda *args: node.results.append(args)
    return node


def event(**changes):
    value = dict(stamp_ns=950_000_000, session_id="session", obstacle_id="bridge",
                 step_index=2, completed=True, safe_stance=True,
                 control_released=True, event="completed")
    value.update(changes)
    return SimpleNamespace(data=json.dumps(value))


def test_feedback_before_actual_phase_send_is_ignored():
    node = adapter()
    node._feedback(event(stamp_ns=850_000_000))
    assert node.calls == [] and not node.final_release
    node.phase_key = ("session", 1)  # New index announced, but no new command sent.
    node._feedback(event())
    assert node.calls == []


def test_negative_confirmation_revokes_prior_success():
    node = adapter()
    node._feedback(event())
    assert node.final_release
    node._feedback(event(stamp_ns=960_000_000, completed=False, event="failed"))
    assert not node.final_release
    assert node.calls[-1][2:4] == (False, False)
    assert node.results[-1][0] is False


def test_release_revocation_and_out_of_order_success():
    node = adapter()
    node._feedback(event())
    assert node.final_release
    node._feedback(event(stamp_ns=970_000_000, control_released=False))
    assert not node.final_release and node.calls[-1] == "cleared"
    node._feedback(event(stamp_ns=960_000_000))
    assert not node.final_release


def test_negative_feedback_while_driving_sends_stop():
    node = adapter()
    node.runner.status = "running"
    node.runner.feedback = lambda *args: False
    node._feedback(event(event="failed", completed=False))
    assert "stop" in node.calls and node.results[-1][0] is False


def test_initialization_window_ignores_previous_runner():
    node = adapter()
    node.execution_started = False
    node._feedback(event())
    node._tick()
    assert node.calls == [] and node.results == []


def test_dry_run_cannot_prove_real_safe_stance():
    node = adapter()
    node.final_release = True
    result = node._result(True)
    assert result.completed and not result.safe_stance
    assert result.error_code == "DRY_RUN"


def test_unknown_or_unsafe_controller_cannot_finish():
    for invalid in ({"known": False}, {"safe_stance": False}, {"error_code": "FAULT"}):
        node = adapter()
        node._feedback(event(**invalid))
        assert not node.final_release
        assert node.results[-1][0] is False


def test_missing_release_and_foreign_sessions_are_ignored():
    for invalid in ({"control_released": False}, {"session_id": "old"}, {"step_index": 1}):
        node = adapter()
        node._feedback(event(**invalid))
        assert not node.final_release
        assert node.calls == (["cleared"] if invalid.get("control_released") is False else [])


def test_send_boundary_is_observed_before_writing():
    events = []
    class RecordingTransport(DryRunTransport):
        def send(self, command, repeat=1):
            events.append("write")
            return super().send(command, repeat)
    transport = ObservedTransport(RecordingTransport(), lambda: events.append("boundary"))
    transport.send("前进")
    assert events == ["boundary", "write"]
