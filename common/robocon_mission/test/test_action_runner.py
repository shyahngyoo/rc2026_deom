"""Exercise transport races without launching any Action server."""
from types import SimpleNamespace
import pytest
from action_msgs.msg import GoalStatus
from robocon_mission import navigation_client as module
from test_async_service import Future


class Handle:
    accepted = True
    def __init__(self):
        self.result_future = Future()
        self.cancel_calls = 0
    def get_result_async(self):
        return self.result_future
    def cancel_goal_async(self):
        self.cancel_calls += 1
        return Future()


class Client:
    def __init__(self, *args):
        self.accept_future = Future()
        self.ready = True
    def server_is_ready(self):
        return self.ready
    def send_goal_async(self, *args, **kwargs):
        return self.accept_future


def runner(monkeypatch):
    clock, outcomes = [0.0], []
    monkeypatch.setattr(module, "ActionClient", Client)
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    obj = module.AsyncActionRunner(None, None, "/fake", 2, .5)
    obj.start(3, object(), 5, lambda *args: outcomes.append(args))
    obj.tick()
    return obj, clock, outcomes


def test_late_acceptance_canceled_but_never_advances(monkeypatch):
    obj, clock, outcomes = runner(monkeypatch)
    obj.request_cancel("shutdown")
    clock[0] = 1
    obj.tick()
    handle = Handle()
    obj.client.accept_future.resolve(handle)
    assert handle.cancel_calls == 1
    assert obj.unknown_terminal and len(outcomes) == 1 and not outcomes[0][2]


def test_cancel_ack_is_not_a_terminal_action_result(monkeypatch):
    obj, clock, outcomes = runner(monkeypatch)
    handle = Handle()
    obj.client.accept_future.resolve(handle)
    obj.request_cancel("timeout")
    assert not outcomes
    clock[0] = 1
    obj.tick()
    assert obj.unknown_terminal and outcomes[0][1] == "cancel_unconfirmed"


def test_success_after_cancel_does_not_complete_task(monkeypatch):
    obj, _, outcomes = runner(monkeypatch)
    handle = Handle()
    obj.client.accept_future.resolve(handle)
    obj.request_cancel("timeout")
    handle.result_future.resolve(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED, result=object()))
    assert outcomes[0][1:3] == ("timeout", True)


def test_nonterminal_result_status_blocks_future_goals(monkeypatch):
    obj, _, outcomes = runner(monkeypatch)
    handle = Handle()
    obj.client.accept_future.resolve(handle)
    handle.result_future.resolve(SimpleNamespace(status=GoalStatus.STATUS_EXECUTING,
                                                result=None))
    assert not outcomes[0][2] and obj.unknown_terminal
    with pytest.raises(RuntimeError):
        obj.start(4, object(), 5, lambda *args: None)

