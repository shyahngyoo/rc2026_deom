"""Measured stops must gate task completion, including the return point."""
from types import SimpleNamespace

import pytest

from robocon_mission.mission_fsm import MissionFSM, State
from robocon_mission.route_manager import RouteManager, Task
from robocon_mission.mission_manager import MissionManager
from robocon_mission import mission_manager


def started_fsm():
    route = RouteManager("start", "map", [
        Task("one", "navigation", pose=(0., 0., 0.)),
        Task("two", "navigation", pose=(1., 0., 0.)),
    ], (0., 0., 0.))
    fsm = MissionFSM(route, 5., 5., 5., 0, True, wait_at_waypoints=True)
    command = fsm.start()
    command = fsm.result(command.token, "succeeded")
    command = fsm.result(command.token, "succeeded", confirmed_owner="NAVIGATION")
    return fsm, command


def test_each_waypoint_and_return_wait_before_completion():
    fsm, command = started_fsm()
    for index in range(2):
        old = command
        assert fsm.result(command.token, "succeeded") is None
        assert fsm.state == State.WAIT_WAYPOINT_STOP
        assert len(fsm.route.completed) == index
        assert fsm.result(old.token, "succeeded") is None
        command = fsm.stopped()
        assert len(fsm.route.completed) == index + 1
        assert fsm.stopped() is None  # duplicate stop cannot advance next task
    assert fsm.state == State.RETURNING and command.task_id == "return"
    assert fsm.result(command.token, "succeeded") is None
    assert fsm.state == State.WAIT_WAYPOINT_STOP and fsm.waiting_return_stop
    assert fsm.stopped() is None
    assert fsm.state == State.FINISHED
    assert fsm.route.completed == ["one", "two"]


def test_waypoint_timeout_never_completes_or_starts_next_goal():
    fsm, command = started_fsm()
    fsm.result(command.token, "succeeded")
    fsm.stop_failed("did not stop")
    assert fsm.state == State.ABORTED and not fsm.route.completed
    assert fsm.stopped() is None


@pytest.mark.parametrize("ready,healthy,tf,terminal,deadline,expected", [
    (False, True, True, True, 20., "waiting"),
    (True, False, True, True, 20., "waiting"),  # old accepted pose
    (False, True, True, True, 9., "abort"),
    (True, False, True, True, 9., "abort"),
    (True, True, False, True, 20., "abort"),
    (True, True, True, False, 20., "abort"),
    (True, True, True, True, 20., "advance"),
])
def test_adapter_requires_stop_localization_and_terminal_actions(
        monkeypatch, ready, healthy, tf, terminal, deadline, expected):
    monkeypatch.setattr(mission_manager.time, "monotonic", lambda: 10.)
    events = []
    manager = SimpleNamespace(
        stop_deadline=deadline, stop_timeout=8.,
        localization=SimpleNamespace(health_ok=True, tf_fresh=lambda: tf,
                                     healthy=lambda: healthy),
        stop_monitor=SimpleNamespace(ready=lambda: ready),
        fsm=SimpleNamespace(state=State.WAIT_WAYPOINT_STOP, precise_waypoints=False,
                            stopped=lambda: "next"),
        _actions_terminal=lambda: terminal,
        _abort=lambda reason: events.append(("abort", reason)),
        _dispatch=lambda cmd: events.append(("advance", cmd)),
    )
    MissionManager._wait_for_stop(manager)
    assert (events[0][0] if events else "waiting") == expected

