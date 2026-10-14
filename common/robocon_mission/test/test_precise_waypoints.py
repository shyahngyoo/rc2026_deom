"""Ordinary goals and return must align and pass a measured pose gate."""
from types import SimpleNamespace
import pytest
from robocon_mission.mission_fsm import MissionFSM, State
from robocon_mission.route_manager import RouteManager, Task
from robocon_mission.mission_manager import MissionManager
from robocon_mission import mission_manager


def initialized(retries=1, navigation_retries=1):
    route = RouteManager("start", "map", [
        Task("one", "navigation", pose=(.2, .3, .4)),
        Task("two", "navigation", pose=(1., 2., 3.)),
    ], (0., 0., 0.))
    fsm = MissionFSM(route, 5., 3., 5., navigation_retries, True,
                     wait_at_waypoints=True, precise_waypoints=True,
                     max_waypoint_alignment_retries=retries)
    cmd = fsm.start()
    cmd = fsm.result(cmd.token, "succeeded")
    return fsm, fsm.result(cmd.token, "succeeded", confirmed_owner="NAVIGATION")


def at_first_stop(retries=1):
    fsm, cmd = initialized(retries)
    cmd = fsm.result(cmd.token, "succeeded")
    fsm.result(cmd.token, "succeeded")
    return fsm


def test_each_original_goal_and_return_has_an_approach_before_stop():
    fsm, transit = initialized()
    for task in fsm.route.tasks:
        assert transit.mode == "transit" and fsm.stopped() is None
        approach = fsm.result(transit.token, "succeeded")
        assert approach.mode == "approach" and approach.pose == task.pose
        assert approach.timeout == 3. and fsm.state == State.APPROACHING_WAYPOINT
        assert fsm.stopped() is None and task.id not in fsm.route.completed
        assert fsm.result(transit.token, "succeeded") is None  # stale callback
        assert fsm.result(approach.token, "succeeded") is None
        assert fsm.state == State.WAIT_WAYPOINT_STOP
        assert fsm.waypoint_target() == task.pose
        transit = fsm.stopped()
    assert fsm.state == State.RETURNING
    approach = fsm.result(transit.token, "succeeded")
    assert fsm.state == State.APPROACHING_RETURN and approach.pose == fsm.route.return_pose
    assert fsm.stopped() is None
    fsm.result(approach.token, "succeeded")
    assert fsm.waiting_return_stop and fsm.waypoint_target() == fsm.route.return_pose
    fsm.stopped()
    assert fsm.state == State.FINISHED


def test_pose_mismatch_retry_is_bounded_and_does_not_complete_a_point():
    fsm = at_first_stop()
    retry = fsm.retry_waypoint_alignment("xy error")
    assert retry.mode == "approach" and retry.pose == fsm.route.current.pose
    assert fsm.waypoint_alignment_attempts == 1 and not fsm.route.completed
    fsm.result(retry.token, "succeeded")
    assert fsm.retry_waypoint_alignment("yaw error") is None
    assert fsm.state == State.ABORTED and not fsm.route.completed
    assert "retries exhausted" in fsm.error and fsm.stopped() is None


def test_successful_correction_resets_retry_budget_at_next_goal():
    fsm = at_first_stop()
    retry = fsm.retry_waypoint_alignment("xy error")
    fsm.result(retry.token, "succeeded")
    transit = fsm.stopped()
    assert fsm.route.completed == ["one"] and fsm.waypoint_alignment_attempts == 0
    assert transit.task_id == "two"


def test_return_pose_mismatch_retries_return_approach():
    fsm, cmd = initialized()
    for _ in range(2):
        cmd = fsm.result(cmd.token, "succeeded")
        fsm.result(cmd.token, "succeeded")
        cmd = fsm.stopped()
    cmd = fsm.result(cmd.token, "succeeded")
    fsm.result(cmd.token, "succeeded")
    retry = fsm.retry_waypoint_alignment("return drift")
    assert fsm.state == State.APPROACHING_RETURN and retry.task_id == "return"
    assert retry.pose == fsm.route.return_pose and fsm.state != State.FINISHED
    fsm.result(retry.token, "succeeded")
    fsm.retry_waypoint_alignment("return drift")
    assert fsm.state == State.ABORTED


def test_failed_waypoint_approach_retries_same_phase_after_health():
    fsm, cmd = initialized()
    approach = fsm.result(cmd.token, "succeeded")
    health = fsm.result(approach.token, "aborted")
    assert health.kind == "localization"
    retry = fsm.result(health.token, "succeeded")
    assert retry.mode == "approach" and fsm.state == State.APPROACHING_WAYPOINT
    assert retry.pose == fsm.route.current.pose
    fsm.result(retry.token, "aborted")
    assert fsm.state == State.ABORTED and not fsm.route.completed


def test_unknown_approach_cancellation_never_retries_or_completes():
    fsm, cmd = initialized()
    approach = fsm.result(cmd.token, "succeeded")
    assert fsm.result(approach.token, "cancel_unconfirmed", False) is None
    assert fsm.state == State.ABORTED and not fsm.route.completed


@pytest.mark.parametrize("retries", [-1, True, 1.5])
def test_invalid_alignment_retry_limits(retries):
    with pytest.raises(ValueError):
        initialized(retries)


def test_precision_requires_measured_stops():
    fsm, _ = initialized()
    with pytest.raises(ValueError, match="requires wait_at_waypoints"):
        MissionFSM(fsm.route, 5., 3., 5., 0, True, precise_waypoints=True)


@pytest.mark.parametrize("pose,expected", [
    ((.04, .02, .03), "advance"),
    ((.10, 0., .10), "advance"),
    ((.101, 0., 0.), "realign"),
    ((0., 0., .101), "realign"),
    ((float("nan"), 0., 0.), "abort"),
    (None, "abort"),
])
def test_stopped_pose_gate_checks_xy_yaw_and_finite_data(monkeypatch, pose, expected):
    monkeypatch.setattr(mission_manager.time, "monotonic", lambda: 10.)
    events = []
    sample = None if pose is None else SimpleNamespace(pose=pose)
    manager = SimpleNamespace(
        stop_deadline=20., stop_timeout=8., waypoint_xy_tolerance=.10,
        waypoint_yaw_tolerance=.10, waypoint_errors=None,
        route=SimpleNamespace(current=SimpleNamespace(id="one")),
        localization=SimpleNamespace(health_ok=True, tf_fresh=lambda: True,
                                     healthy=lambda: True, sample=lambda: sample),
        stop_monitor=SimpleNamespace(ready=lambda: True),
        fsm=SimpleNamespace(state=State.WAIT_WAYPOINT_STOP, precise_waypoints=True,
                            waypoint_target=lambda: (0., 0., 0.), waypoint_alignment_attempts=0,
                            stopped=lambda: "advance",
                            retry_waypoint_alignment=lambda _: "realign"),
        _actions_terminal=lambda: True, _publish=lambda: None,
        _abort=lambda reason: events.append(("abort", reason)),
        _dispatch=lambda command: events.append((command, None)),
        get_logger=lambda: SimpleNamespace(info=lambda _: None, warning=lambda _: None),
    )
    MissionManager._wait_for_stop(manager)
    assert events[0][0] == expected
    if expected == "abort":
        assert manager.waypoint_errors is None  # Never serialize NaN into status JSON.
