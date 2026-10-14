from robocon_mission.mission_fsm import MissionFSM, State
from robocon_mission.route_manager import RouteManager, Task


def make_fsm(simulation=True, retries=1, obstacles=1):
    route = RouteManager("start", "map", [
        Task("one", "navigation", pose=(0.2, 0.0, 0.0)),
        *[Task(f"obstacle{i}", "obstacle", transit_pose=(0.3, 0.0, 0.0),
               approach_pose=(0.4, 0.0, 0.0), end_pose=(1.0, 0.0, 0.0),
               skill_id="mock", transit_timeout=7.0, approach_timeout=3.0,
               skill_timeout=9.0) for i in range(obstacles)],
        Task("two", "navigation", pose=(0.1, 0.0, 0.0)),
    ], (0.0, 0.0, 0.0))
    return MissionFSM(route, 5.0, 5.0, 5.0, retries, simulation)


def reply(fsm, command, outcome="succeeded", terminal=True, **kwargs):
    return fsm.result(command.token, outcome, terminal, **kwargs)


def initialized(fsm):
    cmd = reply(fsm, fsm.start())
    assert cmd.kind == "handoff" and cmd.mode == "NAVIGATION"
    return reply(fsm, cmd, confirmed_owner="NAVIGATION")


def at_stop(fsm):
    cmd = initialized(fsm)
    cmd = reply(fsm, cmd)
    assert cmd.timeout == 7.0
    cmd = reply(fsm, cmd)
    assert cmd.timeout == 3.0 and cmd.mode == "approach"
    assert reply(fsm, cmd) is None
    assert fsm.state == State.WAIT_STOP


def skill_command(fsm):
    cmd = fsm.stopped()
    assert fsm.state == State.ACQUIRING_CONTROL and cmd.mode == "SKILL"
    cmd = reply(fsm, cmd, confirmed_owner="SKILL")
    assert cmd.kind == "skill" and cmd.timeout == 9.0
    return cmd


def complete_obstacle(fsm):
    cmd = skill_command(fsm)
    cmd = reply(fsm, cmd, completed=True, control_released=True, safe_stance=True)
    assert cmd.kind == "skill_safe" and fsm.state == State.WAIT_SKILL_SAFE
    cmd = reply(fsm, cmd)
    assert cmd.mode == "SAFE_HOLD"
    cmd = reply(fsm, cmd, confirmed_owner="SAFE_HOLD")
    assert cmd.kind == "localization"
    cmd = reply(fsm, cmd)
    assert cmd.kind == "verify_end" and cmd.pose == (1.0, 0.0, 0.0)
    assert fsm.route.current.id not in fsm.route.completed
    cmd = reply(fsm, cmd)
    assert cmd.mode == "NAVIGATION"
    return reply(fsm, cmd, confirmed_owner="NAVIGATION")


def test_complete_route_with_obstacle_and_return():
    fsm = make_fsm()
    at_stop(fsm)
    cmd = complete_obstacle(fsm)
    assert cmd.task_id == "two"
    cmd = reply(fsm, cmd)
    assert cmd.task_id == "return"
    assert reply(fsm, cmd) is None
    assert fsm.state == State.FINISHED
    assert fsm.route.completed == ["one", "obstacle0", "two"]


def test_bounded_navigation_retry_and_stale_callback():
    fsm = make_fsm(retries=1)
    first = initialized(fsm)
    check = reply(fsm, first, "aborted")
    assert check.kind == "localization" and fsm.state == State.RECOVERY
    second = reply(fsm, check)
    assert second.token != first.token
    assert reply(fsm, first) is None and fsm.route.index == 0
    assert reply(fsm, second, "aborted") is None
    assert fsm.state == State.ABORTED and not fsm.route.completed


def test_unknown_cancel_never_retries():
    fsm = make_fsm()
    cmd = initialized(fsm)
    assert reply(fsm, cmd, "cancel_unconfirmed", False) is None
    assert fsm.state == State.ABORTED and fsm.pending is None
    assert reply(fsm, cmd) is None


def test_skill_failure_and_control_hold_block_next_navigation():
    for outcome, completed, released, stance in [
        ("aborted", False, True, True), ("succeeded", True, False, True),
        ("result_unknown", False, False, False), ("canceled", False, True, True),
        ("succeeded", True, True, False),
    ]:
        fsm = make_fsm()
        at_stop(fsm)
        cmd = skill_command(fsm)
        assert reply(fsm, cmd, outcome, completed=completed,
                     control_released=released, safe_stance=stance) is None
        assert fsm.state == State.ABORTED and fsm.route.completed == ["one"]


def test_localization_failure_and_real_skill_gate():
    # Real mode now uses exactly the same confirmed arbiter path as simulation.
    fsm = make_fsm(simulation=False)
    at_stop(fsm)
    cmd = fsm.stopped()
    assert cmd.kind == "handoff"  # no fake success and no permanent placeholder gate
    reply(fsm, cmd, "service_rejected")
    assert fsm.state == State.ABORTED
    fsm = make_fsm()
    cmd = reply(fsm, fsm.start(), "health_failed")
    assert cmd.mode == "SAFE_HOLD"
    cmd = reply(fsm, cmd, confirmed_owner="SAFE_HOLD")
    assert cmd.kind == "relocalize"
    reply(fsm, cmd, "server_unavailable")
    assert fsm.state == State.ABORTED


def test_stop_timeout_and_return_failure():
    fsm = make_fsm(retries=0)
    at_stop(fsm)
    fsm.stop_failed("stale odom")
    assert fsm.state == State.ABORTED and fsm.stopped() is None
    fsm = make_fsm()
    at_stop(fsm)
    cmd = complete_obstacle(fsm)
    cmd = reply(fsm, cmd)
    assert cmd.task_id == "return"
    reply(fsm, cmd, "aborted")
    assert fsm.state == State.ABORTED


def test_approach_failure_never_starts_skill():
    for outcome, terminal in (("aborted", True), ("cancel_unconfirmed", False)):
        fsm = make_fsm(retries=0)
        cmd = reply(fsm, initialized(fsm))
        cmd = reply(fsm, cmd)
        assert cmd.mode == "approach"
        assert reply(fsm, cmd, outcome, terminal) is None
        assert fsm.state == State.ABORTED and fsm.stopped() is None
        assert fsm.route.completed == ["one"]


def test_not_stopped_no_handoff_or_skill():
    fsm = make_fsm()
    at_stop(fsm)
    assert fsm.pending is None and fsm.route.index == 1
    fsm.stop_failed("moving")
    assert fsm.stopped() is None


def test_safe_stance_release_verification_and_restore_fail_closed():
    for failure_kind in ("skill_safe", "SAFE_HOLD", "verify_end", "NAVIGATION"):
        fsm = make_fsm()
        at_stop(fsm)
        cmd = skill_command(fsm)
        cmd = reply(fsm, cmd, completed=True, control_released=True, safe_stance=True)
        while True:
            label = cmd.mode if cmd.kind == "handoff" else cmd.kind
            if label == failure_kind:
                break
            cmd = reply(fsm, cmd, confirmed_owner=cmd.mode)
        reply(fsm, cmd, "failed")
        assert fsm.state == State.ABORTED and fsm.route.completed == ["one"]
        assert fsm.pending is None


def test_relocalization_rechecks_health_and_never_repeats_skill():
    fsm = make_fsm()
    at_stop(fsm)
    cmd = skill_command(fsm)
    cmd = reply(fsm, cmd, completed=True, control_released=True, safe_stance=True)
    cmd = reply(fsm, cmd)
    cmd = reply(fsm, cmd, confirmed_owner="SAFE_HOLD")
    cmd = reply(fsm, cmd, "health_failed")
    assert cmd.kind == "relocalize"
    old = cmd
    cmd = reply(fsm, cmd)
    assert cmd.kind == "localization"
    cmd = reply(fsm, cmd)
    assert cmd.kind == "verify_end"
    assert reply(fsm, old) is None
    cmd = reply(fsm, cmd)
    cmd = reply(fsm, cmd, confirmed_owner="NAVIGATION")
    assert cmd.task_id == "two"


def test_relocalization_retries_bounded_and_unknown_never_retries():
    for terminal in (True, False):
        fsm = make_fsm()
        cmd = reply(fsm, fsm.start(), "health_failed")
        cmd = reply(fsm, cmd, confirmed_owner="SAFE_HOLD")
        cmd = reply(fsm, cmd, "aborted", terminal)
        if terminal:
            assert cmd.kind == "relocalize"
            reply(fsm, cmd, "aborted")
        assert fsm.state == State.ABORTED and not fsm.route.completed


def test_multiple_obstacles_have_full_transitions():
    fsm = make_fsm(obstacles=3)
    at_stop(fsm)
    for index in range(3):
        cmd = complete_obstacle(fsm)
        if index < 2:
            cmd = reply(fsm, cmd)
            reply(fsm, cmd)
            assert fsm.state == State.WAIT_STOP
    assert cmd.task_id == "two"
    assert fsm.route.completed == ["one", "obstacle0", "obstacle1", "obstacle2"]

