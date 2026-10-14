"""Non-ROS tests: angles, real pose evidence, stale samples and stable windows."""
import math
import pytest
from robocon_mission.end_pose_verifier import EndPoseVerifier, PoseSample, pose_errors
from robocon_mission.mission_fsm import Command


def verifier(target=(1.0, 0.0, 0.0), current=(1.0, 0.0, 0.0)):
    clock = [0.0]
    context = {"pose": current, "healthy": True, "stopped": True, "stamp": 1}
    outcomes = []
    v = EndPoseVerifier(lambda: PoseSample(context["pose"], context["stamp"]),
                        lambda: context["healthy"], lambda: context["stopped"],
                        0.5, lambda: clock[0])
    v.start(Command("verify_end", pose=target, token=7, timeout=2), 
            lambda *args: outcomes.append(args))
    return v, clock, context, outcomes


def advance(v, clock, context, steps=6):
    for _ in range(steps):
        clock[0] += 0.1
        context["stamp"] += 1
        v.tick()


def test_yaw_wrap_uses_current_yaw():
    assert pose_errors((0, 0, -math.pi + .05), (0, 0, math.pi - .05))[1] == pytest.approx(.1)


def test_continuous_fresh_correct_pose_required():
    v, clock, context, outcomes = verifier()
    advance(v, clock, context, 3)
    assert not outcomes
    context["healthy"] = False
    advance(v, clock, context, 1)
    context["healthy"] = True
    advance(v, clock, context, 7)
    assert outcomes[0][:3] == (7, "succeeded", True)


@pytest.mark.parametrize("change,reason", [
    ({"pose": (0, 0, 0)}, "xy error"),
    ({"pose": (1, 0, 1)}, "yaw error"),
    ({"healthy": False}, "untrusted"),
    ({"stopped": False}, "not stopped"),
    ({"pose": (math.nan, 0, 0)}, "nonfinite"),
])
def test_failures_have_specific_reasons(change, reason):
    v, clock, context, outcomes = verifier()
    context.update(change)
    advance(v, clock, context, 25)
    assert len(outcomes) == 1 and reason in outcomes[0][1]


def test_cached_transform_cannot_satisfy_window_and_cancel_isolated():
    v, clock, context, outcomes = verifier()
    for _ in range(15):
        clock[0] += .1
        v.tick()
    assert not outcomes
    v.cancel()
    assert outcomes[0][1] == "canceled"
    v.start(Command("verify_end", pose=(1, 0, 0), token=8, timeout=2),
            lambda *args: outcomes.append(args))
    advance(v, clock, context, 7)
    assert outcomes[-1][0:2] == (8, "succeeded")


def test_three_hz_fresh_samples_can_complete_stability_window():
    v, clock, context, outcomes = verifier()
    v.tick()
    for tick in range(1, 14):
        clock[0] = tick * .05
        if tick % 6 == 0:
            context["stamp"] += 1
        v.tick()
    assert outcomes[0][1] == "succeeded"


def test_gap_resets_even_when_no_tick_observed_the_missing_samples():
    v, clock, context, outcomes = verifier()
    v.tick()
    clock[0] = .8
    context["stamp"] += 1
    v.tick()
    assert not outcomes
    advance(v, clock, context, 7)
    assert outcomes[0][1] == "succeeded"


def test_two_samples_do_not_prove_continuity():
    v, clock, context, outcomes = verifier()
    v.max_sample_gap = .8
    v.tick()
    clock[0] = .6
    context["stamp"] += 1
    v.tick()
    assert not outcomes
    clock[0] = .7
    context["stamp"] += 1
    v.tick()
    assert outcomes[0][1] == "succeeded"

