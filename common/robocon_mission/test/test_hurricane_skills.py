"""Synthetic poses and trusted fake feedback only; never opens hardware."""

import copy
import math
from pathlib import Path

import pytest
import yaml

from robocon_mission.hurricane_skills import SkillRunner, load_skills


CONFIG = Path(__file__).resolve().parents[1] / "config" / "hurricane_skills.yaml"
SKILL_IDS = {"bridge_a", "bridge_b", "big_slope", "limit_pole", "bunker",
             "pole", "high_wall", "step"}


class FakeTransport:
    def __init__(self):
        self.sent = []

    def send(self, command, repeat=1):
        self.sent.append((command, repeat))


def make_runner(steps, **kwargs):
    transport = FakeTransport()
    skill = {"calibrated": True, "origin": [0.0, 0.0, 0.0], "steps": steps}
    runner = SkillRunner({"test": skill}, transport, **kwargs)
    runner.start("test", 0.0)
    return runner, transport


def waypoint(target, **kwargs):
    return {"type": "waypoint", "target": target, **kwargs}


def test_config_has_eight_distinct_designs_and_uncalibrated_default():
    skills = load_skills(CONFIG)
    assert set(skills) == SKILL_IDS
    assert all(skill["calibrated"] is False for skill in skills.values())
    assert len([s for s in skills["pole"]["steps"] if s["type"] == "waypoint"]) == 12
    assert [s["command"] for s in skills["step"]["steps"] if s["type"] == "jump"] == [
        "小跳1", "小跳2", "小跳2"]
    transport = FakeTransport()
    runner = SkillRunner(skills, transport)
    with pytest.raises(ValueError, match="calibration"):
        runner.start("bridge_a", 0.0)
    assert transport.sent == []


def test_target_behind_robot_is_not_reported_as_arrival():
    # Regression: Hurricane's signed distance skipped a 1.8 m target behind.
    runner, transport = make_runner([waypoint([-1.8, 0.0, 0.0])])
    assert runner.tick((0.0, 0.0, 0.0), 0.0) == "running"
    assert runner.index == 0
    assert transport.sent[-1][0] in {"左转", "右转"}
    assert runner.tick((0.0, 0.0, math.pi), 0.1) == "running"
    assert runner.index == 0
    assert transport.sent[-1][0] == "前进"


def test_require_yaw_false_still_requires_actual_position_arrival():
    runner, transport = make_runner([waypoint([1.8, 0.0, None], require_yaw=False)])
    # Arrival cannot be inferred from heading acceptance being disabled.
    assert runner.tick((0.0, 0.0, 1.0), 0.0) == "running"
    assert runner.index == 0
    assert runner.target == pytest.approx((1.8, 0.0, 1.0))
    assert transport.sent[-1][0] == "右转"
    # The source's None yaw imposes no terminal orientation condition.
    assert runner.tick((1.8, 0.0, -2.0), 0.1) == "awaiting_safe_stance"
    assert runner.index == 1
    assert transport.sent[-1][0] == "站立"


def test_require_yaw_defaults_to_true_for_dynamic_null_yaw():
    runner, transport = make_runner([waypoint([1.8, 0.0, None])])
    runner.tick((0.0, 0.0, 0.0), 0.0)
    assert runner.tick((1.8, 0.0, 0.5), 0.1) == "running"
    assert runner.index == 0
    assert transport.sent[-1][0] == "右转"


def test_every_jump_has_its_source_mode_preparation_and_confirmation_step():
    skills = load_skills(CONFIG)
    jump_modes = {"bridge_a": "木桥B模式", "big_slope": "木桥B模式",
                  "bunker": "沙坑模式", "high_wall": "高墙模式", "step": "台阶模式"}
    for skill_id, skill in skills.items():
        for index, step in enumerate(skill["steps"]):
            if step["type"] != "jump":
                continue
            assert index > 0
            preceding = skill["steps"][index - 1]
            assert preceding["type"] == "mode"
            assert preceding["command"] == jump_modes[skill_id]
            assert preceding["min_wait"] >= 1.0
    last_bunker_jump = max(i for i, s in enumerate(skills["bunker"]["steps"])
                          if s["type"] == "jump")
    assert skills["bunker"]["steps"][last_bunker_jump - 1]["min_wait"] == 3.0
    first_bridge_waypoint = next(s for s in skills["bridge_a"]["steps"]
                                if s["type"] == "waypoint")
    assert first_bridge_waypoint["require_yaw"] is False


def test_one_tick_never_consumes_two_waypoints():
    # Regression: the original callback invoked multiple navigation targets.
    runner, _ = make_runner([waypoint([0.0, 0.0, 0.0]),
                             waypoint([0.0, 0.0, 0.0])])
    assert runner.tick((0.0, 0.0, 0.0), 0.0) == "running"
    assert runner.index == 1
    assert runner.tick((0.0, 0.0, 0.0), 0.1) == "awaiting_safe_stance"
    assert runner.index == 2


def test_hurricane_y_and_controller_yaw_are_both_reflected():
    skill = {"calibrated": True, "origin": [10.0, 20.0, math.pi / 2],
             "steps": [waypoint([1.0, -2.0, -90.0])]}
    runner = SkillRunner({"test": skill}, FakeTransport())
    runner.start("test", 0.0)
    runner.tick((10.0, 20.0, 0.0), 0.0)
    assert runner.target[0:2] == pytest.approx((8.0, 21.0))
    assert abs(runner.target[2]) == pytest.approx(math.pi)


def test_null_target_components_are_frozen_at_waypoint_entry():
    runner, _ = make_runner([waypoint([None, -3.0, None])])
    runner.tick((1.0, 2.0, 0.25), 0.0)
    assert runner.target == pytest.approx((1.0, 3.0, 0.25))
    runner.tick((2.0, 2.0, 0.5), 0.1)
    assert runner.target == pytest.approx((1.0, 3.0, 0.25))


def test_yaw_wrap_chooses_short_turn_at_pi_boundary():
    runner, transport = make_runner([waypoint([0.0, 0.0, 178.0], yaw_tolerance_deg=1.0)])
    runner.tick((0.0, 0.0, math.radians(179.0)), 0.0)
    # Reflected target is -178 deg; shortest ROS turn is +3 deg.
    assert runner.index == 0
    assert transport.sent[-1][0] == "左转"


def test_reverse_motion_keeps_euclidean_arrival_check():
    runner, transport = make_runner([waypoint([-1.0, 0.0, 0.0], drive_direction="reverse")])
    runner.tick((0.0, 0.0, 0.0), 0.0)
    assert transport.sent[-1][0] == "后退"
    assert runner.index == 0


@pytest.mark.parametrize("kind,command,min_wait", [
    ("mode", "木桥B模式", 1.0), ("jump", "小跳1", 5.0),
])
def test_wait_never_proves_mode_or_jump_completed(kind, command, min_wait):
    runner, transport = make_runner([{"type": kind, "command": command,
                                     "min_wait": min_wait}])
    assert runner.tick((0.0, 0.0, 0.0), 0.0) == "awaiting_feedback"
    assert transport.sent == [(command, 2 if kind == "mode" else 1)]
    assert runner.tick((0.0, 0.0, 0.0), min_wait + 0.1) == "awaiting_feedback"
    assert runner.index == 0
    assert runner.tick((0.0, 0.0, 0.0), min_wait + 10.1) == "failed"
    assert "confirmation_timeout" in runner.error
    assert transport.sent[-1][0] == "站立"


def test_feedback_obeys_min_wait_and_jump_landing_gate():
    runner, _ = make_runner([{"type": "jump", "command": "小跳1", "min_wait": 5.0}])
    runner.tick((0.0, 0.0, 0.0), 0.0)
    assert runner.feedback(runner.session_id, 0, True, True, 0.1)
    assert runner.tick((0.0, 0.0, 0.0), 4.9) == "awaiting_feedback"
    assert runner.tick((0.0, 0.0, 0.0), 5.0) == "awaiting_safe_stance"

    other, _ = make_runner([{"type": "jump", "command": "小跳1", "min_wait": 5.0}])
    other.tick((0.0, 0.0, 0.0), 0.0)
    other.feedback(other.session_id, 0, True, False, 0.1)
    assert other.tick((0.0, 0.0, 0.0), 5.0) == "awaiting_feedback"
    assert other.feedback(other.session_id, 0, True, True, 5.1)
    assert other.tick((0.0, 0.0, 0.0), 5.2) == "awaiting_safe_stance"


def test_mode_feedback_cannot_be_reused_for_jump_or_terminal_stance():
    runner, _ = make_runner([{"type": "mode", "command": "木桥B模式", "min_wait": 0.0},
                             {"type": "jump", "command": "小跳1", "min_wait": 0.0}])
    runner.tick((0.0, 0.0, 0.0), 0.0)
    runner.feedback(runner.session_id, 0, True, True, 0.0)
    runner.tick((0.0, 0.0, 0.0), 0.1)
    assert runner.index == 1
    runner.tick((0.0, 0.0, 0.0), 0.2)
    assert not runner.feedback(runner.session_id, 0, True, True, 0.3)
    assert runner.tick((0.0, 0.0, 0.0), 0.4) == "awaiting_feedback"
    runner.feedback(runner.session_id, 1, True, True, 0.4)
    assert runner.tick((0.0, 0.0, 0.0), 0.5) == "awaiting_safe_stance"
    assert not runner.feedback(runner.session_id, 1, True, True, 0.5)
    assert runner.tick((0.0, 0.0, 0.0), 0.6) == "awaiting_safe_stance"


def test_future_dated_confirmation_is_not_consumed_early():
    runner, _ = make_runner([waypoint([0.0, 0.0, 0.0])])
    runner.tick((0.0, 0.0, 0.0), 0.0)
    assert runner.feedback(runner.session_id, 1, True, True, 0.5)
    assert runner.tick((0.0, 0.0, 0.0), 0.1) == "awaiting_safe_stance"
    assert runner.tick((0.0, 0.0, 0.0), 0.5) == "completed"


def test_allow_uncalibrated_requires_explicit_boolean():
    with pytest.raises(ValueError, match="allow_uncalibrated"):
        make_runner([waypoint([0.0, 0.0, 0.0])], allow_uncalibrated="false")


@pytest.mark.parametrize("steps", [
    [waypoint([1.0, 0.0, 0.0])],
    [{"type": "mode", "command": "正常模式"}],
    [{"type": "jump", "command": "小跳1"}],
])
def test_pose_expiration_stops_every_active_phase(steps):
    runner, transport = make_runner(steps, pose_timeout=0.5)
    runner.tick((0.0, 0.0, 0.0), 0.0)
    assert runner.tick(None, 0.51) == "failed"
    assert runner.error == "pose_stale"
    assert transport.sent[-1][0] == "站立"
    stopped_count = len(transport.sent)
    assert runner.tick(None, 100.0) == "failed"
    assert len(transport.sent) == stopped_count


def test_safe_stance_is_separate_feedback_and_outputs_stop_on_completion():
    runner, transport = make_runner([waypoint([0.0, 0.0, 0.0])])
    assert runner.tick((0.0, 0.0, 0.0), 0.0) == "awaiting_safe_stance"
    assert runner.index == 1
    assert runner.progress < 1.0
    assert not runner.feedback("other_session", 1, True, True, 0.1)
    assert not runner.feedback(runner.session_id, 0, True, True, 0.1)
    assert not runner.feedback(runner.session_id, 1, True, True, -1.0)
    assert runner.feedback(runner.session_id, 1, True, False, 0.1)
    assert runner.tick((0.0, 0.0, 0.0), 0.2) == "awaiting_safe_stance"
    assert runner.feedback(runner.session_id, 1, True, True, 0.3)
    assert runner.tick((0.0, 0.0, 0.0), 0.4) == "completed"
    assert runner.progress == 1.0
    count = len(transport.sent)
    assert runner.tick(None, 100.0) == "completed"
    assert len(transport.sent) == count


def test_missing_terminal_stance_times_out_and_stops():
    runner, transport = make_runner([waypoint([0.0, 0.0, 0.0])], confirmation_timeout=0.3)
    runner.tick((0.0, 0.0, 0.0), 0.0)
    assert runner.tick((0.0, 0.0, 0.0), 0.31) == "failed"
    assert runner.error == "safe_stance_confirmation_timeout"
    assert transport.sent[-1][0] == "站立"


def test_waypoint_timeout_has_no_unbounded_navigation_output():
    runner, transport = make_runner([waypoint([1.0, 0.0, 0.0])], step_timeout=0.2)
    runner.tick((0.0, 0.0, 0.0), 0.0)
    assert runner.tick((0.0, 0.0, 0.0), 0.21) == "failed"
    assert runner.error == "step_0_timeout"
    assert transport.sent[-1][0] == "站立"


def test_cancel_stops_and_next_run_has_a_new_feedback_session():
    runner, transport = make_runner([{"type": "mode", "command": "正常模式"}])
    old_session = runner.session_id
    with pytest.raises(RuntimeError):
        runner.start("test", 0.1)
    runner.cancel(0.1)
    assert runner.status == "canceled"
    assert transport.sent[-1][0] == "站立"
    runner.start("test", 0.2)
    runner.tick((0.0, 0.0, 0.0), 0.2)
    assert runner.session_id != old_session
    assert not runner.feedback(old_session, 0, True, True, 0.3)


def test_negative_hardware_feedback_fails_immediately():
    runner, transport = make_runner([{"type": "jump", "command": "小跳1"}])
    runner.tick((0.0, 0.0, 0.0), 0.0)
    assert runner.feedback(runner.session_id, 0, False, False, 0.1)
    assert runner.status == "failed"
    assert transport.sent[-1][0] == "站立"


def test_invalid_pose_and_clock_regression_fail_closed():
    runner, _ = make_runner([waypoint([1.0, 0.0, 0.0])])
    assert runner.tick((math.nan, 0.0, 0.0), 0.0) == "failed"
    assert runner.error == "pose_invalid"
    other, _ = make_runner([waypoint([1.0, 0.0, 0.0])])
    other.tick((0.0, 0.0, 0.0), 1.0)
    assert other.tick((0.0, 0.0, 0.0), 0.9) == "failed"
    assert other.error == "clock_regression"


def test_transport_error_does_not_report_success():
    class BrokenTransport:
        def send(self, command, repeat=1):
            raise OSError("offline")

    skill = {"calibrated": True, "origin": [0.0, 0.0, 0.0],
             "steps": [waypoint([1.0, 0.0, 0.0])]}
    runner = SkillRunner({"test": skill}, BrokenTransport())
    runner.start("test", 0.0)
    assert runner.tick((0.0, 0.0, 0.0), 0.0) == "failed"
    assert "transport_error" in runner.error


@pytest.mark.parametrize("skill_id", sorted(SKILL_IDS))
def test_all_designs_reach_a_bounded_terminal_gate_with_synthetic_feedback(skill_id):
    skills = load_skills(CONFIG)
    transport = FakeTransport()
    runner = SkillRunner(skills, transport, allow_uncalibrated=True)
    runner.start(skill_id, 0.0)
    pose = tuple(skills[skill_id]["origin"])
    now = 0.0
    for _ in range(200):
        runner.tick(pose, now)
        if runner.status == "awaiting_safe_stance":
            break
        assert runner.status != "failed", runner.error
        if runner.status == "awaiting_feedback":
            runner.feedback(runner.session_id, runner.index, True, True, now)
            now += skills[skill_id]["steps"][runner.index]["min_wait"] + 0.1
        elif runner.target is not None:
            pose = runner.target
            now += 0.1
        else:
            now += 0.1
    else:
        pytest.fail("skill did not finish within its configured finite steps")
    # Regression: bunker previously indexed beyond its last waypoint.
    assert runner.index == len(skills[skill_id]["steps"])
    runner.feedback(runner.session_id, runner.index, True, True, now)
    assert runner.tick(pose, now + 0.1) == "completed"
    count = len(transport.sent)
    for later in (now + 0.2, now + 1.0, now + 100.0):
        assert runner.tick(None, later) == "completed"
    assert len(transport.sent) == count


@pytest.mark.parametrize("change", [
    lambda data: data.update(version=2),
    lambda data: data.update(version=True),
    lambda data: data.update(coordinate_convention="map"),
    lambda data: data["skills"]["bridge_a"].update(calibrated="false"),
    lambda data: data["skills"]["bridge_a"].update(origin=[0.0, 0.0, math.nan]),
    lambda data: data["skills"]["bridge_a"]["steps"][0].update(command="unknown"),
    lambda data: data["skills"]["bridge_a"]["steps"][0].update(command=["正常模式"]),
    lambda data: data["skills"]["bridge_a"]["steps"][0].update(min_wait=-1.0),
    lambda data: data["skills"]["bridge_a"]["steps"][1].update(target=[0.0, 0.0]),
    lambda data: data["skills"]["bridge_a"]["steps"][1].update(require_yaw="false"),
])
def test_invalid_configuration_is_rejected_before_output(tmp_path, change):
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    change(data)
    path = tmp_path / "skills.yaml"
    path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    with pytest.raises(ValueError):
        load_skills(path)


def test_loader_preserves_calibration_and_provenance_extensions(tmp_path):
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    extended = copy.deepcopy(data)
    extended["skills"]["bridge_a"]["expected_exit_pose"] = [1.0, 2.0, 0.3]
    path = tmp_path / "skills.yaml"
    path.write_text(yaml.safe_dump(extended, allow_unicode=True), encoding="utf-8")
    assert load_skills(path)["bridge_a"]["expected_exit_pose"] == [1.0, 2.0, 0.3]
