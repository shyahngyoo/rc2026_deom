from dataclasses import replace
import math
import pytest
import yaml
from go2_mission_backends.skill_executor import SkillConfig, TestSkillExecutor, load_skills


def config(mode="success"):
    return SkillConfig("test_walk", "test", mode, 1., (.1, .02, .03), 3.)


def test_test_executor_requires_explicit_simulation():
    with pytest.raises(ValueError, match="FORBIDDEN"):
        TestSkillExecutor(config())


def test_progress_output_finish_and_release():
    e = TestSkillExecutor(config(), simulation_mode=True)
    e.start(10.)
    u = e.tick(10.5)
    assert u.phase == "running" and u.progress == .5 and u.velocity == (.1, .02, .03)
    u = e.tick(11.1)
    assert u.terminal == "completed" and u.velocity == (0., 0., 0.)
    assert e.close(11.2)
    with pytest.raises(RuntimeError):
        e.tick(11.3)


def test_wrong_exit_reports_executor_completion_without_motion_or_arrival():
    e = TestSkillExecutor(config("wrong_exit"), simulation_mode=True)
    e.start(0.)
    assert e.tick(.4).velocity == (0., 0., 0.)
    assert e.tick(1.).terminal == "completed"


@pytest.mark.parametrize("mode", ["timeout", "cancel_unconfirmed"])
def test_timeout_modes_never_self_complete(mode):
    e = TestSkillExecutor(config(mode), simulation_mode=True)
    e.start(0.)
    assert not e.tick(100.).terminal


def test_fail_and_exception_are_explicit():
    e = TestSkillExecutor(config("fail"), simulation_mode=True)
    e.start(0.)
    assert e.tick(.6).error_code == "SKILL_EXECUTION_FAILED"
    e = TestSkillExecutor(config("crash"), simulation_mode=True)
    e.start(0.)
    with pytest.raises(RuntimeError, match="CRASH"):
        e.tick(.6)


@pytest.mark.parametrize("mode,expected", [("success", True), ("cancel_unconfirmed", False)])
def test_cancel_confirmation(mode, expected):
    e = TestSkillExecutor(config(mode), simulation_mode=True)
    e.start(0.)
    assert e.cancel(.1) is expected
    if expected:
        assert e.tick(.2).terminal == "canceled"
        assert e.tick(.2).velocity == (0., 0., 0.)


def test_release_failure_is_not_acknowledged():
    e = TestSkillExecutor(config("release_failure"), simulation_mode=True)
    e.start(0.)
    assert e.tick(2.).terminal == "completed"
    assert not e.close(2.1)


def test_double_start_and_missing_start():
    e = TestSkillExecutor(config(), simulation_mode=True)
    with pytest.raises(RuntimeError):
        e.tick(0.)
    e.start(0.)
    with pytest.raises(RuntimeError):
        e.start(.1)


def test_catalog_allowlist_enabled_and_unknown_real_executor(tmp_path):
    p = tmp_path / "skills.yaml"
    p.write_text(yaml.safe_dump({"skills": {"walk": {"executor": "test"}, "jump": {"executor": "real_team"}}}))
    skills = load_skills(str(p), allowed=["walk"])
    assert skills["walk"].enabled
    assert not skills["jump"].enabled
    assert skills["jump"].executor == "real_team"
    assert load_skills("") == {}


@pytest.mark.parametrize("changes", [{"duration": 0}, {"duration": math.nan}, {"execution_timeout": -1},
                                    {"velocity": [0, 0]}, {"velocity": [0, math.inf, 0]},
                                    {"enabled": "true"}, {"mode": "unknown"}, {"executor": ""},
                                    {"velocity": [True, 0, 0]}])
def test_invalid_configuration_rejected(tmp_path, changes):
    p = tmp_path / "skills.yaml"
    p.write_text(yaml.safe_dump({"skills": {"walk": dict(executor="test", **{k:v for k,v in changes.items() if k != "executor"})}}))
    if "executor" in changes:
        p.write_text(yaml.safe_dump({"skills": {"walk": changes}}))
    with pytest.raises(ValueError):
        load_skills(str(p))


def test_duplicate_skill_mapping_is_rejected(tmp_path):
    p=tmp_path/"duplicate.yaml"
    p.write_text("skills:\n  walk:\n    executor: test\n    mode: success\n  walk:\n    executor: test\n    mode: wrong_exit\n")
    with pytest.raises(ValueError, match="unique"):
        load_skills(str(p))
