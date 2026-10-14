import textwrap

import pytest

from robocon_mission.route_manager import RouteManager


VALID = """
start_area: start_a
frame_id: map
route:
  - id: one
    type: navigation
    pose: [0.1, 0.2, 0.0]
  - id: stair
    type: obstacle
    transit_pose: [0.3, 0.2, 0.0]
    approach_pose: [0.4, 0.2, 0.0]
    skill_id: stair_jump
    end_pose: [0.8, 0.2, 0.0]
    timeout: 15.0
    max_retries: 1
return:
  pose: [0.0, 0.0, 0.0]
"""


def load(tmp_path, text):
    path = tmp_path / "route.yaml"
    path.write_text(textwrap.dedent(text), encoding="utf-8")
    return RouteManager.load(path)


def test_valid_route_and_index(tmp_path):
    route = load(tmp_path, VALID)
    assert route.current.id == "one"
    assert route.current.pose == (0.1, 0.2, 0.0)
    route.complete_current()
    assert route.current.id == "stair"
    assert route.current.transit_pose == (0.3, 0.2, 0.0)
    assert route.current.approach_pose == (0.4, 0.2, 0.0)
    assert route.current.skill_id == "stair_jump"
    route.complete_current()
    assert route.current is None
    assert route.completed == ["one", "stair"]
    assert route.return_pose == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("old,new", [
    ("id: stair", "id: one"),
    ("pose: [0.1, 0.2, 0.0]", "pose: [.nan, 0.2, 0.0]"),
    ("type: navigation", "type: flying"),
    ("skill_id: stair_jump", "other_field: x"),
    ("timeout: 15.0", "timeout: 0"),
    ("max_retries: 1", "max_retries: -1"),
    ("pose: [0.0, 0.0, 0.0]", "pose: [0.0, 0.0]"),
    ("  - id: one\n    type: navigation\n    pose: [0.1, 0.2, 0.0]\n"
     "  - id: stair\n    type: obstacle\n    transit_pose: [0.3, 0.2, 0.0]\n"
     "    approach_pose: [0.4, 0.2, 0.0]\n    skill_id: stair_jump\n"
     "    end_pose: [0.8, 0.2, 0.0]\n"
     "    timeout: 15.0\n    max_retries: 1",
     "  []"),
])
def test_rejects_invalid_route(tmp_path, old, new):
    with pytest.raises(ValueError):
        load(tmp_path, VALID.replace(old, new))


def test_missing_pose_and_empty_route(tmp_path):
    with pytest.raises(ValueError):
        load(tmp_path, VALID.replace("pose: [0.1, 0.2, 0.0]", "wrong: 1"))
    with pytest.raises(ValueError):
        load(tmp_path, VALID.split("route:")[0] +
             "route: []\nreturn:\n  pose: [0.0, 0.0, 0.0]\n")

@pytest.mark.parametrize("old,new", [
    ("    end_pose: [0.8, 0.2, 0.0]\n", ""),
    ("    end_pose: [0.8, 0.2, 0.0]", "    end_pose: [0.8, 0.2, true]"),
    ("    timeout: 15.0", "    transit_timeout: -1"),
    ("    timeout: 15.0", "    skill_timeout: true"),
    ("    timeout: 15.0", "    verify_timeout: .inf"),
    ("    timeout: 15.0", "    end_yaw_tolerance: 4.0"),
    ("    timeout: 15.0", "    unexpected: 5"),
    ("frame_id: map", "frame_id: map\nunknown: x"),
    ("frame_id: map", "frame_id: map\nframe_id: odom"),
    ("return:\n  pose:", "return:\n  unknown: 1\n  pose:"),
])
def test_strict_new_fields(tmp_path, old, new):
    with pytest.raises(ValueError):
        load(tmp_path, VALID.replace(old, new))


def test_independent_timeouts_and_legacy_timeout(tmp_path):
    route = load(tmp_path, VALID.replace(
        "timeout: 15.0", "timeout: 15.0\n    transit_timeout: 20.0\n"
        "    approach_timeout: 10.0\n    skill_timeout: 25.0\n"
        "    verify_timeout: 3.0\n    end_xy_tolerance: 0.1"))
    task = route.tasks[1]
    assert task.timeout == 15 and task.transit_timeout == 20
    assert task.approach_timeout == 10 and task.skill_timeout == 25
    assert task.verify_timeout == 3 and task.end_xy_tolerance == .1

