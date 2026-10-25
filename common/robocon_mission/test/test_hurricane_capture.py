import math
from pathlib import Path

import pytest
import yaml

from robocon_mission.hurricane_capture import (
    FieldCalibration, LABELS, SKILL_IDS, main,
)
from robocon_mission.route_manager import RouteManager


def complete_field():
    field = FieldCalibration()
    field.record("start", [10.0, 20.0, -0.2])
    field.record("finish", [10.0, 25.0, 0.7])
    for index, skill in enumerate(SKILL_IDS):
        field.record(f"{skill}.begin", [10.0 + index, 20.0, -0.5])
        field.record(f"{skill}.entry", [10.0 + index, 20.2, 0.5])
        field.record(f"{skill}.exit", [10.0 + index, 21.0, 1.0])
    return field


def subset_field(*skills):
    full = complete_field()
    labels = ['start', 'finish'] + [f'{s}.{p}' for s in skills for p in ('begin', 'entry', 'exit')]
    return FieldCalibration({label: full.samples[label] for label in labels})


def test_selected_obstacle_exports_with_five_points(tmp_path):
    field = subset_field('bridge_b')
    source = Path(__file__).resolve().parent.parent / 'config/hurricane_skills.yaml'
    route = field.export_route(tmp_path / 'route.yaml', ['bridge_b'])
    skills = field.export_skills(source, tmp_path / 'skills.yaml', ['bridge_b'])
    assert [task.id for task in RouteManager.load(route).tasks] == ['bridge_b']
    data = yaml.safe_load(skills.read_text())
    assert list(data['skills']) == ['bridge_b']
    assert data['skills']['bridge_b']['calibrated']
    field.save(tmp_path / 'field.yaml')
    assert not FieldCalibration.load(tmp_path / 'field.yaml').calibrated


def test_selected_anchor_requires_only_origin_without_exporting_dependency(tmp_path):
    field = subset_field('bridge_a')
    source = Path(__file__).resolve().parent.parent / 'config/hurricane_skills.yaml'
    output = tmp_path / 'skills.yaml'
    output.write_text('previous output')
    with pytest.raises(ValueError, match=r'bridge_b.begin'):
        field.export_skills(source, output, ['bridge_a'])
    assert output.read_text() == 'previous output'
    field.record('bridge_b.begin', complete_field().samples['bridge_b.begin'])
    field.export_skills(source, output, ['bridge_a'])
    data = yaml.safe_load(output.read_text())
    assert list(data['skills']) == ['bridge_a']
    full = complete_field().export_skills(source, tmp_path / 'full.yaml')
    assert data['skills']['bridge_a'] == yaml.safe_load(full.read_text())['skills']['bridge_a']


@pytest.mark.parametrize('selection', [[], ['unknown'], ['bridge_a', 'bridge_a']])
def test_invalid_selection_never_overwrites_output(tmp_path, selection):
    output = tmp_path / 'route.yaml'
    output.write_text('previous output')
    with pytest.raises(ValueError):
        complete_field().export_route(output, selection)
    assert output.read_text() == 'previous output'


def test_subset_cli_uses_course_order_and_preserves_capture_store(tmp_path):
    field = subset_field('bridge_b', 'step')
    path = field.save(tmp_path / 'field.yaml')
    before = path.read_bytes()
    route, skills = tmp_path / 'route.yaml', tmp_path / 'skills.yaml'
    main(['--file', str(path), '--select', 'step', 'bridge_b',
          '--export-route', str(route), '--export-skills', str(skills)])
    assert [task.id for task in RouteManager.load(route).tasks] == ['bridge_b', 'step']
    assert list(yaml.safe_load(skills.read_text())['skills']) == ['bridge_b', 'step']
    assert path.read_bytes() == before


def test_selected_missing_entry_is_reported(tmp_path):
    field = subset_field('bridge_b')
    del field.samples['bridge_b.entry']
    with pytest.raises(ValueError, match=r'bridge_b.entry'):
        field.export_route(tmp_path / 'route.yaml', ['bridge_b'])


@pytest.mark.parametrize("pose", [
    [math.nan, 0, 0], [0, math.inf, 0], [0, 0, -math.inf],
    [0, True, 0], [0, "1", 0], [0, 0], {"x": 0, "y": 0},
])
def test_rejects_invalid_map_pose(pose):
    with pytest.raises(ValueError):
        FieldCalibration().record("start", pose)


def test_rejects_unknown_label_and_nonmap_file(tmp_path):
    with pytest.raises(ValueError, match="unknown"):
        FieldCalibration().record("bridge_A.begin", [0, 0, 0])
    path = tmp_path / "field.yaml"
    path.write_text("version: 1\nframe_id: odom\nsamples: {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="map"):
        FieldCalibration.load(path)


def test_partial_capture_roundtrip_and_export_rejected(tmp_path):
    field = FieldCalibration()
    field.record("start", {"x": 1, "y": 2, "yaw": 0.1})
    path = tmp_path / "field.yaml"
    field.save(path)
    assert yaml.safe_load(path.read_text(encoding="utf-8"))["calibrated"] is False
    restored = FieldCalibration.load(path)
    assert restored.samples["start"] == (1.0, 2.0, 0.1)
    assert not restored.calibrated
    assert len(restored.missing) == len(LABELS) - 1
    with pytest.raises(ValueError, match="missing map samples"):
        restored.export_route(tmp_path / "route.yaml")
    with pytest.raises(ValueError, match="missing map samples"):
        restored.export_skills(tmp_path / "profile.yaml", tmp_path / "skills.yaml")
    assert not (tmp_path / "route.yaml").exists()


def test_reference_cannot_supply_real_map_samples(tmp_path):
    path = Path(__file__).resolve().parent.parent / "config" / "hurricane_field_reference.yaml"
    reference = FieldCalibration.load(path)
    assert len(reference.missing) == len(LABELS)
    assert not reference.calibrated
    with pytest.raises(ValueError, match="missing"):
        reference.export_route(tmp_path / "route.yaml")
    reference.save(tmp_path / "new_field.yaml")
    data = yaml.safe_load((tmp_path / "new_field.yaml").read_text(encoding="utf-8"))
    assert data["source_reference"]["calibrated"] is False


def test_complete_route_uses_samples_in_map_and_returns_start(tmp_path):
    field = complete_field()
    field.save(tmp_path / "field.yaml")
    restored = FieldCalibration.load(tmp_path / "field.yaml")
    assert restored.calibrated
    path = restored.export_route(tmp_path / "route.yaml")
    route = RouteManager.load(path)
    assert route.frame_id == "map"
    assert route.return_pose == (10.0, 20.0, -0.2)
    assert tuple(task.id for task in route.tasks) == SKILL_IDS
    assert tuple(task.skill_id for task in route.tasks) == SKILL_IDS
    assert all(task.type == "obstacle" for task in route.tasks)
    assert route.tasks[0].transit_pose == (10.0, 20.0, -0.5)
    assert route.tasks[0].approach_pose == (10.0, 20.2, 0.5)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert data["route"][0]["end_pose"] == [10.0, 21.0, 1.0]
    assert route.tasks[0].end_pose == (10.0, 21.0, 1.0)
    assert set(data) == {"start_area", "frame_id", "route", "return"}


def test_skill_origins_use_field_axis_not_begin_robot_yaw(tmp_path):
    field = complete_field()
    profile = {
        "version": 1,
        "coordinate_convention": "hurricane_reflected_y",
        "skills": {skill: {"calibrated": False, "origin": [0, 0, 0],
                           "steps": [{"type": "waypoint", "target": [1, 2, 90]}]}
                   for skill in SKILL_IDS},
    }
    source = tmp_path / "profile.yaml"
    source.write_text(yaml.safe_dump(profile), encoding="utf-8")
    output = field.export_skills(source, tmp_path / "skills.yaml")
    result = yaml.safe_load(output.read_text(encoding="utf-8"))
    first = result["skills"]["bridge_a"]
    assert first["origin"] == pytest.approx([10.0, 20.0, math.pi / 2])
    assert first["entry_pose"] == [10.0, 20.2, 0.5]
    assert first["expected_exit_pose"] == [10.0, 21.0, 1.0]
    assert all(value["calibrated"] for value in result["skills"].values())
    assert first["steps"][0]["target"] == [1, 2, 90]
    assert yaml.safe_load(source.read_text(encoding="utf-8")) == profile


def test_degenerate_axis_and_incomplete_skill_profile_refused(tmp_path):
    field = complete_field()
    field.record("finish", field.samples["start"])
    assert not field.calibrated
    with pytest.raises(ValueError, match="distinct"):
        field.export_route(tmp_path / "route.yaml")
    field.record("finish", [11, 20, 0])
    path = tmp_path / "profile.yaml"
    path.write_text("skills: {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="all eight"):
        field.export_skills(path, tmp_path / "skills.yaml")


def test_false_complete_marker_and_nan_in_file_refused(tmp_path):
    path = tmp_path / "field.yaml"
    path.write_text("version: 1\nframe_id: map\ncalibrated: true\nsamples: {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="incomplete"):
        FieldCalibration.load(path)
    path.write_text("version: 1\nframe_id: map\nsamples: {start: [.nan, 0, 0]}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="finite"):
        FieldCalibration.load(path)


def test_offline_cli_capture_does_not_import_ros(tmp_path, capsys):
    path = tmp_path / "field.yaml"
    main(["--file", str(path), "--label", "start", "--pose", "1", "2", "0.3"])
    assert FieldCalibration.load(path).samples["start"] == (1.0, 2.0, 0.3)
    assert "Captured start" in capsys.readouterr().out


def anchored_profile(tmp_path, anchor="bridge_b", offset=None):
    profile = {
        "version": 1,
        "coordinate_convention": "hurricane_reflected_y",
        "skills": {skill: {"calibrated": False, "origin": [0, 0, 0],
                           "steps": [{"type": "waypoint", "target": [0, 0, 0]}]}
                   for skill in SKILL_IDS},
    }
    profile["skills"]["bridge_a"]["steps"] = [{
        "type": "waypoint", "target": [0.5, -4.08, 180.0],
        "anchor_skill": anchor, "anchor_offset": [0.5, 0.05] if offset is None else offset,
    }, {
        "type": "waypoint", "target": [0, -4.08, 180.0],
        "anchor_skill": "bridge_b", "anchor_offset": [0, 0.05],
    }]
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    return path


def test_cross_obstacle_waypoints_follow_surveyed_anchor_and_axis(tmp_path):
    field = complete_field()
    field.record("bridge_b.begin", [12.0, 23.0, -0.7])
    source = anchored_profile(tmp_path)
    output = field.export_skills(source, tmp_path / "skills.yaml")
    document = yaml.safe_load(output.read_text(encoding="utf-8"))
    steps = document["skills"]["bridge_a"]["steps"]
    assert steps[0]["target"] == pytest.approx([3.5, 2.05, 180.0])
    assert steps[1]["target"] == pytest.approx([3.0, 2.05, 180.0])
    assert steps[0]["anchor_skill"] == "bridge_b"
    # Re-exporting an already calibrated profile recalculates, without
    # cumulatively adding the original offset a second time.
    field.export_skills(output, tmp_path / "skills_again.yaml")
    again = yaml.safe_load((tmp_path / "skills_again.yaml").read_text(encoding="utf-8"))
    assert again["skills"]["bridge_a"]["steps"] == steps
    original = yaml.safe_load(source.read_text(encoding="utf-8"))
    assert original["skills"]["bridge_a"]["steps"][0]["target"] == [0.5, -4.08, 180.0]


@pytest.mark.parametrize("anchor,offset", [
    ("missing_skill", [0, 0]), ("bridge_b", [0]), ("bridge_b", [math.nan, 0]),
    ("bridge_b", [True, 0]),
])
def test_invalid_anchor_calibration_is_refused(tmp_path, anchor, offset):
    source = anchored_profile(tmp_path, anchor, offset)
    with pytest.raises(ValueError):
        complete_field().export_skills(source, tmp_path / "skills.yaml")
    assert not (tmp_path / "skills.yaml").exists()


def test_measured_exit_mismatch_is_recorded_without_changing_waypoints(tmp_path):
    field = complete_field()
    field.record("bridge_b.begin", [12.0, 23.0, -0.7])
    source = anchored_profile(tmp_path)
    output = field.export_skills(source, tmp_path / "skills.yaml")
    document = yaml.safe_load(output.read_text(encoding="utf-8"))
    warning = next(item for item in document["calibration"]["warnings"]
                   if item["skill_id"] == "bridge_a")
    assert warning["configured_exit_pose"] == pytest.approx([12.05, 23.0, -math.pi / 2])
    assert warning["measured_exit_pose"] == [10.0, 21.0, 1.0]
    assert warning["position_error_m"] > 0.1
    assert document["skills"]["bridge_a"]["expected_exit_pose"] == [10.0, 21.0, 1.0]
