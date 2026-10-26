"""Headless button workflow tests: no hardware processes are started."""
import queue
from pathlib import Path
from unittest.mock import Mock
import pytest
pytest.importorskip('rclpy')
from go2_onboard_bringup.operator_panel import Panel


def panel(tmp_path):
    p=Panel.__new__(Panel)
    p.procs=Mock()
    p.config=tmp_path
    p.mode='mapping'
    p.session_dir=tmp_path
    p.events=queue.Queue()
    return p


def test_save_only_after_mapping_and_success(tmp_path,monkeypatch):
    p=panel(tmp_path)
    mapfile=tmp_path/'test.pcd'
    mapfile.write_text('FIELDS x y z\nPOINTS 1\nDATA ascii\n0 0 0\n')
    monkeypatch.setattr('go2_onboard_bringup.operator_panel.run_command',lambda *a:'SAVED '+str(mapfile))
    p.save_map({})
    assert p.events.get()==('map',str(mapfile))
    p.mode='localization'
    with pytest.raises(ValueError):p.save_map({})


def test_only_keys_rejects_waypoint_before_stopping_processes(tmp_path):
    p=panel(tmp_path)
    plan=tmp_path/'plan.yaml'
    plan.write_text('steps: [{type: waypoint, x: 1, y: 2}]')
    with pytest.raises(ValueError):p.automatic({'plan':str(plan),'only_keys':True})
    p.procs.stop_all.assert_not_called()


def test_capture_uses_existing_localization(tmp_path):
    p=panel(tmp_path);p.mode='localization'
    p.capture({'field':str(tmp_path/'field.yaml')})
    args=p.procs.start.call_args.args
    assert args[0]=='capture'
    assert 'capture_panel' in args[1]
    p.procs.stop_all.assert_not_called()


def test_quick_course_keeps_odometry_running(tmp_path,monkeypatch):
    import yaml
    p=panel(tmp_path);p.sensor_mode='odometry';p.odom_session='same';p.capture_frame='nav_odom'
    p.launch_bridge=Mock()
    plan=tmp_path/'plan.yaml'
    plan.write_text(yaml.safe_dump(dict(frame='nav_odom',course_session='same',steps=[dict(type='waypoint',x=1,y=2)])))
    p.automatic(dict(plan=str(plan),only_keys=False))
    p.procs.stop_all.assert_not_called()
    assert all(call.args[0]!='sensors' for call in p.procs.stop.call_args_list)
    p.launch_bridge.assert_called_once()
    p.odom_session='changed'
    with pytest.raises(ValueError):p.automatic(dict(plan=str(plan),only_keys=False))
