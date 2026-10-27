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


def test_stop_sensors_keeps_panel_and_removes_only_sensor_processes(tmp_path):
    p=panel(tmp_path);p.procs.alive.return_value=False;p.odom_session='old'
    p.stop_sensors({})
    assert [c.args[0] for c in p.procs.stop.call_args_list]==['capture','rviz','sensors']
    assert p.odom_session is None and p.sensor_mode is None
    p.procs.stop_all.assert_not_called()


def test_stop_sensors_rejects_live_bridge(tmp_path):
    p=panel(tmp_path);p.procs.alive.return_value=True
    with pytest.raises(ValueError):p.stop_sensors({})
    p.procs.stop.assert_not_called()


def test_status_reasons_and_stale_ready():
    from go2_onboard_bringup.panel_view import status_summary
    assert status_summary({},False,None)[0]=='串口未启动'
    assert status_summary({},True,None)[0]=='等待状态'
    assert status_summary({'connected':False},True,0)[0]=='串口未连接'
    assert status_summary({'connected':True},True,0)[0]=='等待 STM32'
    assert status_summary({'connected':True,'status':{'mode':'z'}},True,0)[0]=='尚未初始化'
    ready={'connected':True,'ready':True,'status':{'mode':'a','fault':0}}
    assert status_summary(ready,True,0)[0]=='就绪：True'
    assert status_summary(ready,True,3)[0]=='状态已过期'
    ready['status']['fault']=1
    assert status_summary(ready,True,0)[0]=='电控故障'


def test_signal_close_waits_for_busy_operation(tmp_path):
    p=panel(tmp_path);p.busy=True;p.state=Mock();p.close=Mock()
    p.request_close()
    assert p.close_requested
    p.close.assert_not_called()
    p.busy=False;p.request_close()
    p.close.assert_called_once()


def test_saved_map_links_captured_start(tmp_path,monkeypatch):
    from go2_onboard_bringup.map_pose import read_start
    p=panel(tmp_path);p.mapping_start=dict(pose=[1,2,3,0,0,1],frame='camera_init')
    mapfile=tmp_path/'test.pcd';mapfile.write_text('FIELDS x y z\nPOINTS 1\nDATA ascii\n0 0 0\n')
    monkeypatch.setattr('go2_onboard_bringup.operator_panel.run_command',lambda *a:'SAVED '+str(mapfile)+'\nFRAME camera_init')
    p.save_map({})
    assert read_start(mapfile)==[1,2,3,0,0,1]


def test_saved_map_does_not_link_mismatched_frame(tmp_path,monkeypatch):
    p=panel(tmp_path);p.mapping_start=dict(pose=[1,2,3,0,0,1],frame='other')
    mapfile=tmp_path/'test.pcd';mapfile.write_text('FIELDS x y z\nPOINTS 1\nDATA ascii\n0 0 0\n')
    monkeypatch.setattr('go2_onboard_bringup.operator_panel.run_command',lambda *a:'SAVED '+str(mapfile)+'\nFRAME camera_init')
    assert '不一致' in p.save_map({})
    assert not Path(str(mapfile)+'.start.yaml').exists()


def test_match_rejects_stale_result(tmp_path):
    from geometry_msgs.msg import PoseStamped
    p=panel(tmp_path);p.sensor_mode='localization';p.match_pose=PoseStamped();p.match_valid=True
    p.node=Mock();p.node.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
    p.match_valid_time=0
    with pytest.raises(ValueError,match='过期'):p.use_match({})


def test_mapping_records_first_pose_with_ros_yaml_separator(tmp_path,monkeypatch):
    import yaml
    p=panel(tmp_path);p.real=Mock();p.rviz=Mock()
    monkeypatch.setattr(Path,'home',lambda:tmp_path)
    monkeypatch.setattr('go2_onboard_bringup.operator_panel.load_profile',lambda *a:None)
    msg=dict(header=dict(frame_id='camera_init'),pose=dict(pose=dict(position=dict(x=1,y=2,z=3),orientation=dict(x=0,y=0,z=0,w=1))))
    monkeypatch.setattr('go2_onboard_bringup.operator_panel.run_command',lambda *a:yaml.safe_dump(msg)+'---\n')
    p.mapping({'map_name':'test'})
    assert p.mapping_start['pose']==[1,2,3,0,0,0]
    assert p.events.get()==('pose',[1,2,3,0,0,0])


def test_retry_seed_uses_initialpose_without_restarting_sensors(tmp_path):
    import math
    from builtin_interfaces.msg import Time
    p=panel(tmp_path);p.sensor_mode='localization';p.active_map=str(tmp_path/'map.pcd')
    p.procs.alive.side_effect=lambda name:name=='sensors'
    p.node=Mock();p.node.get_clock.return_value.now.return_value.nanoseconds=1_000_000_000
    p.node.get_clock.return_value.now.return_value.to_msg.return_value=Time(sec=1)
    p.initial_pub=Mock()
    p.try_localization({'map':p.active_map,'pose':f'1 2 3 0 0 {math.pi}'})
    msg=p.initial_pub.publish.call_args.args[0]
    assert msg.header.frame_id=='map'
    assert msg.pose.pose.position.x==1
    assert msg.pose.pose.orientation.z==pytest.approx(1)
    p.procs.stop_all.assert_not_called()
    assert p.match_pose is None and not p.match_valid
