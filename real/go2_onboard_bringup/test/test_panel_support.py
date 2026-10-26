from pathlib import Path
import sys
import time
import pytest
from go2_onboard_bringup.panel_support import Processes, run_command, saved_path, apply_localization


def test_process_group_start_stop(tmp_path):
    processes=Processes(tmp_path)
    processes.start('test',[sys.executable,'-c','import time; time.sleep(30)'])
    assert processes.alive('test')
    with pytest.raises(ValueError):
        processes.start('test',[sys.executable,'-c','pass'])
    processes.stop_all()
    assert not processes.alive('test')


def test_command_failure_and_timeout():
    assert 'hello' in run_command([sys.executable,'-c','print("hello")'])
    with pytest.raises(RuntimeError):
        run_command([sys.executable,'-c','raise SystemExit(1)'])
    with pytest.raises(RuntimeError):
        run_command([sys.executable,'-c','import time; time.sleep(10)'],0.1)


def test_saved_map_validation(tmp_path):
    p=tmp_path/'map.pcd'
    p.write_text('FIELDS x y z\nPOINTS 1\nDATA ascii\n0 0 0\n')
    assert saved_path('SAVED '+str(p)+'\nPOINTS 1')==str(p)
    with pytest.raises(ValueError):saved_path('no map')
    with pytest.raises(ValueError):saved_path('SAVED '+str(tmp_path/'missing.pcd'))


def test_invalid_pose_does_not_write(tmp_path):
    config=tmp_path/'config';config.mkdir()
    original='unchanged'
    (config/'profile.yaml').write_text(original)
    p=tmp_path/'map.pcd';p.write_text('FIELDS x y z\nPOINTS 1\nDATA ascii\n0 0 0\n')
    with pytest.raises(ValueError):apply_localization(config,str(p),[0,0])
    assert (config/'profile.yaml').read_text()==original


def test_apply_preserves_parameters_and_backup(tmp_path):
    import yaml
    from go2_onboard_bringup.profile import create_profile
    folder=tmp_path/'config'
    create_profile(str(folder),'192.168.1.50','192.168.1.111',[0.0]*6)
    original=(folder/'profile.yaml').read_bytes()
    lidar=(folder/'mid360.json').read_bytes()
    nav=(folder/'nav2.yaml').read_bytes()
    p=tmp_path/'map.pcd';p.write_text('FIELDS x y z\nPOINTS 1\nDATA ascii\n0 0 0\n')
    backup=apply_localization(folder,str(p),[1,2,3,0,0,0.5])
    data=yaml.safe_load((folder/'profile.yaml').read_text())
    assert data['host_ip']=='192.168.1.50'
    assert data['initial_pose']==[1,2,3,0,0,0.5]
    assert data['map_pcd']==str(p)
    assert (backup/'profile.yaml').read_bytes()==original
    assert (folder/'mid360.json').read_bytes()==lidar
    assert (folder/'nav2.yaml').read_bytes()==nav
