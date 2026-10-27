import math
from pathlib import Path
import pytest
from go2_onboard_bringup.map_pose import pose_vector,write_start,read_start,prepare_cloud


def test_pose_quaternion_normalized_and_finite():
    pose={'position':dict(x=1,y=2,z=.3),'orientation':dict(x=0,y=0,z=2,w=2)}
    assert pose_vector(pose)==pytest.approx([1,2,.3,0,0,math.pi/2])
    pose['orientation']=dict(x=0,y=0,z=0,w=0)
    with pytest.raises(ValueError):pose_vector(pose)


def test_start_survives_move_but_rejects_changed_map(tmp_path):
    p=tmp_path/'field.pcd';p.write_bytes(b'original map')
    expected=[1,2,.5,0,0,1]
    record=write_start(p,expected,'camera_init')
    other=tmp_path/'moved.pcd';p.rename(other);record.rename(str(other)+'.start.yaml')
    assert read_start(other)==expected
    other.write_bytes(b'changed map')
    with pytest.raises(ValueError,match='不匹配'):read_start(other)


def test_old_map_without_record(tmp_path):
    p=tmp_path/'old.pcd';p.write_bytes(b'old')
    with pytest.raises(ValueError,match='没有建图起点'):read_start(p)


def test_pcd_preview_filters_invalid_points(tmp_path):
    pytest.importorskip('numpy')
    p=tmp_path/'map.pcd'
    p.write_text('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n1 2 3\n4 5 6\nnan 2 3\n')
    xyz,(lo,hi)=prepare_cloud(p)
    assert len(xyz)==2
    assert list(lo)==[1,2] and list(hi)==[4,5]
