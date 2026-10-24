import copy
import json
import sys
from pathlib import Path
import pytest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from go2_onboard_bringup.profile import create_profile,load_profile,validate_profile,footprint
from go2_onboard_bringup.configure import main as configure
from go2_onboard_bringup.doctor import static_report


@pytest.fixture
def inputs(tmp_path):
    pcd=tmp_path/'room.pcd'
    pcd.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n')
    image=tmp_path/'map.pgm';image.write_bytes(b'P5\n4 4\n255\n'+bytes([254])*16)
    nav=tmp_path/'map.yaml';nav.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.1,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.25)))
    return dict(config_dir=tmp_path/'profile',host_ip='192.168.1.50',lidar_ip='192.168.1.111',
        imu_in_base=[.11,-.07,.08,.01,.02,-.03],stage='localization',map_pcd=str(pcd),
        initial_pose=[1.,2.,.3,.0,.0,.6],nav_map=str(nav),
        measured_footprint=[[-.4,-.2],[.4,-.2],[.4,.2],[-.4,.2]])


def test_explicit_profile_round_trip_and_no_automatic_motion(inputs):
    folder=create_profile(**inputs)
    profile,_=load_profile(folder,'navigation')
    assert profile['imu_in_base']['xyz']==[.11,-.07,.08]
    assert not profile['motion_output_enabled'] and not profile['mission_autostart']
    gicp=yaml.safe_load((folder/'gicp.yaml').read_text())['gicp_localizer']['ros__parameters']
    assert gicp['initial_pose_enabled'] and gicp['initial_pose']==inputs['initial_pose']
    report=static_report(folder,'navigation')
    assert report['configuration_valid'] and report['status']=='NOT_READY'
    assert report['planning_only'] and not report['mission_allowed']
    assert 'REAL_LOW_LEVEL_STATE_NOT_CONNECTED' in report['reasons']


def test_sensor_only_needs_no_map_but_cannot_localize(inputs):
    inputs.update(stage='sensor_only',map_pcd=None,initial_pose=None,nav_map=None,measured_footprint=None)
    folder=create_profile(**inputs)
    load_profile(folder,'lidar');load_profile(folder,'odometry')
    with pytest.raises(ValueError):load_profile(folder,'localization')
    params=yaml.safe_load((folder/'gicp.yaml').read_text())['gicp_localizer']['ros__parameters']
    assert params['initial_pose_enabled'] is False


@pytest.mark.parametrize('key,value',[('host_ip','127.0.0.1'),('host_ip','0.0.0.0'),('lidar_ip','224.1.2.3'),
 ('lidar_ip','not_ip'),('lidar_ip','192.168.1.50'),('imu_in_base',[1.,2.]),
 ('imu_in_base',[float('nan')]*6),('initial_pose',None),('map_pcd',None),('map_pcd','relative.pcd')])
def test_bad_required_inputs_rejected_before_write(inputs,key,value):
    inputs[key]=value
    with pytest.raises(ValueError):create_profile(**inputs)
    assert not inputs['config_dir'].exists()


@pytest.mark.parametrize('field,value',[('source_imu_frame','base_link'),('source_odom_frame','camera_init'),
 ('odom_topic','/cmd_vel'),('use_sim_time',True),('imu_in_base_xyz',[.3,.1,.2])])
def test_modified_state_source_or_unmeasured_extrinsics_rejected(inputs,field,value):
    folder=create_profile(**inputs);p=folder/'robot_state.yaml';data=yaml.safe_load(p.read_text())
    data['robot_state_adapter']['ros__parameters'][field]=value;p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_profile(folder,'localization')


@pytest.mark.parametrize('field,value',[('motion_output_enabled',True),('mission_autostart',True),
 ('use_sim_time',True),('nav_output_topic','/cmd_vel'),('frames',dict(map='odom'))])
def test_manual_profile_cannot_unlock_robot_or_change_frames(inputs,field,value):
    folder=create_profile(**inputs);p=folder/'profile.yaml';data=yaml.safe_load(p.read_text());data[field]=value;p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_profile(folder,'navigation')


@pytest.mark.parametrize('missing',['footprint','nav_map'])
def test_navigation_requires_measured_geometry_and_map(inputs,missing):
    if missing=='footprint':inputs['measured_footprint']=None
    else:inputs['nav_map']=None
    folder=create_profile(**inputs)
    load_profile(folder,'localization')
    with pytest.raises(ValueError):load_profile(folder,'navigation')


@pytest.mark.parametrize('polygon',[[],[[0,0],[1,0]],[[0,0],[1,0],[2,0]],
 [[0,0],[1,1],[0,1],[1,0]],[[0,0],[1,0],[0,0]],[[0,0],[1,float('inf')],[0,1]]])
def test_invalid_footprints_rejected(polygon):
    with pytest.raises(ValueError):footprint(polygon)


def test_no_profile_and_cli_missing_explicit_measurements(tmp_path):
    with pytest.raises(ValueError):load_profile(tmp_path,'lidar')
    report=static_report(tmp_path,'localization')
    assert not report['configuration_valid'] and not report['mission_allowed']
    with pytest.raises(SystemExit):configure(['--host-ip','192.168.1.50','--lidar-ip','192.168.1.111'])


def test_cli_sensor_configuration_and_named_overwrite_only(tmp_path,capsys):
    path=tmp_path/'user_config'
    argv=['--config-dir',str(path),'--host-ip','192.168.1.50','--lidar-ip','192.168.1.111',
          '--imu-in-base','.11','-.07','.08','.01','.02','-.03']
    assert configure(argv)==0
    message=json.loads(capsys.readouterr().out);assert message['motion_enabled'] is False
    keep=path/'unrelated.txt';keep.write_text('preserved')
    assert configure(argv)==2
    assert configure(argv+['--force'])==0 and keep.read_text()=='preserved'


def test_corrupted_gicp_initial_pose_and_nav_footprint_are_rejected(inputs):
    folder=create_profile(**inputs)
    p=folder/'gicp.yaml';data=yaml.safe_load(p.read_text());data['gicp_localizer']['ros__parameters']['initial_pose']=[0.]*6;p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_profile(folder,'localization')
    create_profile(**dict(inputs,force=True))
    p=folder/'nav2.yaml';data=yaml.safe_load(p.read_text());data['local_costmap']['local_costmap']['ros__parameters']['footprint']='[]';p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_profile(folder,'navigation')


def test_odometry_navigation_without_pcd_or_icp_guess(inputs):
    inputs.update(map_pcd=None,initial_pose=None,localization_source='odometry',
                  odom_in_map=[1.,2.,0.,0.,0.,.6])
    folder=create_profile(**inputs)
    profile,_=load_profile(folder,'navigation')
    assert profile['map_pcd'] is None
    params=yaml.safe_load((folder/'localization.yaml').read_text())['localization_evidence_adapter']['ros__parameters']
    assert params['input_topic']=='/odometry/evidence'
    assert params['evidence_max_age']==.5
    assert static_report(folder,'navigation')['absolute_drift_correction'] is False
    params['input_topic']='/gicp/evidence'
    (folder/'localization.yaml').write_text(yaml.safe_dump({'localization_evidence_adapter':{'ros__parameters':params}}))
    with pytest.raises(ValueError,match='evidence'):
        load_profile(folder,'navigation')


@pytest.mark.parametrize('alignment',[None,[0.]*5,[float('nan')]*6])
def test_odometry_requires_explicit_alignment(inputs,alignment):
    inputs.update(map_pcd=None,initial_pose=None,localization_source='odometry',odom_in_map=alignment)
    with pytest.raises(ValueError):create_profile(**inputs)


def test_odometry_rejects_ambiguous_icp_inputs(inputs):
    inputs.update(localization_source='odometry',odom_in_map=[0.]*6)
    with pytest.raises(ValueError,match='not GICP'):create_profile(**inputs)
