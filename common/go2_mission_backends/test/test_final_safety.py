"""Independent final safety regressions against production callbacks.

No hardware, publishers, timers, or motor commands are constructed here.
The PRECHECK test uses a spy factory so a no-op simulation start cannot hide
an unwanted SDK/executor start after an accepted cancellation request.
"""
from types import SimpleNamespace
import threading
import time
import pytest
from go2_obstacle_interfaces.msg import LocalizationEvidence
from robocon_mission.localization_evidence import Evidence, EvidenceHealth
from go2_mission_backends.obstacle_adapter import ObstacleAdapter
from go2_mission_backends.skill_executor import SkillConfig, SkillUpdate


def test_invalid_ros_stamp_fields_cannot_become_accepted_localization():
    m=LocalizationEvidence();m.header.frame_id='map';m.body_frame='livox_frame'
    m.header.stamp.sec=0;m.header.stamp.nanosec=1500000000
    m.source_id='source';m.epoch=1;m.sequence=1;m.accepted=True
    m.pose.pose.orientation.w=1.
    # Sum(sec*1e9+nanosec) looks like a plausible 1.5s stamp but this is NOT a
    # valid builtin_interfaces/Time. Both adapter and Mission share this parser.
    h=EvidenceHealth(wall_clock=lambda:1.)
    assert not h.observe(Evidence.from_message(m),1600000000)
    assert not h.phase_fresh(1600000000)


def test_precheck_cancellation_never_starts_executor_even_when_safe_state_arrives():
    starts=[];finished=[];commands=[]
    class SpyExecutor:
        simulation_only=True
        def __init__(self,config,simulation_mode=False):pass
        def start(self,now):starts.append(now)
        def tick(self,now):return SkillUpdate('running',0.,(0.,0.,0.))
        def cancel(self,now):return True
        def close(self,now):return True
    node=ObstacleAdapter.__new__(ObstacleAdapter)
    node.lock=threading.RLock()
    node.factories={'spy':SpyExecutor}
    p=dict(simulation_mode=True,state_timeout=.5,release_timeout=2.)
    node.p=lambda name:p[name]
    now=time.monotonic()
    op=dict(phase='PRECHECK',cancel_requested=True,deadline=now+5.,started=now,
        skill=SkillConfig('one','spy'),executor=None,last_valid_state=now,
        finishing_at=0.,released_at=0.,terminal='',state_requested=now)
    node.op=op
    node._authorized=lambda _op:True
    node._query=lambda _op,_now:None
    node._measured=lambda _op,safe=False,after=0.:True
    node._command=lambda _op,v:commands.append(v)
    node._event=lambda *_args:None
    node._finish=lambda _op,kind,error='':finished.append((kind,error))
    node._fault=lambda _op,error:finished.append(('failed',error))
    node.get_logger=lambda:SimpleNamespace(error=lambda _s:None)
    for _ in range(3):node._tick()
    assert starts==[], 'accepted cancellation must not trigger executor.start()'
    assert all(v==(0.,0.,0.) for v in commands)
    assert not any(kind=='completed' for kind,_ in finished)



def test_no_test_executor_available_when_simulation_mode_is_false(tmp_path):
    from go2_mission_backends.skill_executor import load_skills, TestSkillExecutor
    path=tmp_path/'skills.yaml'
    path.write_text('skills:\n  step:\n    executor: test\n')
    skills=load_skills(path)
    node=ObstacleAdapter.__new__(ObstacleAdapter)
    node.factories={'test':TestSkillExecutor}
    node.p=lambda name:False if name=='simulation_mode' else None
    assert not node._available(skills['step'])
    with pytest.raises(ValueError,match='FORBIDDEN'):
        TestSkillExecutor(skills['step'],simulation_mode=False)


def test_unknown_covariance_never_becomes_zero_certainty():
    import numpy as np
    from go2_mission_backends.robot_state import checked_covariance
    covariance,unknown=checked_covariance([0.]*36)
    assert unknown and np.all(np.diag(covariance)==1e6)


def test_skill_mapping_duplicate_key_cannot_silently_replace_skill(tmp_path):
    from go2_mission_backends.skill_executor import load_skills
    path=tmp_path/'duplicate.yaml'
    path.write_text('skills:\n  step:\n    executor: test\n  step:\n    executor: different_executor\n')
    with pytest.raises(ValueError):load_skills(path)



def test_delayed_post_fault_measurement_can_recover_without_chasing_receive_time():
    def sample(stamp,sequence,accepted=True):
        return Evidence(stamp,'map','livox_frame','source',1,sequence,accepted,False,
            (0.,0.,0.),(0.,0.,0.,1.),(0.,)*36,'injected_rejection')
    h=EvidenceHealth(wall_clock=lambda:1.)
    assert h.observe(sample(1000000000,1),1050000000)
    assert not h.observe(sample(1100000000,2,False),1100000000)
    h.require_new(1100000000)
    assert not h.observe(sample(1080000000,3),1150000000)
    # Measurement 1.12 is newer than the actual fault at1.10. Its 80ms
    # delivery delay must not force it behind a continually moving boundary.
    assert h.observe(sample(1120000000,4),1200000000)
    assert h.phase_fresh(1200000000)
