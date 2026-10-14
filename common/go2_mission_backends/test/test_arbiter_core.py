import math
import pytest
from go2_mission_backends.arbiter_core import Arbiter, Switch

BASE=10_000_000_000
V=(0.1,0,0,0,0,0)
Z=(0,)*6

def request(owner='NAVIGATION',expected='SAFE_HOLD',session='m',rid='r',lease=2.0):
    return Switch(owner,expected,session,rid,lease)
def granted():
    a=Arbiter();assert a.switch(request(),10,BASE,True).accepted;return a

def command(a,source='NAVIGATION',owner='NAVIGATION',session='m',gen=None,stamp=BASE+10_000_000,v=V,wall=10.01,ros=BASE+10_000_000,active=True):
    return a.receive(source,owner,session,a.generation if gen is None else gen,stamp,v,wall,ros,True,active)

@pytest.mark.parametrize('owner', ['NAVIGATION','SKILL'])
def test_single_owner(owner):
    a=Arbiter();assert a.switch(request(owner),10,BASE,True).accepted
    assert command(a,source=owner,owner=owner)
    assert a.output(10.02,BASE+20_000_000,True,True)==V
    other='SKILL' if owner=='NAVIGATION' else 'NAVIGATION'
    assert not command(a,source=other,owner=other,stamp=BASE+20_000_000,ros=BASE+20_000_000)
    assert a.output(10.02,BASE+20_000_000,True,True)==V

def test_no_owner():
    a=Arbiter();assert not command(a);assert a.output(10,BASE,True,True)==Z

@pytest.mark.parametrize('new_owner',['SKILL','SAFE_HOLD'])
def test_switch_flushes_old_commands(new_owner):
    a=granted();old=a.generation;assert command(a)
    assert a.switch(request(new_owner,'NAVIGATION',rid='new'),10.1,BASE+100_000_000,True).accepted
    assert a.command is None
    assert not command(a,source=new_owner,owner=new_owner,gen=old,stamp=BASE+110_000_000,wall=10.11,ros=BASE+110_000_000)
    assert a.output(10.12,BASE+120_000_000,True,True)==Z

def test_return_to_navigation_rejects_old_generation_and_timestamp():
    a=granted();old=a.generation
    a.switch(request('SKILL','NAVIGATION',rid='skill'),10.1,BASE+100_000_000,True)
    a.switch(request('NAVIGATION','SKILL',rid='nav'),10.2,BASE+200_000_000,True)
    assert not command(a,gen=old,stamp=BASE+210_000_000,wall=10.21,ros=BASE+210_000_000)
    assert not command(a,stamp=BASE+100_000_000,wall=10.21,ros=BASE+210_000_000)
    assert command(a,stamp=BASE+220_000_000,wall=10.22,ros=BASE+220_000_000)

def test_lease_expiry_and_stale_replay_do_not_restore():
    a=granted();r=request();deadline=a.deadline
    assert a.switch(r,11,BASE+1_000_000_000,True).accepted
    assert a.deadline==deadline
    a.tick(12.1,BASE+2_100_000_000)
    assert a.owner=='SAFE_HOLD'
    replay=a.switch(r,12.2,BASE+2_200_000_000,True)
    assert not replay.accepted and replay.owner=='SAFE_HOLD'
    assert a.owner=='SAFE_HOLD';assert not command(a,wall=12.2,ros=BASE+2_200_000_000)

def test_heartbeat_loss():
    a=granted();assert command(a)
    assert a.output(10.6,BASE+600_000_000,True,True)==Z
    assert a.owner=='SAFE_HOLD';assert a.reason=='COMMAND_HEARTBEAT_TIMEOUT'

def test_idle_owner_can_wait_stopped_but_cannot_emit_motion():
    a=granted();assert not command(a,active=False)
    assert a.output(11,BASE+1_000_000_000,True,False)==Z
    assert a.owner=='NAVIGATION'

def test_wrong_session_cannot_release():
    a=granted();assert not a.switch(request('SAFE_HOLD','NAVIGATION','intruder'),10.1,BASE+100_000_000,True).accepted
    assert a.owner=='NAVIGATION'

def test_conflicting_duplicate_fences():
    a=granted();assert not a.switch(request('SKILL','NAVIGATION'),10.1,BASE+100_000_000,True).accepted
    assert a.owner=='SAFE_HOLD'

@pytest.mark.parametrize('safe,fresh',[(False,True),(True,False),(False,False)])
def test_unconfirmed_handoff_rejected(safe,fresh):
    a=Arbiter();assert not a.switch(request(),10,BASE,safe,fresh).accepted
    assert a.owner=='SAFE_HOLD'

def test_state_loss_fences():
    a=granted();assert command(a)
    assert a.output(10.02,BASE+20_000_000,False,True)==Z
    assert a.owner=='SAFE_HOLD'

def test_clock_rollback_fences():
    a=granted();assert command(a)
    assert a.output(10.02,BASE-1,True,True)==Z
    assert a.owner=='SAFE_HOLD'

@pytest.mark.parametrize('overrides',[
    dict(source='SKILL'),dict(owner='SKILL'),dict(session='old'),dict(gen=0),
    dict(stamp=BASE),dict(stamp=BASE-1),dict(stamp=BASE+20_000_000),
    dict(v=(math.nan,0,0,0,0,0)),dict(v=(0,0,.1,0,0,0)),dict(v=(.6,0,0,0,0,0)),
    dict(v=(0,0,0,0,0,2)),dict(v=(0,)),])
def test_invalid_commands_are_rejected(overrides):
    a=granted();assert not command(a,**overrides)
    assert a.output(10.02,BASE+20_000_000,True,True)==Z

def test_request_cache_overflow_fences_without_renewing():
    a=Arbiter(max_cached_requests=1);a.switch(request(),10,BASE,True)
    assert not a.switch(request(expected='NAVIGATION',rid='second'),10.1,BASE+100_000_000,True).accepted
    assert a.owner=='SAFE_HOLD'


def test_finished_hold_lease_allows_new_session_after_expiry():
    a=granted()
    assert a.switch(request('SAFE_HOLD','NAVIGATION',rid='stop'),10.1,BASE+100_000_000,True).accepted
    assert not a.switch(request(session='new',rid='new'),10.2,BASE+200_000_000,True).accepted
    a.tick(12.2,BASE+2_200_000_000)
    assert a.switch(request(session='new',rid='after'),12.3,BASE+2_300_000_000,True).accepted


def test_emergency_unknown_can_acknowledge_already_fenced_state():
    a=Arbiter()
    result=a.switch(request('SAFE_HOLD','UNKNOWN',rid='emergency'),10,BASE,False,False)
    assert result.accepted and result.owner=='SAFE_HOLD'
    assert a.output(10.1,BASE+100_000_000,False,False)==Z
