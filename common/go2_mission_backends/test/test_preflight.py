from types import SimpleNamespace
import pytest
from builtin_interfaces.msg import Time
from go2_mission_backends.preflight import recent_stamp, route_requirements, valid_stamp


def route(skill='step',same=False):
    t=SimpleNamespace(type='obstacle',id='one',skill_id=skill,
        approach_pose=(0.,0.,0.),end_pose=((0.,0.,0.) if same else (1.,0.,0.)))
    return SimpleNamespace(tasks=[t])


def test_registered_route_and_distinct_endpoints():
    assert route_requirements(route(),['step'],True)==[]
    assert route_requirements(route(),['other'],True)==['SKILL_NOT_REGISTERED:step']
    assert route_requirements(route(same=True),['step'],True)==['ENTRANCE_EXIT_NOT_DISTINCT:one']
    assert route_requirements(route(same=True),['step'],False)==[]


@pytest.mark.parametrize('stamp,now,expected', [
    (1000000000,1100000000,True),(1000000000,2000000000,False),
    (0,1100000000,False),(1000000000,0,False),
    (1300000000,1100000000,False),(1120000000,1100000000,True)])
def test_freshness(stamp,now,expected):
    assert recent_stamp(stamp,now,.5,.05)==expected


def test_invalid_time_fields():
    assert valid_stamp(Time(sec=1,nanosec=1500000000))==0
    assert valid_stamp(Time(sec=-1,nanosec=0))==0
    assert valid_stamp(Time(sec=1,nanosec=2))==1000000002
