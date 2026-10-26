import copy
import math
from pathlib import Path
import pytest
import yaml
from quadruped_stm32_bridge.course import compile_course, new_instance, write_yaml
from quadruped_stm32_bridge.cybergear_core import validate_plan, Runner

TEMPLATES=yaml.safe_load((Path(__file__).parents[1]/'config/cybergear_obstacles.yaml').read_text())['templates']


def item(key,identifier='one'):
    i=new_instance(key,TEMPLATES[key],identifier)
    i['samples']={'origin':[2,3,math.pi/2],'entry':[1,2,0],'exit':[4,5,0]}
    return i


@pytest.mark.parametrize('key',list(TEMPLATES))
def test_all_templates_compile_without_other_obstacles(key):
    plan=compile_course(dict(frame='map',obstacles=[item(key)]))
    steps=validate_plan(plan)
    assert steps[0]['label']=='到达入口'
    assert steps[-1]['key']==5
    assert all('obstacle' in s for s in steps)
    assert not any('anchor_skill' in s for s in steps)


def test_instances_order_selection_and_skip():
    a=item('step','a');b=item('step','b');b['enabled']=False
    course=dict(frame='map',obstacles=[b,a])
    p=compile_course(course)
    assert {s['obstacle'] for s in p['steps']}=={a['name']}
    p=compile_course(course,'b')
    assert {s['obstacle'] for s in p['steps']}=={b['name']}
    b['steps'][0]['key']=18
    assert a['steps'][0]['key']==9


def test_missing_capture_rejected_and_transit_used():
    i=item('bridge_b');i['samples'].pop('exit')
    with pytest.raises(ValueError):compile_course(dict(frame='map',obstacles=[i]))
    i['samples']['exit']=[1,2,3];i['transit']=[[8,9,0]]
    p=compile_course(dict(frame='map',obstacles=[i]))
    assert p['steps'][0]['x']==8


def test_local_dynamic_coordinates_and_reverse():
    s=dict(type='local_waypoint',origin=[2,3,math.pi/2],target=[None,1,0],mode_key=9,reverse=True)
    runner=Runner(validate_plan({'steps':[s]}));runner.start()
    assert runner.tick(0,{'mode':'a'},0,(2,5,0))==[9]
    # Local x locks to 2; local y=1 -> world (1,5). Reverse heading points east.
    assert runner.tick(1,{'mode':'a'},1,(2,5,0))==[4]
    assert runner.resolved['x']==pytest.approx(1)
    assert runner.resolved['y']==pytest.approx(5)
    assert runner.tick(2,{'mode':'a'},2,(1,5,math.pi/2))==[5]


def test_plan_persistence(tmp_path):
    data=compile_course(dict(frame='nav_odom',session='test',obstacles=[item('step')]))
    p=tmp_path/'nested/course.yaml';write_yaml(p,data)
    assert yaml.safe_load(p.read_text())==data
    assert data['course_session']=='test'


def test_local_timeout_and_scaling():
    i=item('step');i.update(x_scale=2,y_scale=3,jump_wait_scale=1.5)
    plan=compile_course(dict(frame='map',obstacles=[i]))
    local=next(s for s in plan['steps'] if s['type']=='local_waypoint')
    assert local['target'][0]==pytest.approx(-0.30)
    jump=next(s for s in plan['steps'] if s['type']=='key' and s['key']==6)
    assert jump['duration_s']==7.5
    local['timeout_s']=1
    r=Runner(validate_plan({'steps':[local]}));r.start()
    r.tick(0,{'mode':'a'},0,(0,0,0))
    assert r.tick(2,{'mode':'a'},2,(0,0,0))==[5]
    assert r.state=='step_timeout'
