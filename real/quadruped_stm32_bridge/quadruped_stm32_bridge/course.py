"""Independent obstacle instances and compilation into key-backend plans."""
import copy
import math
import os
from pathlib import Path
import tempfile
import yaml
from .cybergear_core import validate_plan, number


def write_yaml(path,data):
    path=Path(path).expanduser();path.parent.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(dir=path.parent,prefix='.'+path.name)
    try:
        with os.fdopen(fd,'w') as f:yaml.safe_dump(data,f,allow_unicode=True,sort_keys=False)
        os.replace(name,path)
    finally:
        if os.path.exists(name):os.unlink(name)


def new_instance(template_id,template,identifier):
    return dict(id=identifier,template=template_id,name=template['name']+' '+identifier,
                enabled=True,samples={},transit=[],x_scale=1.0,y_scale=1.0,jump_wait_scale=1.0,steps=copy.deepcopy(template['steps']))


def pose3(value):
    if isinstance(value,dict):value=[value[k] for k in ('x','y','yaw')]
    if not isinstance(value,(list,tuple)) or len(value)!=3:raise ValueError('坐标需要 x y yaw')
    return [number(v,'pose') for v in value]


def compile_course(course,selected=None):
    if course.get('frame') not in ('map','nav_odom'):raise ValueError('不支持的场地坐标系')
    instances=course.get('obstacles',[])
    ids=[i['id'] for i in instances]
    if len(ids)!=len(set(ids)):raise ValueError('障碍 ID 重复')
    if selected is not None and selected not in ids:raise ValueError('未找到所选障碍')
    plan=[]
    for item in instances:
        if selected is not None:
            if item['id']!=selected:continue
        elif not item.get('enabled',True):continue
        name=item['name'];samples=item['samples']
        missing=[k for k in ('origin','entry','exit') if k not in samples]
        if missing:raise ValueError(name+' 尚未采集：'+','.join(missing))
        origin=pose3(samples['origin'])
        sx=number(item.get('x_scale',1.0),'x_scale',True)
        sy=number(item.get('y_scale',1.0),'y_scale',True)
        jw=number(item.get('jump_wait_scale',1.0),'jump_wait_scale',True)
        def add(step,label):plan.append(dict(step,obstacle=name,label=label))
        for index,point in enumerate(item.get('transit',[])):
            x,y,yaw=pose3(point);add(dict(type='waypoint',x=x,y=y,yaw=yaw,mode_key=9),'过渡点 '+str(index+1))
        x,y,yaw=pose3(samples['entry'])
        add(dict(type='waypoint',x=x,y=y,yaw=yaw,mode_key=9),'到达入口')
        add(dict(type='key',key=5,duration_s=0.5),'入口停止')
        for index,raw in enumerate(item['steps']):
            step=copy.deepcopy(raw);kind=step['type']
            if kind=='local_waypoint':
                target=step['target']
                if not isinstance(target,list) or len(target)!=3:raise ValueError('模板 target 需要三个值')
                step['target']=[None if target[0] is None else number(target[0],'target x')*sx,None if target[1] is None else number(target[1],'target y')*sy,None if target[2] is None else math.radians(number(target[2],'yaw'))]
                step['origin']=origin
                if 'heading_deg' in step:step['heading']=math.radians(number(step.pop('heading_deg'),'heading_deg',True))
            if kind=='key' and step.get('key') in (6,7,8):step['duration_s']=number(step['duration_s'],'duration',True)*jw
            add(step,'模板步骤 '+str(index+1)+' / '+kind)
        x,y,yaw=pose3(samples['exit'])
        add(dict(type='key',key=5,duration_s=0.5),'障碍动作收尾')
        # Exit approach uses the last mode from the template, not a guessed normal gait.
        last_mode=9
        for raw in item['steps']:
            if raw['type']=='mode':last_mode=raw['key']
            elif 'mode_key' in raw:last_mode=raw['mode_key']
        add(dict(type='waypoint',x=x,y=y,yaw=yaw,mode_key=last_mode),'到达出口')
        add(dict(type='key',key=5,duration_s=0.5),'出口停止')
    if not plan:raise ValueError('请添加并启用至少一个障碍')
    result=dict(version=1,frame=course['frame'],course_session=course.get('session'),steps=plan)
    validate_plan(result)
    return result
