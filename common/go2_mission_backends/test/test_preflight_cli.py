"""Regress actual Humble YAML/CLI array parsing via installed console entrypoint."""
import json
import os
import signal
import subprocess
import time
from pathlib import Path
import pytest
import yaml
import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

PACKAGE=Path(__file__).resolve().parents[1]
BASE=PACKAGE/'config/preflight.yaml'
ROUTE=PACKAGE/'config/backend_test_route.yaml'


def environment():
    env=os.environ.copy()
    env.update(ROS_DOMAIN_ID=os.environ.get('RC2026_CLI_DOMAIN_ID', '153'),ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
    return env


def command(params):
    return ['ros2','run','go2_mission_backends','preflight','--ros-args',
        '--params-file',str(params),'-p','route_file:='+str(ROUTE),
        '-p','service_name:=/preflight_cli_check','-p','use_sim_time:=false']


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3.)


@pytest.fixture(scope='module')
def observer():
    old={k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION')}
    os.environ.update({k:v for k,v in environment().items() if k in old})
    context=Context();rclpy.init(context=context)
    node=Node('preflight_cli_observer',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    yield node,executor
    executor.shutdown();node.destroy_node();context.shutdown()
    for k,v in old.items():
        if v is None:os.environ.pop(k,None)
        else:os.environ[k]=v


def wait(executor,predicate,process,timeout=6.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        assert process.poll() is None,'real ros2 run preflight exited unexpectedly'
        if predicate():return
        executor.spin_once(timeout_sec=.01)
    assert predicate(),'console-node condition did not become true'


def query(node,executor,client,process):
    future=client.call_async(Trigger.Request());wait(executor,future.done,process)
    response=future.result()
    return json.loads(response.message)


def test_delivered_empty_array_params_file_starts_real_console_node(observer,tmp_path):
    node,executor=observer
    with (tmp_path/'empty.log').open('w+') as log:
        process=subprocess.Popen(command(BASE),env=environment(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        client=node.create_client(Trigger,'/preflight_cli_check')
        try:
            wait(executor,client.service_is_ready,process)
            result=query(node,executor,client,process)
            assert result['checks']['clock'] and result['checks']['route']
            assert not any(k.startswith('nav_node:') for k in result['checks'])
            # No other backends are running; alive does not mean preflight ready.
            assert not result['ready']
        except Exception:
            log.seek(0);print(log.read());raise
        finally:stop(process);node.destroy_client(client)


def test_nonempty_cli_array_retains_actual_lifecycle_assertions(observer,tmp_path):
    node,executor=observer
    data=yaml.safe_load(BASE.read_text())
    data['mission_preflight']['ros__parameters']['required_nav_nodes']=['/cli_controller','/cli_planner']
    params=tmp_path/'nonempty.yaml';params.write_text(yaml.safe_dump(data))
    counts={'controller':0,'planner':0}
    def controller(_req,res):counts['controller']+=1;res.current_state.id=3;return res
    def planner(_req,res):counts['planner']+=1;res.current_state.id=2;return res
    services=[node.create_service(GetState,'/cli_controller/get_state',controller),
              node.create_service(GetState,'/cli_planner/get_state',planner)]
    with (tmp_path/'nonempty.log').open('w+') as log:
        process=subprocess.Popen(command(params),env=environment(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        client=node.create_client(Trigger,'/preflight_cli_check')
        try:
            wait(executor,lambda:client.service_is_ready() and all(counts.values()),process)
            deadline=time.monotonic()+3.
            while True:
                result=query(node,executor,client,process)
                if result['checks']['nav_node:/cli_controller']:break
                assert time.monotonic()<deadline
            assert not result['checks']['nav_node:/cli_planner']
            assert 'NAVIGATION_NODE_NOT_ACTIVE:/cli_planner' in result['reasons']
        except Exception:
            log.seek(0);print(log.read());raise
        finally:
            stop(process);node.destroy_client(client)
            for service in services:node.destroy_service(service)


@pytest.mark.parametrize('invalid',['/controller_server',42,True,['/controller',42]])
def test_cli_scalar_and_mixed_array_are_not_silently_empty(invalid,tmp_path):
    data=yaml.safe_load(BASE.read_text());data['mission_preflight']['ros__parameters']['required_nav_nodes']=invalid
    params=tmp_path/'invalid.yaml';params.write_text(yaml.safe_dump(data))
    process=subprocess.Popen(command(params),env=environment(),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
        text=True,start_new_session=True)
    try:
        output,_=process.communicate(timeout=6.)
        assert process.returncode!=0,output
        assert 'required_nav_nodes' in output or 'Sequence should be of same type' in output,output
    finally:stop(process)
