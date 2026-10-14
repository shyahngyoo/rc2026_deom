"""Real nonblocking preflight RPC tests with independent simulated providers."""
import json
import os
import time
import uuid
os.environ['ROS_DOMAIN_ID']='128'
os.environ['ROS_LOCALHOST_ONLY']='1'
import pytest
rclpy=pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from go2_obstacle_interfaces.msg import MotionAuthority
from go2_obstacle_interfaces.srv import GetMotionState
from go2_mission_backends.preflight import MissionPreflight


ROUTE='start_area: test\nframe_id: map\nroute:\n  - id: obstacle_one\n    type: obstacle\n    transit_pose: [0.0, 0.0, 0.0]\n    approach_pose: [0.1, 0.0, 0.0]\n    skill_id: step\n    end_pose: [1.0, 0.0, 0.0]\nreturn:\n  pose: [0.0, 0.0, 0.0]\n'


class Fixture:
    def __init__(self,route_file,wait_ready=True,require_lifecycle=False,context=None):
        self.owns_context=context is None
        self.context=context or Context()
        if self.owns_context:rclpy.init(context=self.context)
        self.tag='preflight_'+uuid.uuid4().hex[:8]
        self.path='/'+self.tag
        self.base=self.tag+'_base';self.odom_frame=self.tag+'_odom'
        params=dict(route_file=str(route_file),service_name=self.path+'/check',
            localization_ok_service=self.path+'/localization',robot_state_topic=self.path+'/robot_state',
            odom_topic=self.path+'/odom',authority_topic=self.path+'/authority',
            motion_state_service=self.path+'/motion',nav_action_name=self.path+'/navigate',
            skills_ready_service=self.path+'/skills',base_frame=self.base,odom_frame=self.odom_frame,
            require_distinct_endpoints=True,topic_max_age=.18,adapter_status_max_age=.18,
            tf_max_age=.18,rpc_timeout=.08,rpc_max_age=.18,poll_period=.025)
        if require_lifecycle:params['required_nav_nodes']=[self.path+'/controller']
        self.node=MissionPreflight(context=self.context,
            parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
        self.mock=Node('motion_arbiter',context=self.context)
        self.observer=Node(self.tag,context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context)
        for n in (self.node,self.mock,self.observer):self.executor.add_node(n)
        self.flags=dict(adapter=True,odom=True,authority=True,tf=True,health=True,motion=True,skills=True)
        self.lifecycle_state=3
        if require_lifecycle:self.mock.create_service(GetState,self.path+'/controller/get_state',self.lifecycle)
        self.registered=['step'];self.authority_owner='SAFE_HOLD';self.authority_session=''
        self.malformed_adapter=False
        self.health_service=self.mock.create_service(Trigger,self.path+'/localization',self.health)
        self.skill_service=self.mock.create_service(Trigger,self.path+'/skills',self.skills)
        self.motion_service=self.mock.create_service(GetMotionState,self.path+'/motion',self.motion)
        self.nav=ActionServer(self.mock,NavigateToPose,self.path+'/navigate',execute_callback=self.navigate)
        self.adapter_pub=self.mock.create_publisher(String,self.path+'/robot_state',10)
        self.odom_pub=self.mock.create_publisher(Odometry,self.path+'/odom',10)
        self.authority_pub=self.mock.create_publisher(MotionAuthority,self.path+'/authority',10)
        self.output_pub=self.mock.create_publisher(Twist,'/backend_sim/cmd_vel',1)
        self.tf=TransformBroadcaster(self.mock)
        self.timer=self.mock.create_timer(.025,self.publish)
        self.client=self.observer.create_client(Trigger,self.path+'/check')
        self.rogue=None;self.manager=None
        self.spin(.15)
        if wait_ready:self.wait(lambda:self.node.evaluate()['ready'])

    def lifecycle(self,_req,res):res.current_state.id=self.lifecycle_state;return res
    def health(self,_req,res):res.success=self.flags['health'];res.message='evidence';return res
    def skills(self,_req,res):
        res.success=self.flags['skills'];res.message=json.dumps(dict(skills=self.registered));return res
    def motion(self,_req,res):
        res.known=self.flags['motion'];res.safe_stance=self.flags['motion']
        res.control_releasable=self.flags['motion'];res.skill_active=False;return res
    async def navigate(self,goal):goal.succeed();return NavigateToPose.Result()

    def publish(self):
        stamp=self.mock.get_clock().now().to_msg()
        ns=stamp.sec*1000000000+stamp.nanosec
        if self.flags['adapter']:
            self.adapter_pub.publish(String(data='bad json' if self.malformed_adapter else json.dumps(dict(
                global_ready=True,local_odom_valid=True,input_stamp_ns=ns))))
        if self.flags['odom']:
            m=Odometry();m.header.stamp=stamp;m.header.frame_id=self.odom_frame
            m.child_frame_id=self.base;m.pose.pose.orientation.w=1.;self.odom_pub.publish(m)
        if self.flags['authority']:
            a=MotionAuthority();a.header.stamp=stamp;a.owner=self.authority_owner
            a.mission_id=self.authority_session;a.generation=1;a.healthy=True;self.authority_pub.publish(a)
        if self.flags['tf']:
            tf=[]
            for parent,child in [('map',self.odom_frame),(self.odom_frame,self.base)]:
                t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
                t.transform.rotation.w=1.;tf.append(t)
            self.tf.sendTransform(tf)

    def spin(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:self.executor.spin_once(timeout_sec=min(.005,end-time.monotonic()))
    def wait(self,predicate,timeout=1.5):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            self.executor.spin_once(timeout_sec=.005)
        assert predicate(),self.node.evaluate()
    def check(self):
        future=self.client.call_async(Trigger.Request());self.wait(future.done)
        res=future.result();return res.success,json.loads(res.message)
    def close(self):
        self.timer.cancel();self.node.timer.cancel()
        for runner in self.node.runners.values():runner.cancel()
        if self.manager is not None:
            self.manager.closing=True
            if self.manager.startup_runner is not None:self.manager.startup_runner.cancel()
            self.manager.timer.cancel();self.manager.status_timer.cancel()
        # Release the executor wait set before destroying Action/Service handles.
        self.executor.shutdown()
        if self.manager is not None:self.manager.destroy_node()
        if self.nav is not None:self.nav.destroy()
        if self.rogue is not None:self.rogue.destroy_node()
        for n in (self.node,self.mock,self.observer):n.destroy_node()
        if self.owns_context:self.context.shutdown()


@pytest.fixture(scope='module')
def ros_context():
    context=Context();rclpy.init(context=context)
    yield context
    context.shutdown()


@pytest.fixture
def fixture(tmp_path,ros_context):
    route=tmp_path/'route.yaml';route.write_text(ROUTE)
    f=Fixture(route,context=ros_context)
    yield f
    f.close()


def test_all_ready_has_structured_positive_response(fixture):
    ok,result=fixture.check()
    assert ok and result['ready'] and not result['reasons']
    assert all(result['checks'].values())
    assert result['required_skills']==['step']
    assert result['output_publishers']==[dict(name='motion_arbiter',namespace='/')]


@pytest.mark.parametrize('missing', ['adapter','odom','authority','tf','health','motion','skills'])
def test_any_missing_evidence_refuses_preflight(fixture,missing):
    fixture.flags[missing]=False
    fixture.wait(lambda:not fixture.node.evaluate()['ready'])
    ok,result=fixture.check()
    assert not ok and result['reasons']
    assert not result['checks'][dict(adapter='robot_state',odom='odometry',authority='authority',tf='tf',
        health='localization',motion='motion_state',skills='skills_backend')[missing]]


def test_missing_skill_registration_refuses_preflight(fixture):
    fixture.registered=['different_skill']
    fixture.wait(lambda:'SKILL_NOT_REGISTERED:step' in fixture.node.evaluate()['reasons'])
    ok,result=fixture.check();assert not ok and 'SKILL_NOT_REGISTERED:step' in result['reasons']


def test_actual_graph_multi_publisher_and_missing_publisher_refuse_preflight(fixture):
    fixture.rogue=Node('rogue_velocity',context=fixture.context)
    fixture.executor.add_node(fixture.rogue)
    fixture.rogue.create_publisher(Twist,'/backend_sim/cmd_vel',1)
    fixture.wait(lambda:len(fixture.node.evaluate()['output_publishers'])==2)
    ok,result=fixture.check();assert not ok
    assert 'FINAL_COMMAND_PUBLISHER_NOT_UNIQUE_ARBITER' in result['reasons']
    fixture.executor.remove_node(fixture.rogue);fixture.rogue.destroy_node();fixture.rogue=None
    fixture.mock.destroy_publisher(fixture.output_pub)
    fixture.wait(lambda:len(fixture.node.evaluate()['output_publishers'])==0)
    ok,_=fixture.check();assert not ok


def test_initial_authority_not_safe_or_previous_session_blocks_start(fixture):
    fixture.authority_owner='NAVIGATION'
    fixture.wait(lambda:not fixture.node.evaluate()['checks']['authority'])
    assert not fixture.check()[0]
    fixture.authority_owner='SAFE_HOLD';fixture.authority_session='previous_mission'
    fixture.spin(.05)
    assert not fixture.check()[0]


def test_malformed_status_and_action_server_exit_refuse_preflight(fixture):
    fixture.malformed_adapter=True
    fixture.wait(lambda:not fixture.node.evaluate()['checks']['robot_state'])
    assert not fixture.check()[0]
    fixture.malformed_adapter=False;fixture.nav.destroy();fixture.nav=None
    fixture.wait(lambda:not fixture.node.evaluate()['checks']['nav_action'])
    assert not fixture.check()[0]


def test_lost_rpc_response_times_out_without_blocking_executor(fixture):
    fixture.mock.destroy_service(fixture.health_service)
    fixture.wait(lambda:not fixture.node.evaluate()['checks']['localization'])
    # Other provider callbacks still advance, proving RPC waiting is nonblocking.
    before=fixture.node.adapter['input_stamp_ns'];fixture.spin(.08)
    assert fixture.node.adapter['input_stamp_ns']>before
    ok,result=fixture.check();assert not ok
    assert 'LOCALIZATION_EVIDENCE_UNAVAILABLE' in result['reasons']


def test_identical_entrance_exit_in_required_profile_is_rejected(tmp_path,ros_context):
    path=tmp_path/'same.yaml';path.write_text(ROUTE.replace('end_pose: [1.0, 0.0, 0.0]','end_pose: [0.1, 0.0, 0.0]'))
    f=Fixture(path,wait_ready=False,context=ros_context)
    try:
        f.wait(lambda:'ENTRANCE_EXIT_NOT_DISTINCT:obstacle_one' in f.node.evaluate()['reasons'])
        ok,result=f.check();assert not ok
        assert 'ENTRANCE_EXIT_NOT_DISTINCT:obstacle_one' in result['reasons']
    finally:f.close()



def add_manager(f,**overrides):
    from robocon_mission.mission_manager import MissionManager
    p=dict(route_file=f.node.params['route_file'],startup_check_service=f.path+'/check',
        startup_check_max_age=.12,startup_check_timeout=.08,startup_timeout=.3,
        localization_ok_service=f.path+'/localization',base_frame=f.base,
        nav_action_name=f.path+'/navigate',autostart=False,simulation_mode=True)
    p.update(overrides)
    f.manager=MissionManager(context=f.context,
        parameter_overrides=[Parameter(k,value=v) for k,v in p.items()])
    f.executor.add_node(f.manager)
    f.start_client=f.observer.create_client(Trigger,'/mission/start')
    return f.manager


def request_start(f):
    future=f.start_client.call_async(Trigger.Request());f.wait(future.done)
    return future.result()


def test_manual_service_and_direct_start_cannot_bypass_missing_precheck(fixture):
    fixture.flags['health']=False
    fixture.wait(lambda:not fixture.node.evaluate()['ready'])
    manager=add_manager(fixture)
    assert manager._start() is False and not manager.started
    fixture.spin(.12)
    result=request_start(fixture)
    assert not result.success and not manager.started
    assert manager.fsm.state.value=='IDLE'
    fixture.flags['health']=True
    fixture.wait(manager._startup_ready)
    result=request_start(fixture)
    assert result.success and manager.started
    assert not request_start(fixture).success


def test_stale_positive_precheck_does_not_start(fixture):
    manager=add_manager(fixture)
    fixture.wait(manager._startup_ready)
    fixture.executor.remove_node(fixture.node)  # queued RPC can no longer answer
    fixture.spin(.22)
    assert not manager._startup_ready() and not manager._start()
    assert not request_start(fixture).success and not manager.started
    fixture.executor.add_node(fixture.node)


def test_autostart_waits_for_preflight_and_does_not_reapply_gate_during_motion(fixture):
    fixture.flags['health']=False;fixture.wait(lambda:not fixture.node.evaluate()['ready'])
    manager=add_manager(fixture,autostart=True,startup_timeout=1.)
    fixture.spin(.12);assert not manager.started
    fixture.flags['health']=True;fixture.wait(lambda:manager.started)
    # Authority changing for motion makes startup evidence fail by design.
    # Startup gate must not become a separate runtime abort source.
    fixture.authority_owner='NAVIGATION'
    fixture.wait(lambda:not fixture.node.evaluate()['ready'])
    fixture.spin(.15)
    assert manager.started and manager.fsm.state.value!='ABORTED'


def test_bounded_autostart_failure_remains_idle_and_requires_manual_retry(fixture):
    fixture.flags['health']=False;fixture.wait(lambda:not fixture.node.evaluate()['ready'])
    manager=add_manager(fixture,autostart=True,startup_timeout=.18)
    fixture.wait(lambda:manager.startup_timed_out)
    assert not manager.autostart and not manager.started and manager.fsm.state.value=='IDLE'
    fixture.flags['health']=True;fixture.wait(manager._startup_ready)
    fixture.spin(.12);assert not manager.started
    assert request_start(fixture).success



def test_clock_rollback_before_timer_cannot_reuse_positive_startup_result(fixture,monkeypatch):
    from types import SimpleNamespace
    from rclpy.time import Time
    manager=add_manager(fixture);fixture.wait(manager._startup_ready)
    # Inject a tiny rollback between the periodic watcher and manual request.
    # Old measurements still lie inside future tolerance, so the epoch guard
    # itself must reject; waiting for timeout cannot prove this property.
    past=manager.startup_last_ros-1000000
    monkeypatch.setattr(manager,'get_clock',lambda:SimpleNamespace(now=lambda:Time(nanoseconds=past)))
    assert not manager._startup_ready() and not manager._start() and not manager.started


def test_preflight_immediate_small_clock_rollback_is_not_hidden_by_future_tolerance(fixture,monkeypatch):
    from types import SimpleNamespace
    from rclpy.time import Time
    past=fixture.node.last_ros-1000000
    monkeypatch.setattr(fixture.node,'get_clock',lambda:SimpleNamespace(now=lambda:Time(nanoseconds=past)))
    result=fixture.node.evaluate()
    assert not result['ready'] and 'CLOCK_ROLLBACK' in result['reasons']



def test_configured_navigation_lifecycle_must_be_active(tmp_path,ros_context):
    path=tmp_path/'route.yaml';path.write_text(ROUTE)
    f=Fixture(path,require_lifecycle=True,context=ros_context)
    try:
        assert f.check()[0]
        f.lifecycle_state=2
        f.wait(lambda:'NAVIGATION_NODE_NOT_ACTIVE:'+f.path+'/controller' in f.node.evaluate()['reasons'])
        assert not f.check()[0]
    finally:f.close()
