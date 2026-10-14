"""Actual final-command tests in an isolated local ROS domain.

The simulated low level independently publishes measured state and odometry,
then subscribes to /backend_sim/cmd_vel. Service results alone are never used
as proof that a command was delivered or fenced.
"""
import json
import os
import time
import uuid
os.environ['ROS_DOMAIN_ID'] = '128'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import pytest
rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from go2_obstacle_interfaces.msg import MotionAuthority, MotionCommand, MotionTelemetry
from go2_obstacle_interfaces.srv import SetControlMode
from go2_mission_backends.motion_arbiter import MotionArbiter


def values(t):
    return (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)


class LowLevelHarness:
    def __init__(self, overrides=None):
        self.context = Context(); rclpy.init(context=self.context)
        self.tag = 'arbiter_test_'+uuid.uuid4().hex[:8]
        p = dict(enable_output=True,simulation_mode=True,output_topic='/backend_sim/cmd_vel',
            telemetry_source_id=self.tag,base_frame='base_footprint',state_timeout=.18,
            command_timeout=.4,max_command_age=.12,stable_duration=.05,output_frequency=50.)
        p.update(overrides or {})
        self.arbiter = MotionArbiter(context=self.context,
            parameter_overrides=[Parameter(k,value=v) for k,v in p.items()])
        self.low = Node(self.tag,context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.arbiter); self.executor.add_node(self.low)
        self.sink = []; self.authorities = []; self.status = []
        self.low.create_subscription(Twist,'/backend_sim/cmd_vel',
            lambda m:self.sink.append((time.monotonic(),values(m))),20)
        self.low.create_subscription(MotionAuthority,'/motion/authority',self.authorities.append,10)
        self.low.create_subscription(String,'/motion/status',lambda m:self.status.append(json.loads(m.data)),10)
        self.telemetry_pub = self.low.create_publisher(MotionTelemetry,'/motion/telemetry',10)
        self.odom_pub = self.low.create_publisher(Odometry,'/odom',20)
        self.nav_pub = self.low.create_publisher(MotionCommand,'/motion/nav_command',1)
        self.skill_pub = self.low.create_publisher(MotionCommand,'/motion/skill_command',1)
        self.control = self.low.create_client(SetControlMode,'/motion/set_control_mode')
        self.level = dict(known=True,nav_active=False,skill_active=False,stopped=True,
            safe_stance=True,control_releasable=True,error_code='')
        self.sequence = 0; self.telemetry_enabled = True; self.odom_enabled = True
        self.override_stamp = None
        self.timer = self.low.create_timer(.015,self.measure)
        self.spin(.12)
        self.wait(lambda:self.control.service_is_ready() and bool(self.authorities))

    def close(self):
        self.executor.remove_node(self.arbiter); self.executor.remove_node(self.low)
        self.arbiter.destroy_node(); self.low.destroy_node()
        self.executor.shutdown(); self.context.shutdown()

    def spin(self,seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            self.executor.spin_once(timeout_sec=min(.005,end-time.monotonic()))

    def wait(self,predicate,timeout=1.):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate(): return
            self.executor.spin_once(timeout_sec=.005)
        assert predicate(), self.status[-1] if self.status else 'no status'

    @property
    def authority(self):
        return self.authorities[-1]

    def measure(self):
        stamp = self.override_stamp or self.low.get_clock().now().to_msg()
        self.sequence += 1
        if self.telemetry_enabled:
            t = MotionTelemetry(); t.header.stamp = stamp; t.header.frame_id='base_footprint'
            t.source_id = self.tag; t.sequence = self.sequence
            for k,v in self.level.items(): setattr(t,k,v)
            self.telemetry_pub.publish(t)
        if self.odom_enabled:
            m = Odometry(); m.header.stamp = stamp; m.header.frame_id='nav_odom'
            m.child_frame_id='base_footprint'; m.pose.pose.orientation.w=1.
            # Independent simulated measurement, not echoed from command output.
            m.twist.twist.linear.x = 0. if self.level['stopped'] else .1
            self.odom_pub.publish(m)

    def stop(self, safe=True):
        self.level.update(nav_active=False,skill_active=False,stopped=True,
                          control_releasable=True,safe_stance=safe)
        self.spin(.09)

    def activate(self,owner,safe=True):
        self.level.update(nav_active=owner=='NAVIGATION',skill_active=owner=='SKILL',
                          stopped=False,control_releasable=False,safe_stance=safe)
        self.spin(.025)

    def switch(self,owner,expected=None,mission='test_mission',request=None,lease=2.):
        req=SetControlMode.Request(); req.owner=owner
        req.expected_owner = expected if expected is not None else self.authority.owner
        req.mission_id=mission;req.request_id=request or uuid.uuid4().hex;req.lease_duration=lease
        future=self.control.call_async(req)
        self.wait(future.done)
        result=future.result();self.spin(.025)
        return result,req

    def replay(self,req):
        future=self.control.call_async(req);self.wait(future.done);self.spin(.025)
        return future.result()

    def command(self,source,vector,owner=None,mission=None,generation=None,stamp=None):
        m=MotionCommand();m.header.stamp=stamp or self.low.get_clock().now().to_msg()
        m.header.frame_id='base_footprint';m.owner=owner or source
        m.mission_id=mission if mission is not None else self.authority.mission_id
        m.generation=generation if generation is not None else self.authority.generation
        m.twist.linear.x,m.twist.linear.y,m.twist.linear.z=vector[:3]
        m.twist.angular.x,m.twist.angular.y,m.twist.angular.z=vector[3:]
        (self.nav_pub if source=='NAVIGATION' else self.skill_pub).publish(m)
        self.spin(.025)
        return m

    def mark(self):
        self.spin(.025)
        return len(self.sink)

    def observed(self,mark):
        self.spin(.025)
        result=[v for _,v in self.sink[mark:]]
        assert result, 'simulated low level did not receive final output'
        return result


@pytest.fixture
def low_factory():
    fixtures=[]
    def make(overrides=None):
        h=LowLevelHarness(overrides);fixtures.append(h);return h
    yield make
    for h in reversed(fixtures):h.close()


ZERO=(0.,)*6
NAV=(.2,.1,0.,0.,0.,.3)
SKILL=(-.15,.12,0.,0.,0.,-.25)


def test_final_output_no_owner_and_nav_skill_mutual_exclusion(low_factory):
    h=low_factory();mark=h.mark()
    h.command('NAVIGATION',NAV,mission='test_mission')
    h.command('SKILL',SKILL,mission='test_mission')
    assert set(h.observed(mark)) == {ZERO}
    res,_=h.switch('NAVIGATION');assert res.accepted
    h.activate('NAVIGATION')
    mark=h.mark();h.command('SKILL',SKILL)
    assert set(h.observed(mark)) == {ZERO}
    mark=h.mark();h.command('NAVIGATION',NAV)
    assert NAV in h.observed(mark)
    mark=h.mark();h.command('SKILL',SKILL)
    actual=h.observed(mark)
    assert SKILL not in actual and set(actual).issubset({NAV,ZERO})
    h.stop();res,_=h.switch('SKILL');assert res.accepted
    h.activate('SKILL',safe=False)
    mark=h.mark();h.command('NAVIGATION',NAV)
    assert set(h.observed(mark)) == {ZERO}
    mark=h.mark();h.command('SKILL',SKILL)
    assert SKILL in h.observed(mark)
    assert h.authority.owner=='SKILL'  # support during execution is low-level owned


def test_switch_clears_commands_and_fences_old_generation_and_session(low_factory):
    h=low_factory();res,_=h.switch('NAVIGATION');assert res.accepted
    h.activate('NAVIGATION');old=h.command('NAVIGATION',NAV)
    assert NAV in [v for _,v in h.sink]
    old_generation=h.authority.generation
    h.stop();res,_=h.switch('SKILL');assert res.accepted
    h.activate('SKILL');mark=h.mark()
    h.nav_pub.publish(old);h.command('SKILL',SKILL,generation=old_generation)
    assert set(h.observed(mark)) == {ZERO}
    mark=h.mark();h.command('SKILL',SKILL,mission='old_mission')
    assert set(h.observed(mark)) == {ZERO}
    mark=h.mark();h.command('SKILL',SKILL)
    assert SKILL in h.observed(mark)
    h.stop();res,_=h.switch('NAVIGATION');assert res.accepted
    h.activate('NAVIGATION');mark=h.mark()
    h.command('NAVIGATION',NAV,generation=old_generation)
    h.command('SKILL',SKILL)
    assert set(h.observed(mark)) == {ZERO}


def test_lease_expiry_stops_actual_output_and_replay_does_not_confirm_old_owner(low_factory):
    h=low_factory(dict(command_timeout=.5))
    res,req=h.switch('NAVIGATION',lease=.18);assert res.accepted
    h.activate('NAVIGATION');h.command('NAVIGATION',NAV)
    h.wait(lambda:h.authority.owner=='SAFE_HOLD')
    mark=h.mark();h.command('NAVIGATION',NAV,mission='test_mission',generation=2)
    assert set(h.observed(mark)) == {ZERO}
    duplicate=h.replay(req)
    assert not duplicate.accepted and duplicate.owner=='SAFE_HOLD'
    assert set(h.observed(mark)) == {ZERO}


def test_command_heartbeat_expiry_cannot_resume_from_cached_velocity(low_factory):
    h=low_factory(dict(command_timeout=.14,max_command_age=.4));res,_=h.switch('NAVIGATION');assert res.accepted
    h.activate('NAVIGATION');cmd=h.command('NAVIGATION',NAV)
    h.wait(lambda:h.authority.owner=='SAFE_HOLD')
    assert h.authority.reason=='COMMAND_HEARTBEAT_TIMEOUT'
    mark=h.mark();h.nav_pub.publish(cmd)
    assert set(h.observed(mark)) == {ZERO}


@pytest.mark.parametrize('fault', ['unknown','telemetry_dropout','odom_dropout','conflict'])
def test_low_level_fault_or_dropout_fences_actual_output(low_factory,fault):
    h=low_factory(dict(command_timeout=.8))
    res,_=h.switch('NAVIGATION');assert res.accepted
    h.activate('NAVIGATION');h.command('NAVIGATION',NAV)
    if fault=='unknown':h.level['known']=False
    elif fault=='telemetry_dropout':h.telemetry_enabled=False
    elif fault=='odom_dropout':h.odom_enabled=False
    else:h.level.update(nav_active=True,skill_active=True)
    h.wait(lambda:h.authority.owner=='SAFE_HOLD')
    mark=h.mark();h.command('NAVIGATION',NAV,mission='test_mission')
    assert set(h.observed(mark)) == {ZERO}


def test_safe_stance_required_for_switch_but_not_during_authorized_skill(low_factory):
    h=low_factory();h.stop(safe=False)
    res,_=h.switch('SKILL');assert not res.accepted
    mark=h.mark();h.command('SKILL',SKILL,mission='test_mission')
    assert set(h.observed(mark)) == {ZERO}
    h.stop(safe=True);res,_=h.switch('SKILL');assert res.accepted
    h.activate('SKILL',safe=False);mark=h.mark()
    h.command('SKILL',SKILL)
    assert SKILL in h.observed(mark) and h.authority.owner=='SKILL'
    # An incomplete action termination cannot authorize navigation.
    res,_=h.switch('NAVIGATION');assert not res.accepted
    mark=h.mark();h.command('NAVIGATION',NAV)
    assert NAV not in h.observed(mark)


def test_wrong_release_duplicate_rpc_and_request_conflict(low_factory):
    h=low_factory(dict(command_timeout=.8))
    res,req=h.switch('NAVIGATION',request='grant_nav');assert res.accepted
    initial_generation=h.authority.generation
    before=h.authority.lease_remaining
    duplicate=h.replay(req)
    assert duplicate.accepted and duplicate.owner=='NAVIGATION'
    assert h.authority.generation==initial_generation and h.authority.lease_remaining<before
    h.activate('NAVIGATION');h.command('NAVIGATION',NAV)
    wrong,_=h.switch('SAFE_HOLD',mission='other_mission');assert not wrong.accepted
    mark=h.mark();h.command('NAVIGATION',NAV)
    assert NAV in h.observed(mark)
    conflict,_=h.switch('SKILL',request='grant_nav');assert not conflict.accepted
    h.wait(lambda:h.authority.owner=='SAFE_HOLD')
    mark=h.mark();h.command('NAVIGATION',NAV,mission='test_mission')
    assert set(h.observed(mark)) == {ZERO}


def test_command_identical_stamp_rejected_without_replacing_valid_output(low_factory):
    h=low_factory(dict(command_timeout=.5));res,_=h.switch('NAVIGATION');assert res.accepted
    h.activate('NAVIGATION');cmd=h.command('NAVIGATION',NAV)
    mark=h.mark();h.command('NAVIGATION',SKILL,stamp=cmd.header.stamp)
    actual=h.observed(mark)
    assert NAV in actual and SKILL not in actual
    assert h.authority.reason=='OLD_COMMAND_TIMESTAMP'


def test_repeated_measured_stamp_with_increasing_sequence_is_not_fresh_evidence(low_factory):
    h=low_factory(dict(command_timeout=.8));res,_=h.switch('NAVIGATION');assert res.accepted
    h.activate('NAVIGATION');h.command('NAVIGATION',NAV)
    h.odom_enabled=False  # independently force only telemetry stamp replay now
    h.override_stamp=h.low.get_clock().now().to_msg()
    h.spin(.06)
    # Since odometry is still within TTL, only telemetry repetition can trigger
    # this immediate fence; do not wait for the ordinary odom timeout.
    assert h.authority.owner=='SAFE_HOLD'
    assert 'TELEMETRY' in h.authority.reason
    mark=h.mark();h.command('NAVIGATION',NAV,mission='test_mission')
    assert set(h.observed(mark)) == {ZERO}


def test_unknown_motion_output_disabled_never_forwards(low_factory):
    h=low_factory(dict(enable_output=False))
    res,_=h.switch('NAVIGATION');assert not res.accepted and res.message=='OUTPUT_DISABLED'
    mark=h.mark();h.command('NAVIGATION',NAV,mission='test_mission')
    assert set(h.observed(mark)) == {ZERO}


@pytest.mark.parametrize('invalid', ['wrong_frame','wrong_owner','nonfinite','nonplanar','speed_limit','future','bad_stamp_fields'])
def test_invalid_command_never_reaches_final_output(low_factory,invalid):
    h=low_factory();res,_=h.switch('NAVIGATION');assert res.accepted
    h.activate('NAVIGATION')
    m=MotionCommand();m.header.stamp=h.low.get_clock().now().to_msg()
    m.header.frame_id='base_footprint';m.owner='NAVIGATION'
    m.mission_id=h.authority.mission_id;m.generation=h.authority.generation
    m.twist.linear.x=.2
    if invalid=='wrong_frame':m.header.frame_id='wrong_body'
    elif invalid=='wrong_owner':m.owner='SKILL'
    elif invalid=='nonfinite':m.twist.linear.x=float('nan')
    elif invalid=='nonplanar':m.twist.linear.z=.1
    elif invalid=='speed_limit':m.twist.linear.x=1.
    elif invalid=='future':m.header.stamp.sec+=1
    else:
        # Preserve the combined integer time but violate ROS builtin Time
        # field constraints. A mere sec*1e9+nanosec conversion misses this.
        m.header.stamp.sec-=1;m.header.stamp.nanosec+=1000000000
    mark=h.mark();h.nav_pub.publish(m)
    assert set(h.observed(mark)) == {ZERO}


def test_same_owner_renewal_preserves_generation_during_active_unsafe_stance(low_factory):
    h=low_factory();res,_=h.switch('SKILL');assert res.accepted
    h.activate('SKILL',safe=False)
    generation=h.authority.generation
    h.command('SKILL',SKILL)
    res,_=h.switch('SKILL');assert res.accepted
    assert h.authority.owner=='SKILL' and h.authority.generation==generation
    mark=h.mark();h.command('SKILL',SKILL)
    assert SKILL in h.observed(mark)


def test_hold_lease_expiry_allows_new_session_but_never_old_commands(low_factory):
    h=low_factory();res,_=h.switch('NAVIGATION');assert res.accepted
    h.activate('NAVIGATION');old=h.command('NAVIGATION',NAV)
    h.stop();res,_=h.switch('SAFE_HOLD',lease=.12);assert res.accepted
    assert h.authority.owner=='SAFE_HOLD'
    rejected,_=h.switch('NAVIGATION',mission='next_mission');assert not rejected.accepted
    h.wait(lambda:h.authority.owner=='SAFE_HOLD' and not h.authority.mission_id)
    res,_=h.switch('NAVIGATION',mission='next_mission');assert res.accepted
    h.activate('NAVIGATION');mark=h.mark();h.nav_pub.publish(old)
    assert set(h.observed(mark)) == {ZERO}
    mark=h.mark();h.command('NAVIGATION',NAV)
    assert NAV in h.observed(mark)
