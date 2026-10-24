"""Software-only real ROS topic tests. No serial port or hardware is opened."""
import copy
import json
import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from go2_obstacle_interfaces.msg import MotionAuthority, MotionCommand, MotionTelemetry
from std_msgs.msg import String
from quadruped_stm32_bridge.nav_velocity_adapter import NavVelocityAdapter, velocity_reason
from quadruped_stm32_bridge.telemetry_adapter import TelemetryAdapter, measured_status


@pytest.fixture
def ros():
    context = Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    nodes = []
    def add(cls, *args, **kwargs):
        n = cls(*args, context=context, **kwargs)
        nodes.append(n)
        executor.add_node(n)
        return n
    def spin(duration=.12):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
    yield add, spin
    executor.shutdown()
    for node in reversed(nodes):
        node.destroy_node()
    context.shutdown()


@pytest.mark.parametrize('values,reason', [
    ((.1,0.,0.,0.,0.,.2), ''),
    ((.1,1e-12,0.,0.,0.,0.), 'LATERAL_MOTION_UNSUPPORTED'),
    ((.1,0.,0.,0.,.1,0.), 'NONPLANAR_COMMAND'),
    ((.6,0.,0.,0.,0.,0.), 'COMMAND_LIMIT_EXCEEDED'),
    ((float('nan'),0.,0.,0.,0.,0.), 'NONFINITE_COMMAND')])
def test_velocity_contract(values, reason):
    assert velocity_reason(values) == reason


def test_real_nav_topics_require_lease_reject_lateral_and_do_not_replay(ros):
    add, spin = ros
    adapter = add(NavVelocityAdapter)
    peer = add(Node, 'nav_adapter_test_peer')
    raw = peer.create_publisher(Twist, '/onboard/nav_cmd_raw', 1)
    authority = peer.create_publisher(MotionAuthority, '/motion/authority', 1)
    received = []
    peer.create_subscription(MotionCommand, '/motion/nav_command', received.append, 10)
    spin(.3)
    command = Twist(); command.linear.x=.1
    raw.publish(command); spin()
    assert not received
    def lease(owner='NAVIGATION', generation=7):
        m=MotionAuthority();m.header.stamp=peer.get_clock().now().to_msg()
        m.owner=owner;m.mission_id='mission';m.generation=generation
        m.healthy=True;m.lease_remaining=2.;authority.publish(m);spin(.05)
    lease();raw.publish(command);spin(.05)
    assert received[-1].owner == 'NAVIGATION' and received[-1].generation == 7
    assert received[-1].mission_id == 'mission' and received[-1].twist.linear.x == .1
    bad=Twist();bad.linear.y=1e-9;raw.publish(bad);spin(.05)
    assert received[-1].twist.linear.y == 0. and received[-1].twist.linear.x == 0.
    count=len(received);lease('SKILL',8);raw.publish(command);spin(.05)
    assert len(received)==count
    lease('NAVIGATION',9);spin(.05)
    assert len(received)==count  # New lease cannot replay the previous raw command.
    raw.publish(command);spin(.05)
    assert received[-1].generation==9
    lease(generation=9);spin(.25)
    assert received[-1].twist.linear.x==0.
    count=len(received);spin(.15)
    assert len(received)==count  # A stale raw source cannot be kept alive by this adapter.


def snapshot(node, seq=1):
    return dict(connected=True, handshake=True, session=100, sequence=seq,
        gateway_sequence=seq, stamp_ns=node.get_clock().now().nanoseconds,
        status=dict(known=True, mode='HOLD', nav_active=False, skill_active=False,
            stopped=True, safe_stance=True, control_releasable=True, obstacle_id='',
            estop=False,human_enable=True,fault_code='',skill_state='idle',capabilities=[]), error_code='')


def test_telemetry_gateway_restart_accepts_low_sequence_and_rejects_old_sessions(ros):
    add, _ = ros
    adapter = add(TelemetryAdapter)
    adapter._status(String(data=json.dumps(snapshot(adapter, 1000))))
    assert adapter.valid
    for session in (101, 102):
        fresh = snapshot(adapter, 1)
        fresh['session'] = session
        adapter._status(String(data=json.dumps(fresh)))
        assert adapter.valid and adapter.gateway_session == session
        assert adapter.gateway_sequence == 1
        for old_session in range(100, session):
            replay = snapshot(adapter, 2000)
            replay['session'] = old_session
            adapter._status(String(data=json.dumps(replay)))
            assert not adapter.valid and adapter.gateway_session == session
            assert adapter.gateway_sequence == 1
        fresh = snapshot(adapter, 2)
        fresh['session'] = session
        adapter._status(String(data=json.dumps(fresh)))
        assert adapter.valid
        adapter._status(String(data=json.dumps(fresh)))
        assert not adapter.valid  # Same-session replay protection remains active.


def test_stale_session_cannot_advance_telemetry_replay_watermark(ros):
    add, _ = ros
    adapter = add(TelemetryAdapter)
    adapter._status(String(data=json.dumps(snapshot(adapter, 1))))
    stale = snapshot(adapter, 10000)
    stale.update(session=101, stamp_ns=stale['stamp_ns'] - 1_000_000_000)
    adapter._status(String(data=json.dumps(stale)))
    assert adapter.gateway_session == 100 and adapter.gateway_sequence == 1
    adapter._status(String(data=json.dumps(snapshot(adapter, 2))))
    assert adapter.valid


def test_telemetry_uses_new_measured_feedback_and_stale_state_is_unknown(ros):
    add, spin=ros
    adapter=add(TelemetryAdapter)
    peer=add(Node,'telemetry_adapter_test_peer')
    pub=peer.create_publisher(String,'/stm32/status',10)
    received=[]
    peer.create_subscription(MotionTelemetry,'/motion/telemetry',received.append,10)
    spin(.25)
    data=snapshot(peer)
    pub.publish(String(data=json.dumps(data)));spin(.06)
    assert received[-1].known and received[-1].safe_stance
    assert received[-1].header.stamp.sec*10**9+received[-1].header.stamp.nanosec == data['stamp_ns']
    pub.publish(String(data=json.dumps(data)));spin(.06)
    assert not received[-1].known and not received[-1].safe_stance
    data=snapshot(peer,2);pub.publish(String(data=json.dumps(data)));spin(.06)
    assert received[-1].known
    spin(.35)
    assert not received[-1].known and not received[-1].stopped and not received[-1].control_releasable
    data=snapshot(peer,3);data['status']['estop']=True
    pub.publish(String(data=json.dumps(data)));spin(.06)
    assert not received[-1].known and received[-1].error_code in ('PHYSICAL_ESTOP','STATUS_STALE_OR_UNAVAILABLE')
    assert not adapter.get_service_names_and_types() or all(name not in ('/motion/get_state', '/motion/set_control_mode')
        for name,_ in adapter.get_service_names_and_types_by_node(adapter.get_name(), '/'))


@pytest.mark.parametrize('field,value', [('known',1),('stopped','true'),('safe_stance',None),('nav_active',1)])
def test_status_rejects_non_boolean_safety_fields(field,value):
    data=dict(connected=True,handshake=True,status=dict(known=True,mode='HOLD',nav_active=False,
        skill_active=False,stopped=True,safe_stance=True,control_releasable=True,
        estop=False,human_enable=True,fault_code=''))
    data['status'][field]=value
    with pytest.raises(ValueError):
        measured_status(data)


def test_arbiter_optional_final_output_binds_current_lease_and_remains_disabled(ros):
    from go2_mission_backends.motion_arbiter import MotionArbiter
    from rclpy.parameter import Parameter
    add,spin=ros
    arbiter=add(MotionArbiter,parameter_overrides=[Parameter('authorized_output_topic',value='/stm32/test_authorized')])
    peer=add(Node,'typed_arbiter_output_test_peer')
    received=[];leases=[]
    peer.create_subscription(MotionCommand,'/stm32/test_authorized',received.append,10)
    peer.create_subscription(MotionAuthority,'/motion/authority',leases.append,10)
    spin(.35)
    assert received and leases
    assert received[-1].generation==leases[-1].generation
    assert received[-1].owner==leases[-1].owner=='SAFE_HOLD'
    assert received[-1].header.frame_id=='base_footprint'
    assert received[-1].twist==Twist()
    assert not arbiter.enabled


def test_duplicate_nav_authority_cannot_extend_a_short_lease(ros):
    add,spin=ros
    adapter=add(NavVelocityAdapter)
    a=MotionAuthority();a.header.stamp=adapter.get_clock().now().to_msg()
    a.owner='NAVIGATION';a.mission_id='short_lease';a.generation=1
    a.healthy=True;a.lease_remaining=.08
    adapter._authority(a)
    assert adapter._authorized()
    spin(.05)
    adapter._authority(a)  # Same stamp must not restart the monotonic countdown.
    spin(.05)
    assert not adapter._authorized()
