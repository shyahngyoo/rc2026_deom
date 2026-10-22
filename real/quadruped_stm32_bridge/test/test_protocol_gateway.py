"""No robot: protocol, injected clocks, fail-closed authority and lifecycle checks."""
import math
import struct
import zlib

import pytest
from quadruped_stm32_bridge.protocol import (
    HEADER, CRC, MAGIC, MAX_FRAME, MAX_PAYLOAD, MessageType as T,
    FrameParser, TEST_VECTOR_HELLO, encode_frame, validate_status)
from quadruped_stm32_bridge.gateway_core import GatewayCore, BASE_FEATURES


def status(**changes):
    value = dict(known=True, mode='HOLD', stopped=True, safe_stance=True,
                 control_releasable=True, nav_active=False, skill_active=False,
                 obstacle_id='', skill_state='idle', request_id='', runner_session='',
                 step_index=-1, estop=False, human_enable=True, fault_code='', capabilities=[6, 7, 9])
    return dict(value, **changes)


class Controller:
    def __init__(self, core):
        self.core = core
        self.sequence = 0
        self.stamp = 1000000000

    def send(self, kind, payload, now=0.0, session=None):
        self.sequence += 1
        self.stamp += 1000000
        raw = encode_frame(kind, self.sequence, session or self.core.session, payload)
        self.core.receive(raw, now, self.stamp)
        return raw

    def ack(self, frame, now=0.0, accepted=True, **extra):
        return self.send(T.ACK, dict(acked_sequence=frame.sequence, acked_type=int(frame.kind),
                                    accepted=accepted, **extra), now)

    def status(self, now=0.0, **changes):
        return self.send(T.STATUS, status(**changes), now)


def frames(core, now=0.0):
    return FrameParser().feed(b''.join(core.outgoing(now)))


def ready(**settings):
    core = GatewayCore(enable_motion=True, test_mode=False, calibration_confirmed=True, **settings)
    core.connect(0.0)
    peer = Controller(core)
    hello, = frames(core)
    peer.ack(hello, protocol_version=1, features=sorted(BASE_FEATURES | {'SKILL_START', 'SKILL_CANCEL', 'STOP'}))
    peer.status()
    core.set_authority('SAFE_HOLD', '', 1, 0.0, True, 0.0)
    for frame in frames(core):
        peer.ack(frame)
    peer.status()
    return core, peer


def authorize(core, owner='NAVIGATION', generation=2, now=0.0):
    assert core.set_authority(owner, 'mission', generation, 5.0, True, now)
    return (owner, 'mission', generation)


def navigation(core, peer):
    token = authorize(core)
    core.nav_goal(True, 'goal-a', 0.01)
    mode, = frames(core, .01)
    assert mode.kind == T.SET_MODE
    peer.ack(mode, .02)
    peer.status(.02, mode='NAVIGATION', nav_active=True,
                stopped=False, control_releasable=False)
    return token


def request(request_id='r1', command_id=6, **extra):
    return dict(op='command', request_id=request_id, runner_session='runner', obstacle_id='obstacle',
                step_index=0, command_id=command_id, owner='SKILL', mission_id='mission', generation=2, **extra)


def skill_mode(core, peer):
    authorize(core, 'SKILL')
    assert core.skill_request(request(command_id=2), .01)
    mode, = frames(core, .01)
    peer.ack(mode, .02)
    peer.status(.02, mode='SKILL', skill_active=True)


def test_stable_c_vector_and_roundtrip():
    v = TEST_VECTOR_HELLO
    raw = encode_frame(v['kind'], v['sequence'], v['session'], v['payload'])
    assert raw.hex() == v['hex']
    frame, = FrameParser().feed(bytes.fromhex(v['hex']))
    assert frame.payload == v['payload'] and frame.session == v['session']
    assert HEADER.size == 18 and MAX_FRAME == 1046


@pytest.mark.parametrize('cut', range(1, 45))
def test_half_and_sticky_packets(cut):
    packet = encode_frame(T.WALK, 23, 5, {'forward': .2, 'turn': -.1})
    parser = FrameParser()
    assert parser.feed(packet[:cut]) == []
    decoded = parser.feed(packet[cut:] + packet)
    assert len(decoded) == 2
    assert decoded[0] == decoded[1]


def test_crc_garbage_and_malformed_header_resynchronize():
    packet = encode_frame(T.HELLO, 1, 4, {})
    bad = bytearray(packet); bad[-1] ^= 1
    parser = FrameParser()
    assert len(parser.feed(b'noise' + bytes(bad) + packet)) == 1
    assert parser.errors > 0
    parser.feed(b'x' * 100000)
    assert len(parser.buffer) <= MAX_FRAME
    oversized = HEADER.pack(MAGIC, 1, 1, MAX_PAYLOAD+1, 1, 1)
    assert len(parser.feed(oversized + packet)) == 1


@pytest.mark.parametrize('payload', [{'a': float('nan')}, {'a': float('inf')}, {'long': 'x'*1024}])
def test_payload_bounds_nonfinite(payload):
    with pytest.raises(ValueError): encode_frame(T.WALK, 1, 1, payload)


@pytest.mark.parametrize('key', ['known', 'stopped', 'safe_stance', 'control_releasable', 'nav_active',
                                 'skill_active', 'human_enable', 'estop'])
@pytest.mark.parametrize('bad', [1, 0, 'true', None])
def test_never_coerce_safety_flags(key, bad):
    with pytest.raises(ValueError): validate_status(status(**{key: bad}))


def test_handshake_requires_version_features_and_ack_is_not_status():
    core = GatewayCore(); core.connect(0)
    peer = Controller(core); hello, = frames(core)
    peer.ack(hello, protocol_version=1, features=list(BASE_FEATURES))
    assert core.handshake and core.status is None
    assert core.snapshot(0, 1)['status'] is None
    assert not core.skill_request(request(), 0)
    assert all(f.kind not in (T.WALK, T.HOLD, T.STOP, T.SET_MODE) for f in frames(core))
    other = GatewayCore();other.connect(0);p=Controller(other);hello,=frames(other)
    p.ack(hello, protocol_version=1, features=['WALK', 'STATUS'])
    assert not other.handshake
    assert other.error_code == 'HANDSHAKE_UNSUPPORTED'


def test_defaults_cannot_drive_even_with_human_and_valid_authority():
    core = GatewayCore();core.connect(0);p=Controller(core);hello,=frames(core)
    p.ack(hello, protocol_version=1, features=list(BASE_FEATURES))
    p.status();token=authorize(core);core.nav_goal(True,'g',.01)
    assert not core.velocity((.1,0,0,0,0,0), .02, command_token=token)
    assert frames(core,.02) == []


def test_navigation_mode_handshake_then_normalized_authorized_velocity():
    core,peer=ready();token=navigation(core,peer)
    assert core.velocity((.25,0,0,0,0,-.5),.03,command_token=token)
    frame,=frames(core,.03)
    assert frame.kind==T.WALK
    assert frame.payload['forward']==.5 and frame.payload['turn']==-.5
    assert frame.payload['generation']==2


@pytest.mark.parametrize('values', [(0,1e-15,0,0,0,0),(0,0,1e-15,0,0,0),
                                   (.51,0,0,0,0,0),(0,0,0,0,0,1.01),
                                   (float('nan'),0,0,0,0,0)])
def test_lateral_nonplanar_out_of_range_rejected(values):
    core,peer=ready();token=navigation(core,peer)
    assert not core.velocity(values,.03,command_token=token)
    assert all(f.kind!=T.WALK for f in frames(core,.03))


def test_missing_wrong_lease_and_spoofed_publisher_rejected():
    core,peer=ready()
    assert not core.velocity((.1,0,0,0,0,0),.01)
    token=navigation(core,peer)
    assert not core.velocity((.1,0,0,0,0,0),.03,command_token=('NAVIGATION','old',2))
    assert not core.velocity((.1,0,0,0,0,0),.03,publisher_valid=False,command_token=token)
    assert all(f.kind!=T.WALK for f in frames(core,.03))


def test_authority_change_discards_unsent_old_speed_and_terminal_holds():
    core,peer=ready();token=navigation(core,peer)
    assert core.velocity((.1,0,0,0,0,0),.03,command_token=token)
    authorize(core,'SAFE_HOLD',3,.04)
    assert all(f.kind!=T.WALK for f in frames(core,.04))
    assert core.fenced_generation>=3
    core,peer=ready();navigation(core,peer)
    core.nav_goal(False,'',.03)
    hold,=frames(core,.03)
    assert hold.kind==T.HOLD
    core.tick(.04)
    assert all(f.kind!=T.SET_MODE for f in frames(core,.04))


def test_status_duplicates_cannot_refresh_clock_or_gateway_sequence():
    core,peer=ready();original=core.snapshot(.01,3)
    raw=peer.status(.02)
    fresh=core.snapshot(.02,4)
    core.receive(raw,.2,9999999)
    duplicate=core.snapshot(.2,9999999)
    assert fresh['stamp_ns']==duplicate['stamp_ns']
    assert fresh['gateway_sequence']==duplicate['gateway_sequence']
    core.tick(.5)
    assert core.snapshot(.5,10000000000)['status'] is None
    assert original['stamp_ns']<fresh['stamp_ns']


def test_status_stale_fences_and_requests_hold():
    core,peer=ready();token=navigation(core,peer)
    core.velocity((.1,0,0,0,0,0),.03,command_token=token)
    frames(core,.03)
    core.tick(.5)
    assert core.snapshot(.5,50)['status'] is None
    assert any(f.kind==T.HOLD for f in frames(core,.5))
    peer.status(.51)
    authorize(core,generation=2,now=.51)
    core.nav_goal(True,'new-goal',.51)
    assert not core.velocity((.1,0,0,0,0,0),.52,command_token=token)


def test_reconnect_changes_session_forbids_old_generation_goal_and_pending():
    core,peer=ready();token=navigation(core,peer);oldsession=core.session
    core.velocity((.2,0,0,0,0,0),.03,command_token=token)
    core.disconnect('TEST_UNPLUG',.04);core.connect(.05)
    assert core.session!=oldsession
    assert [f.kind for f in frames(core,.05)]==[T.HELLO]
    assert not core.nav_active and not core.pending.get(2)
    assert 'goal-a' in core.blocked_nav_keys
    assert core.fenced_generation>=2


def test_old_session_and_invalid_status_cannot_produce_safe_evidence():
    core,peer=ready()
    peer.send(T.STATUS,status(),.01,session=(core.session+1)&0xffffffffffffffff or 1)
    previous=core.status_stamp_ns
    bad=status(stopped=1);peer.send(T.STATUS,bad,.02)
    assert core.status is None and core.error_code=='INVALID_STATUS'
    assert core.snapshot(.02,5)['status'] is None
    assert previous>0


@pytest.mark.parametrize('changes', [dict(estop=True),dict(human_enable=False),dict(fault_code='MOTOR_FAULT'),dict(known=False)])
def test_physical_safety_fields_gate_motion(changes):
    core,peer=ready();token=navigation(core,peer)
    peer.status(.03,**changes)
    assert not core.velocity((.1,0,0,0,0,0),.04,command_token=token)
    assert core.snapshot(.04,5)['status'] is None


def test_skill_unsupported_authority_mode_and_completion_not_ack():
    core,peer=ready();authorize(core,'SKILL')
    assert not core.skill_request(request(command_id=18),.01)
    assert core.pop_events()[-1]['error_code']=='UNSUPPORTED'
    assert core.skill_request(request('r2'),.01)
    mode,=frames(core,.01);assert mode.kind==T.SET_MODE
    peer.ack(mode,.02);assert core.deferred_skill
    peer.status(.03,mode='SKILL',skill_active=True)
    start,=frames(core,.03);assert start.kind==T.SKILL_START
    peer.ack(start,.04)
    assert core.status['skill_state']=='idle'  # ACK never synthesizes completion.
    assert core.active_skill
    assert not core.skill_request(request('r3'),.04)
    peer.status(.05,mode='SKILL',skill_active=True,skill_state='running',
                request_id='r2',runner_session='runner',obstacle_id='obstacle',step_index=0)
    assert core.active_skill
    peer.status(.06,mode='SKILL',skill_active=False,skill_state='succeeded',
                request_id='r2',runner_session='runner',obstacle_id='obstacle',step_index=0)
    assert core.active_skill is None


@pytest.mark.parametrize('terminal',['succeeded','failed','canceled'])
def test_real_skill_terminal_is_preserved(terminal):
    core,peer=ready();skill_mode(core,peer)
    assert core.skill_request(request('r2'),.03)
    start,=frames(core,.03);peer.ack(start,.04)
    peer.status(.05,mode='SKILL',skill_state=terminal,request_id='r2',
                runner_session='runner',obstacle_id='obstacle',step_index=0)
    assert core.snapshot(.05,99)['status']['skill_state']==terminal


def test_skill_busy_dedup_and_heartbeat_does_not_walk_or_complete():
    core,peer=ready();skill_mode(core,peer)
    assert core.skill_request(request('r2'),.03);start,=frames(core,.03);peer.ack(start,.03)
    peer.status(.04,mode='SKILL',skill_active=True,skill_state='running')
    assert not core.skill_request(request('r2'),.04)
    assert not core.skill_request(request('r3'),.04)
    assert core.velocity((0,)*6,.05,command_token=('SKILL','mission',2))
    heartbeat,=frames(core,.05)
    assert heartbeat.kind==T.HEARTBEAT
    assert core.status['skill_state']=='running'


def test_hold_cancel_stop_are_distinct_and_ack_does_not_release():
    for op,kind in [('hold',T.HOLD),('cancel',T.SKILL_CANCEL),('stop',T.STOP)]:
        core,peer=ready();skill_mode(core,peer)
        r=request('end');r['op']=op
        assert core.skill_request(r,.03)
        output=frames(core,.03)
        assert output[-1].kind==kind
        for frame in output:peer.ack(frame,.04)
        assert core.status['mode']=='SKILL'  # no invented stop/support state


def test_ack_timeout_retries_identical_bytes_then_fences():
    core,peer=ready(command_timeout=1.0);token=navigation(core,peer)
    core.velocity((.1,0,0,0,0,0),.03,command_token=token)
    original,=core.outgoing(.03)
    core.tick(.19);retry,=core.outgoing(.19);assert retry==original
    core.set_authority('NAVIGATION','mission',2,5,True,.2)
    peer.status(.2,mode='NAVIGATION',nav_active=True)
    # Keep command heartbeat fresh while withholding exactly this ACK.
    core.last_velocity_at=.34;core.tick(.35);retry,=core.outgoing(.35);assert retry==original
    core.last_velocity_at=.49;core.set_authority('NAVIGATION','mission',2,5,True,.49)
    peer.status(.49,mode='NAVIGATION',nav_active=True)
    core.tick(.51)
    assert any(e['error_code']=='ACK_TIMEOUT' for e in core.pop_events())
    assert any(f.kind==T.HOLD for f in frames(core,.51))


def test_hold_rejection_disables_handshake_and_never_enables_motion():
    core,peer=ready();navigation(core,peer);core.nav_goal(False,'',.03)
    hold,=frames(core,.03);peer.ack(hold,.04,accepted=False,reason='UNSUPPORTED')
    assert not core.handshake and core.error_code=='HOLD_UNSUPPORTED_OR_REJECTED'



def test_new_velocity_supersedes_unacked_speed_including_retries():
    core,peer=ready();token=navigation(core,peer)
    assert core.velocity((.2,0,0,0,0,0),.03,command_token=token)
    old,=frames(core,.03)
    assert old.payload['forward'] != 0
    assert core.velocity((0,)*6,.04,command_token=token)
    zero,=frames(core,.04)
    core.tick(.20)
    retried=frames(core,.20)
    assert all(f.kind != T.WALK or f.payload['forward'] == 0 for f in retried)
    assert old.sequence not in core.pending and zero.sequence in core.pending


def test_new_process_never_adopts_preexisting_navigation_lease():
    core=GatewayCore(enable_motion=True,test_mode=False,calibration_confirmed=True)
    core.connect(0);peer=Controller(core);hello,=frames(core)
    peer.ack(hello,protocol_version=1,features=list(BASE_FEATURES));peer.status()
    token=authorize(core,generation=50)
    core.nav_goal(True,'old-latched-goal',.01)
    assert not core.velocity((.2,0,0,0,0,0),.02,command_token=token)
    assert all(f.kind!=T.SET_MODE for f in frames(core,.02))
    assert core.fenced_generation==50


def test_pending_skill_only_matching_request_terminal_unlocks():
    core,peer=ready();skill_mode(core,peer)
    assert core.skill_request(request('r2'),.03);start,=frames(core,.03);peer.ack(start,.04)
    peer.status(.05,mode='SKILL',skill_state='succeeded',request_id='other',
                runner_session='runner',obstacle_id='obstacle',step_index=0)
    assert core.active_skill
    assert not core.skill_request(request('r3'),.06)


def test_individual_write_boundary_rechecks_expired_lease_and_queue_age():
    core,peer=ready();token=navigation(core,peer)
    core.velocity((.1,0,0,0,0,0),.03,command_token=token)
    # One frame per write: a preceding bounded slow write cannot carry this
    # command across its TTL or the lease deadline.
    core._enqueue(T.SET_MODE,dict(mode='NAVIGATION'),.03)
    assert len(core.outgoing(.03,max_packets=1))==1
    assert core.outgoing(.34,max_packets=1)==[]
    core,peer=ready();token=navigation(core,peer)
    core.set_authority('NAVIGATION','mission',2,.02,True,.03)
    core.velocity((.1,0,0,0,0,0),.03,command_token=token)
    assert core.outgoing(.06,max_packets=1)==[]



def test_hold_then_direction_intent_waits_for_measured_stop_before_reengaging():
    core,peer=ready();skill_mode(core,peer)
    hold=request('hold');hold['op']='hold'
    assert core.skill_request(hold,.03)
    hold_frame,=frames(core,.03);peer.ack(hold_frame,.04)
    assert core.skill_request(request('direction',command_id=2),.05)
    assert frames(core,.05)==[]  # ACK is not measured HOLD completion.
    assert core.hold_pending and core.pending_mode_intent
    peer.status(.06,mode='SKILL',skill_active=True,stopped=False)
    core.tick(.06);assert frames(core,.06)==[]
    peer.status(.07,mode='HOLD',request_id='hold',runner_session='runner',
                obstacle_id='obstacle',step_index=0,skill_state='succeeded')
    modes=frames(core,.07)
    assert len(modes)==1 and modes[0].kind==T.SET_MODE and modes[0].payload['mode']=='SKILL'


def test_stop_motor_disable_requires_explicit_fresh_authority():
    core,peer=ready();r=request('stop');r['op']='stop'
    assert not core.skill_request(r,.01)
    assert frames(core,.01)==[]
    skill_mode(core,peer);r=request('stop-authorized');r['op']='stop'
    assert core.skill_request(r,.03)
    stop,=frames(core,.03)
    assert stop.kind==T.STOP
    assert core.fenced_generation==2



def test_discrete_start_status_race_never_emits_walk_zero():
    core,peer=ready();skill_mode(core,peer)
    assert core.skill_request(request('r2'),.03)
    start,=frames(core,.03)
    assert start.kind==T.SKILL_START and core.status['skill_state']=='idle'
    assert core.velocity((0,)*6,.04,command_token=('SKILL','mission',2))
    output=frames(core,.04)
    assert [frame.kind for frame in output]==[T.HEARTBEAT]
    assert not core.velocity((.1,0,0,0,0,0),.05,command_token=('SKILL','mission',2))



def test_untrusted_identifiers_are_bounded_before_frame_encoding():
    core,peer=ready();authorize(core,'SKILL')
    oversized=request('x'*129)
    assert not core.skill_request(oversized,.01)
    assert core.pop_events()[-1]['error_code']=='INVALID_SKILL_REQUEST'
    assert not core.set_authority('SKILL','x'*129,3,5,True,.02)



def test_legal_bytewise_fragmentation_is_not_a_protocol_error():
    packet=encode_frame(T.STATUS,1,5,status())
    parser=FrameParser();decoded=[]
    for value in packet:
        decoded.extend(parser.feed(bytes([value])))
    assert len(decoded)==1 and parser.errors==0


@pytest.mark.parametrize('sent', [False, True])
@pytest.mark.parametrize('owner', ['SAFE_HOLD', 'NAVIGATION', 'SKILL'])
def test_authority_updates_preserve_hello_transaction(sent, owner):
    core = GatewayCore()
    core.connect(0)
    peer = Controller(core)
    hello = frames(core)[0] if sent else None
    core.set_authority(owner, 'mission', 1, 5, True, .01)
    if not sent:
        hello, = frames(core, .02)
    peer.ack(hello, .03, protocol_version=1, features=sorted(BASE_FEATURES))
    assert core.handshake
    assert not any(e['error_code'] == 'UNMATCHED_ACK' for e in core.pop_events())


def test_authority_updates_do_not_reset_hello_retry_budget():
    core = GatewayCore()
    core.connect(0)
    original, = core.outgoing(0)
    for generation, now in enumerate((.16, .32), 1):
        core.set_authority('SAFE_HOLD', '', generation, 0, True, now)
        core.tick(now)
        assert core.outgoing(now) == [original]
    core.set_authority('SAFE_HOLD', '', 3, 0, True, .48)
    core.tick(.48)
    assert not core.connected
    assert any(e['error_code'] == 'ACK_TIMEOUT' for e in core.pop_events())


@pytest.mark.parametrize('fault', [dict(human_enable=False), dict(estop=True),
                                  dict(known=False), dict(fault_code='MOTOR_FAULT')])
def test_repeated_fault_status_preserves_hold_retry_budget(fault):
    core, peer = ready()
    packets = []
    for i in range(1, 21):
        now = i * .05
        peer.status(now, **fault)
        core.tick(now)
        packets.extend(core.outgoing(now))
    assert len(packets) == 1 + core.config['max_ack_retries']
    assert all(packet == packets[0] for packet in packets)
    assert core.hold_pending
    timeouts = [e for e in core.pop_events() if e['error_code'] == 'ACK_TIMEOUT']
    assert len(timeouts) == 1 and timeouts[0]['command_type'] == int(T.HOLD)


def test_changed_fault_and_authority_preserve_hold_and_delayed_ack():
    core, peer = ready()
    peer.status(.01, human_enable=False)
    hold, = frames(core, .01)
    metadata = dict(core.hold_metadata)
    peer.status(.02, estop=True)
    core.set_authority('NAVIGATION', 'mission', 2, 5, True, .03)
    assert frames(core, .03) == []
    assert core.hold_metadata == metadata
    core.tick(.18)  # Queue a retry; ACK arrives before it is transmitted.
    peer.ack(hold, .19)
    assert frames(core, .19) == []
    assert core.hold_pending  # ACK alone cannot establish a safe measured stop.
    peer.status(.20, estop=True)
    assert core.hold_pending and frames(core, .20) == []
    peer.status(.21)
    assert not core.hold_pending
    assert not any(e['error_code'] == 'UNMATCHED_ACK' for e in core.pop_events())


def test_reconnect_discards_old_safety_transactions():
    core, peer = ready()
    peer.status(.01, human_enable=False)
    hold, = frames(core, .01)
    old_session = core.session
    core.disconnect('TEST_DISCONNECT', .02)
    assert not core.queue and not core.pending and not core.hold_pending
    core.connect(.03)
    hello, = frames(core, .03)
    assert hello.kind == T.HELLO and core.session != old_session
    peer.send(T.ACK, dict(acked_sequence=hold.sequence, acked_type=int(T.HOLD),
                         accepted=True), .04, session=old_session)
    assert not core.handshake
    peer.ack(hello, .05, protocol_version=1, features=sorted(BASE_FEATURES))
    assert core.handshake
