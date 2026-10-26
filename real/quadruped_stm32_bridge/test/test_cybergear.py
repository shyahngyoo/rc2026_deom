import pytest
from quadruped_stm32_bridge.cybergear_core import (
    StatusParser, crc8, encode_key, validate_plan, Runner, MODE_KEYS)


def frame(mode='a', fault=0):
    data = bytes([165, 90, ord(mode), fault, 2, 3])
    return data + bytes([crc8(data), 13])


def test_fragment_noise_crc_and_coalescing():
    p = StatusParser()
    good = frame()
    bad = bytearray(good); bad[6] ^= 1
    assert p.feed(b'noise' + bytes(bad) + good[:3]) == []
    assert p.feed(good[3:] + frame('e', 4)) == [
        dict(mode='a', fault=0, motor_id=2, detail=3),
        dict(mode='e', fault=4, motor_id=2, detail=3)]
    assert len(p.buffer) < 8


def test_remote_flag_and_invalid_status_rejected():
    assert StatusParser().feed(frame(chr(ord('a') | 128))) == []
    assert StatusParser().feed(frame('a', 16)) == []


@pytest.mark.parametrize('key', [0, 21, 22, 23, True, '2', 2.0])
def test_no_remote_enable_from_host(key):
    with pytest.raises(ValueError):
        encode_key(key)


def test_bytes_and_mode_mapping():
    assert encode_key(12) == b'\x53\x0c\x45'
    assert MODE_KEYS[12] == 'e'
    assert MODE_KEYS[18] == 'b'


@pytest.mark.parametrize('step', [
    {'type':'key','key':6,'duration_s':0},
    {'type':'key','key':6,'duration_s':float('nan')},
    {'type':'mode','key':2}, {'type':'key','key':12,'duration_s':1},
    {'type':'waypoint','x':1,'y':float('inf')},
    {'type':'waypoint','x':1,'y':2,'mode_key':True},
    {'type':'wait','duration_s':1,'typo':2}])
def test_invalid_plan(step):
    with pytest.raises(ValueError):
        validate_plan({'steps':[step]})


def test_mode_requires_feedback_after_send():
    r = Runner(validate_plan({'steps':[{'type':'mode','key':12}]})); r.start()
    assert r.tick(1, {'mode':'e'}, 0) == [12]
    assert r.tick(2, {'mode':'e'}, 0) == []
    assert r.index == 0
    r.tick(3, {'mode':'e'}, 3)
    assert r.index == 1
    assert r.tick(4, {'mode':'e'}, 4) == [5]
    assert not r.active


def test_jump_is_sent_once_and_not_replayed_after_stop():
    r = Runner(validate_plan({'steps':[{'type':'key','key':6,'duration_s':2}]})); r.start()
    assert r.tick(0, {}, 0) == [6]
    assert r.tick(0.5, {}, 0) == []
    assert r.tick(1, {}, 0) == []
    assert r.stop('fault') == [5]
    assert r.tick(2, {}, 0) == []


def test_waypoint_turn_forward_arrive():
    r = Runner(validate_plan({'steps':[{'type':'waypoint','x':1,'y':0}]})); r.start()
    assert r.tick(0, {'mode':'a'}, 0, (0,0,1)) == [9]
    assert r.tick(0.1, {'mode':'a'}, 0.1, (0,0,1)) == [3]
    assert r.tick(0.2, {'mode':'a'}, 0.2, (0,0,0)) == []
    assert r.tick(0.4, {'mode':'a'}, 0.4, (0,0,0)) == [2]
    assert r.tick(1, {'mode':'a'}, 1, (1,0,0)) == [5]


def test_pose_loss_and_mode_timeout():
    r = Runner(validate_plan({'steps':[{'type':'waypoint','x':1,'y':0}]})); r.start()
    r.tick(0, {'mode':'a'}, 0)
    assert r.tick(1, {'mode':'a'}, 1) == [5]
    assert r.state == 'localization_unavailable'
    r = Runner(validate_plan({'steps':[{'type':'mode','key':12,'timeout_s':1}]})); r.start()
    r.tick(0, {'mode':'a'}, 0)
    assert r.tick(2, {'mode':'a'}, 2) == [5]
    assert r.state == 'step_timeout'
