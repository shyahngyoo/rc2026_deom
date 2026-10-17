"""POSIX pseudo-UART tests. The fake controller exists only in this test module."""
import os
import pty
import select
import threading
import time

import pytest
import serial
from quadruped_stm32_bridge.gateway_core import GatewayCore, BASE_FEATURES
from quadruped_stm32_bridge.protocol import MessageType as T, FrameParser, encode_frame
from quadruped_stm32_bridge.serial_gateway import SerialWorker


def measured_status():
    # Deliberately unsafe initial conditions: no physical human enable in fixture.
    return dict(known=True, mode='HOLD', nav_active=False, skill_active=False,
                stopped=True, safe_stance=False, control_releasable=False, estop=False,
                human_enable=False, fault_code='', capabilities=[], obstacle_id='', skill_state='idle')


class FakeUART:
    def __init__(self):
        self.master, slave = pty.openpty()
        self.port = os.ttyname(slave)
        os.close(slave)
        self.stop = threading.Event()
        self.frames = []
        self.sequence = 0
        self.session = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, kind, payload):
        self.sequence += 1
        packet = encode_frame(kind, self.sequence, self.session, payload)
        # Intentionally split the frame across writes.
        os.write(self.master, packet[:7])
        os.write(self.master, packet[7:])

    def run(self):
        parser=FrameParser()
        next_status=0
        while not self.stop.is_set():
            try:
                if select.select([self.master],[],[],.01)[0]:
                    data=os.read(self.master,4096)
                    for frame in parser.feed(data):
                        self.frames.append(frame)
                        if frame.session!=self.session:
                            self.session=frame.session;self.sequence=0
                        extra=(dict(protocol_version=1,features=sorted(BASE_FEATURES))
                               if frame.kind==T.HELLO else {})
                        self.send(T.ACK,dict(acked_sequence=frame.sequence,acked_type=int(frame.kind),
                                             accepted=True,**extra))
                if self.session and time.monotonic()>=next_status:
                    self.send(T.STATUS,measured_status());next_status=time.monotonic()+.02
            except OSError:
                self.stop.wait(.01)

    def close(self):
        self.stop.set();self.thread.join(1);os.close(self.master)


def wait_for(predicate, timeout=2):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate(): return
        time.sleep(.01)
    raise AssertionError('condition not reached within bounded wait')


def test_pty_handshake_split_status_exclusive_and_no_default_motion():
    fake=FakeUART();core=GatewayCore(port=fake.port);lock=threading.RLock()
    worker=SerialWorker(core,lock,time.time_ns)
    try:
        worker.start()
        wait_for(lambda:core.handshake and core.status is not None)
        assert core.status['safe_stance'] is False
        assert core.status['human_enable'] is False
        assert core.gateway_sequence>0
        with pytest.raises(serial.SerialException):
            serial.Serial(fake.port,115200,timeout=.01)
        assert {frame.kind for frame in fake.frames}=={T.HELLO}
    finally:
        worker.close();fake.close()
    assert not worker.thread.is_alive() and not core.connected


def test_pty_disconnect_reconnect_new_session_and_finite_retry(tmp_path):
    fake=FakeUART();link=tmp_path/'controller';link.symlink_to(fake.port)
    core=GatewayCore(port=str(link),reconnect_interval=.03,max_reconnect_attempts=3)
    lock=threading.RLock();worker=SerialWorker(core,lock,time.time_ns)
    replacement=None
    try:
        worker.start();wait_for(lambda:core.handshake and core.status is not None)
        first_session=core.session
        core.set_authority('SAFE_HOLD','',1,0,True,time.monotonic())
        core.set_authority('NAVIGATION','old',2,5,True,time.monotonic())
        fake.close()
        replacement=FakeUART();link.unlink();link.symlink_to(replacement.port)
        wait_for(lambda:core.handshake and core.session!=first_session and core.status is not None)
        assert core.fenced_generation>=2 and not core.nav_active
        assert all(frame.kind==T.HELLO for frame in replacement.frames)
        replacement.close();replacement=None
        wait_for(lambda:worker.attempts==3 and core.error_code=='RECONNECT_EXHAUSTED')
        assert not core.connected
        snapshot=core.snapshot(time.monotonic(),time.time_ns())
        assert snapshot['status'] is None
    finally:
        worker.close()
        if replacement:replacement.close()
    assert worker.attempts==3 and not worker.thread.is_alive()
