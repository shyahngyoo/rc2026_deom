"""Protocol contract and hardware-free transport tests."""

import pytest

from robocon_mission.hurricane_protocol import (
    COMMAND_MAP,
    MAX_REPEAT,
    DryRunTransport,
    SerialTransport,
    SerialTransportError,
    command_number,
    encode_command,
)


class FakeSerial:
    def __init__(self, **kwargs):
        self.open_args = kwargs
        self.writes = []
        self.close_calls = 0
        self.next_result = None
        self.next_error = None

    def write(self, payload):
        self.writes.append(payload)
        if self.next_error is not None:
            error, self.next_error = self.next_error, None
            raise error
        if self.next_result is not None:
            result, self.next_result = self.next_result, None
            return result
        return len(payload)

    def close(self):
        self.close_calls += 1


def transport_with_fake(**kwargs):
    opened = []

    def factory(**options):
        serial = FakeSerial(**options)
        opened.append(serial)
        return serial

    transport = SerialTransport("/dev/ttyUSB-test", serial_factory=factory,
                                **kwargs)
    return transport, opened


def test_original_unitree_table_and_frames():
    assert COMMAND_MAP["下台阶"] == 8
    assert COMMAND_MAP["木桥B模式"] == 11
    assert COMMAND_MAP["木桥A模式"] == 12
    assert COMMAND_MAP["大斜坡左"] == COMMAND_MAP["后退"] == 4
    assert encode_command("前进", 4) == b"S2E\r\n" * 4
    assert encode_command("bridge_a_mode") == b"S12E\r\n"
    assert command_number("stop") == 5
    assert encode_command(20, MAX_REPEAT) == b"S20E\r\n" * MAX_REPEAT


@pytest.mark.parametrize("command", [0, 21, 26, -1, "unknown", "2"])
def test_reject_unknown_or_out_of_range_commands(command):
    with pytest.raises(ValueError):
        encode_command(command)


@pytest.mark.parametrize("command", [True, False, 2.0, None, b"forward"])
def test_command_ids_require_integers(command):
    with pytest.raises(TypeError):
        encode_command(command)


@pytest.mark.parametrize("repeat", [0, -1, MAX_REPEAT + 1])
def test_repeat_is_bounded(repeat):
    with pytest.raises(ValueError):
        encode_command(2, repeat)


@pytest.mark.parametrize("repeat", [True, 1.0, "1", None])
def test_repeat_does_not_truncate_non_integers(repeat):
    with pytest.raises(TypeError):
        encode_command(2, repeat)


def test_explicit_port_is_opened_once_with_a_write_timeout():
    transport, opened = transport_with_fake()
    first = transport.send("forward", 2)
    second = transport.send("forward", 2)
    assert len(opened) == 1
    assert opened[0].open_args == {
        "port": "/dev/ttyUSB-test", "baudrate": 115200,
        "timeout": 0, "write_timeout": 0.2,
    }
    assert opened[0].writes == [b"S2E\r\n" * 2] * 2
    assert first.bytes_written == second.bytes_written == 10
    assert not first.dry_run and not first.suppressed
    transport.close()
    transport.close()
    assert opened[0].close_calls == 1
    assert transport.closed
    with pytest.raises(RuntimeError, match="closed"):
        transport.send("stop")


@pytest.mark.parametrize("bad_result", [0, 1, 999, False, "5"])
def test_short_or_invalid_write_is_not_cached_as_success(bad_result):
    transport, opened = transport_with_fake(dedup=True)
    opened[0].next_result = bad_result
    with pytest.raises(SerialTransportError, match="Incomplete"):
        transport.send("forward")
    retry = transport.send("forward")
    assert not retry.suppressed
    assert len(opened[0].writes) == 2
    assert transport.send("forward").suppressed
    transport.close()


def test_failed_write_clears_prior_successful_dedup_state():
    transport, opened = transport_with_fake(dedup=True)
    transport.send("forward")
    opened[0].next_error = OSError("cable disconnected")
    with pytest.raises(SerialTransportError, match="cable disconnected"):
        transport.send("turn_left")
    assert not transport.send("forward").suppressed
    assert len(opened[0].writes) == 3
    transport.close()


def test_only_explicit_dedup_suppresses_normal_commands():
    transport, opened = transport_with_fake()
    transport.send("forward")
    assert transport.send("forward", dedup=True).suppressed
    assert not transport.send("forward").suppressed
    assert len(opened[0].writes) == 2
    transport.close()


@pytest.mark.parametrize("command", ["stop", "站立", 5, "jump1", 6, "jump2", 7])
def test_stop_and_jump_are_never_suppressed(command):
    transport, opened = transport_with_fake(dedup=True)
    transport.send(command)
    assert not transport.send(command, dedup=True).suppressed
    assert len(opened[0].writes) == 2
    transport.close()


def test_dry_run_receipts_do_not_claim_hardware_writes():
    with DryRunTransport() as transport:
        receipt = transport.send("wall_mode", 2)
        transport.send("wall_mode", 2)
        assert transport.sent == [b"S17E\r\n" * 2] * 2
        assert receipt.bytes_written == 0
        assert receipt.dry_run
        assert not receipt.suppressed
    assert transport.closed
    with pytest.raises(RuntimeError, match="closed"):
        transport.send("stop")


def test_dry_run_uses_the_same_explicit_dedup_policy():
    transport = DryRunTransport(dedup=True)
    transport.send("forward")
    assert transport.send("forward").suppressed
    transport.send("stop")
    assert not transport.send("stop").suppressed
    assert transport.sent == [b"S2E\r\n", b"S5E\r\n", b"S5E\r\n"]
    assert len(transport.receipts) == 4


@pytest.mark.parametrize("port", [None, "", "   "])
def test_no_port_selection_or_hardware_open_when_port_missing(port):
    opened = []
    with pytest.raises(ValueError, match="explicit"):
        SerialTransport(port, serial_factory=lambda **kwargs: opened.append(kwargs))
    assert opened == []


@pytest.mark.parametrize("timeout", [0, -1, float("inf"), float("nan"), True])
def test_serial_write_timeout_must_bound_the_write(timeout):
    opened = []
    with pytest.raises(ValueError, match="finite and positive"):
        SerialTransport("/dev/ttyUSB-test", write_timeout=timeout,
                        serial_factory=lambda **kwargs: opened.append(kwargs))
    assert opened == []
