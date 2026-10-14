"""Hurricane Unitree command encoding and explicit serial transports.

The command numbers come from ``task_to_do_unitree.py`` in the Hurricane
snapshot.  A send receipt only reports a local write (or a dry run); the
original protocol has no acknowledgement or action-completion message.
"""

from dataclasses import dataclass
import math
from threading import Lock
from types import MappingProxyType
from typing import Callable, Optional, Union


COMMAND_MAP = MappingProxyType({
    "左转": 1, "前进": 2, "右转": 3, "后退": 4, "站立": 5,
    "小跳1": 6, "小跳2": 7, "下台阶": 8,
    "正常模式": 9, "小斜坡模式": 10, "木桥B模式": 11, "木桥A模式": 12,
    "大斜坡模式": 13, "匍匐模式": 14, "沙坑模式": 15, "绕杆模式": 16,
    "高墙模式": 17, "台阶模式": 18, "左平移": 19, "右平移": 20,
    "大斜坡左": 4,
})

# English names are aliases for the same Unitree table, not another robot's
# protocol.  "stop" requests the original stand command, not an emergency stop.
COMMAND_ALIASES = MappingProxyType({
    "turn_left": "左转", "forward": "前进", "turn_right": "右转",
    "backward": "后退", "stand": "站立", "stop": "站立",
    "jump1": "小跳1", "jump2": "小跳2", "down_stairs": "下台阶",
    "normal_mode": "正常模式", "small_slope_mode": "小斜坡模式",
    "bridge_b_mode": "木桥B模式", "bridge_a_mode": "木桥A模式",
    "large_slope_mode": "大斜坡模式", "crawl_mode": "匍匐模式",
    "sand_mode": "沙坑模式", "slalom_mode": "绕杆模式",
    "wall_mode": "高墙模式", "stairs_mode": "台阶模式",
    "strafe_left": "左平移", "strafe_right": "右平移",
    "large_slope_left": "大斜坡左",
})

MIN_COMMAND = 1
MAX_COMMAND = 20
MAX_REPEAT = 100
_ALWAYS_SEND = frozenset({COMMAND_MAP["站立"], COMMAND_MAP["小跳1"],
                          COMMAND_MAP["小跳2"]})
CommandName = Union[str, int]


def command_number(command: CommandName) -> int:
    """Resolve an exact command name or a known integer Unitree command ID."""
    if isinstance(command, str):
        name = COMMAND_ALIASES.get(command, command)
        if name not in COMMAND_MAP:
            raise ValueError(f"Unknown Hurricane Unitree command: {command!r}")
        return COMMAND_MAP[name]
    if isinstance(command, bool) or not isinstance(command, int):
        raise TypeError("command must be a known command name or an integer")
    if not MIN_COMMAND <= command <= MAX_COMMAND:
        raise ValueError(f"command must be in [{MIN_COMMAND}, {MAX_COMMAND}]")
    return command


def _repeat_count(repeat: int) -> int:
    if isinstance(repeat, bool) or not isinstance(repeat, int):
        raise TypeError("repeat must be an integer")
    if not 1 <= repeat <= MAX_REPEAT:
        raise ValueError(f"repeat must be in [1, {MAX_REPEAT}]")
    return repeat


def encode_command(command: CommandName, repeat: int = 1) -> bytes:
    """Encode repeated ``S<number>E\r\n`` frames with bounded input sizes."""
    number = command_number(command)
    count = _repeat_count(repeat)
    return f"S{number}E\r\n".encode("ascii") * count


@dataclass(frozen=True)
class SendReceipt:
    """Local transmission evidence; never an acknowledgement from the robot."""

    command_id: int
    repeat: int
    payload: bytes
    bytes_written: int
    dry_run: bool = False
    suppressed: bool = False


class SerialTransportError(IOError):
    """The local serial write did not complete reliably."""


class _TransportBase:
    def __init__(self, *, dedup: bool = False):
        if not isinstance(dedup, bool):
            raise TypeError("dedup must be a bool")
        self._dedup = dedup
        self._last_sent = None
        self._closed = False
        self._lock = Lock()

    @property
    def closed(self) -> bool:
        return self._closed

    def _prepare(self, command, repeat, dedup):
        if self._closed:
            raise RuntimeError("transport is closed")
        if dedup is not None and not isinstance(dedup, bool):
            raise TypeError("dedup must be a bool or None")
        number = command_number(command)
        count = _repeat_count(repeat)
        payload = encode_command(number, count)
        suppress = (
            (self._dedup if dedup is None else dedup)
            and number not in _ALWAYS_SEND
            and self._last_sent == (number, count)
        )
        return number, count, payload, suppress

    def __enter__(self):
        if self._closed:
            raise RuntimeError("transport is closed")
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class SerialTransport(_TransportBase):
    """Persistently open only the specified port and verify every write length.

    Construct this class only when real serial output is intended.  Importing
    this module and constructing ``DryRunTransport`` never open a serial port.
    ``serial_factory`` accepts the same keyword arguments as ``serial.Serial``.
    """

    def __init__(self, port: str, baudrate: int = 115200,
                 write_timeout: float = 0.2,
                 serial_factory: Optional[Callable] = None, *,
                 dedup: bool = False):
        super().__init__(dedup=dedup)
        if not isinstance(port, str) or not port.strip():
            raise ValueError("port must be an explicit, nonempty serial port")
        if isinstance(baudrate, bool) or not isinstance(baudrate, int):
            raise TypeError("baudrate must be a positive integer")
        if baudrate <= 0:
            raise ValueError("baudrate must be a positive integer")
        if (isinstance(write_timeout, bool)
                or not isinstance(write_timeout, (int, float))
                or not math.isfinite(write_timeout) or write_timeout <= 0):
            raise ValueError("write_timeout must be finite and positive")
        if serial_factory is None:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("Real serial output requires pyserial") from exc
            serial_factory = serial.Serial
        self.port = port
        self.baudrate = baudrate
        self.write_timeout = float(write_timeout)
        self._serial = serial_factory(
            port=port, baudrate=baudrate, timeout=0,
            write_timeout=self.write_timeout,
        )

    def send(self, command: CommandName, repeat: int = 1, *,
             dedup: Optional[bool] = None) -> SendReceipt:
        with self._lock:
            number, count, payload, suppress = self._prepare(command, repeat, dedup)
            if suppress:
                return SendReceipt(number, count, payload, 0, suppressed=True)
            try:
                written = self._serial.write(payload)
                if (isinstance(written, bool) or not isinstance(written, int)
                        or written != len(payload)):
                    raise SerialTransportError(
                        f"Incomplete serial write to {self.port}: "
                        f"{written!r} of {len(payload)} bytes"
                    )
            except Exception as exc:
                # A failed write is never retained as a successful dedup key.
                self._last_sent = None
                if isinstance(exc, SerialTransportError):
                    raise
                raise SerialTransportError(
                    f"Serial write to {self.port} failed: {exc}"
                ) from exc
            self._last_sent = (number, count)
            return SendReceipt(number, count, payload, written)

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._last_sent = None
            self._serial.close()


class DryRunTransport(_TransportBase):
    """Record planned bytes without importing pyserial or opening hardware."""

    def __init__(self, *, dedup: bool = False):
        super().__init__(dedup=dedup)
        self.sent = []
        self.receipts = []

    def send(self, command: CommandName, repeat: int = 1, *,
             dedup: Optional[bool] = None) -> SendReceipt:
        with self._lock:
            number, count, payload, suppress = self._prepare(command, repeat, dedup)
            if suppress:
                receipt = SendReceipt(number, count, payload, 0, dry_run=True,
                                      suppressed=True)
            else:
                self.sent.append(payload)
                self._last_sent = (number, count)
                receipt = SendReceipt(number, count, payload, 0, dry_run=True)
            self.receipts.append(receipt)
            return receipt

    def close(self):
        with self._lock:
            self._closed = True
            self._last_sent = None
