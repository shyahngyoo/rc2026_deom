"""STM32 UART protocol v1 DRAFT. Existing firmware does not implement this wire format.

Header: magic(2), version(u8), type(u8), JSON length(u16 LE), sequence(u32 LE),
session(u64 LE), followed by UTF-8 compact JSON and CRC32(u32 LE) over header
and payload. SI velocity is converted to normalized forward/turn before WALK.
The controller must deduplicate (session, sequence), never replay a side effect,
and clear its command/authority on a new session or local communication timeout.
"""
from dataclasses import dataclass
from enum import IntEnum
import json
import math
import struct
import zlib

MAGIC = b'\xa5\x5a'
VERSION = 1
MAX_PAYLOAD = 1024
HEADER = struct.Struct('<2sBBHIQ')
CRC = struct.Struct('<I')
MAX_FRAME = HEADER.size + MAX_PAYLOAD + CRC.size


class MessageType(IntEnum):
    HELLO = 1
    SET_MODE = 2
    WALK = 3
    SKILL_START = 4
    SKILL_CANCEL = 5
    HOLD = 6
    STOP = 7
    HEARTBEAT = 8
    ACK = 128
    STATUS = 129
    FAULT = 130


@dataclass(frozen=True)
class Frame:
    kind: MessageType
    sequence: int
    session: int
    payload: dict


def _integer(value, maximum):
    return type(value) is int and 0 < value <= maximum


def encode_frame(kind, sequence, session, payload):
    if not _integer(sequence, 0xffffffff) or not _integer(session, 0xffffffffffffffff):
        raise ValueError('sequence/session must be nonzero unsigned integers')
    if not isinstance(payload, dict):
        raise ValueError('payload must be a JSON object')
    body = json.dumps(payload, separators=(',', ':'), sort_keys=True,
                      ensure_ascii=True, allow_nan=False).encode('utf-8')
    if len(body) > MAX_PAYLOAD:
        raise ValueError('payload exceeds 1024 bytes')
    header = HEADER.pack(MAGIC, VERSION, int(MessageType(kind)), len(body), sequence, session)
    packet = header + body
    return packet + CRC.pack(zlib.crc32(packet) & 0xffffffff)


def _no_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON key')
        result[key] = value
    return result


class FrameParser:
    """Bounded streaming parser; malformed frames cannot refresh any watchdog."""
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def reset(self):
        self.buffer.clear()

    def feed(self, data):
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError('serial data must be bytes')
        # Work in bounded chunks so a single malicious read cannot grow memory.
        frames = []
        for start in range(0, len(data), MAX_FRAME):
            self.buffer.extend(data[start:start + MAX_FRAME])
            while self.buffer:
                offset = self.buffer.find(MAGIC)
                if offset < 0:
                    keep = self.buffer[-1:] if self.buffer[-1:] == MAGIC[:1] else b''
                    if len(self.buffer) > len(keep):
                        self.errors += 1
                    self.buffer[:] = keep
                    break
                if offset:
                    del self.buffer[:offset]
                    self.errors += 1
                if len(self.buffer) < HEADER.size:
                    break
                magic, version, kind, length, sequence, session = HEADER.unpack_from(self.buffer)
                if (version != VERSION or length > MAX_PAYLOAD or not sequence or not session
                        or kind not in MessageType._value2member_map_):
                    del self.buffer[0]
                    self.errors += 1
                    continue
                total = HEADER.size + length + CRC.size
                if len(self.buffer) < total:
                    break
                packet = bytes(self.buffer[:total])
                if CRC.unpack_from(packet, total - CRC.size)[0] != zlib.crc32(packet[:-4]) & 0xffffffff:
                    del self.buffer[0]
                    self.errors += 1
                    continue
                del self.buffer[:total]
                try:
                    payload = json.loads(packet[HEADER.size:-4].decode('utf-8'),
                                         object_pairs_hook=_no_duplicate_keys,
                                         parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                    if not isinstance(payload, dict):
                        raise ValueError('payload is not object')
                except (ValueError, UnicodeError, RecursionError):
                    self.errors += 1
                    continue
                frames.append(Frame(MessageType(kind), sequence, session, payload))
        return frames


STATUS_BOOLEANS = ('known', 'nav_active', 'skill_active', 'stopped', 'safe_stance',
                   'control_releasable', 'estop', 'human_enable')
STATUS_MODES = ('HOLD', 'NAVIGATION', 'SKILL', 'DISABLED')
SKILL_STATES = ('idle', 'accepted', 'running', 'succeeded', 'failed', 'canceled')


def validate_status(data):
    """Never coerce 0/1, strings or absent safety evidence into booleans."""
    if not isinstance(data, dict) or any(type(data.get(k)) is not bool for k in STATUS_BOOLEANS):
        raise ValueError('STATUS requires exact booleans: ' + ','.join(STATUS_BOOLEANS))
    if data.get('mode') not in STATUS_MODES or data.get('skill_state') not in SKILL_STATES:
        raise ValueError('invalid controller mode or skill state')
    if any(not isinstance(data.get(k), str) for k in ('obstacle_id', 'fault_code')):
        raise ValueError('obstacle_id/fault_code must be strings')
    capabilities = data.get('capabilities')
    if (not isinstance(capabilities, list) or len(capabilities) > 18
            or any(type(x) is not int or not 1 <= x <= 18 for x in capabilities)
            or len(set(capabilities)) != len(capabilities)):
        raise ValueError('capabilities must be unique supported command IDs 1..18')
    if data['nav_active'] and data['skill_active']:
        raise ValueError('conflicting controller activities')
    if ((data['nav_active'] and data['mode'] != 'NAVIGATION') or
            (data['skill_active'] and data['mode'] != 'SKILL')):
        raise ValueError('controller mode/activity conflict')
    for key in ('request_id', 'runner_session'):
        if key in data and not isinstance(data[key], str):
            raise ValueError(key + ' must be string')
    if 'step_index' in data and (type(data['step_index']) is not int or data['step_index'] < -1):
        raise ValueError('step_index must be integer >= -1')
    return dict(data)


# Stable vectors consumed by the Python tests and suitable for a C decoder test.
TEST_VECTOR_HELLO = {
    'kind': 1, 'sequence': 1, 'session': 0x0102030405060708,
    'payload': {'protocol_version': 1},
    'hex': 'a55a010116000100000008070605040302017b2270726f746f636f6c5f76657273696f6e223a317d36d63165',
}
