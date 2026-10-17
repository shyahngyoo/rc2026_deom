"""ROS-free fail-closed gateway state and session fencing for protocol v1 draft."""
from collections import deque
import math
import secrets

from .protocol import FrameParser, MessageType as T, encode_frame, validate_status

DEFAULTS = dict(port='', baudrate=115200, read_timeout=0.02, write_timeout=0.05,
                reconnect_interval=1.0, max_reconnect_attempts=5, handshake_timeout=2.0,
                status_timeout=0.4, authority_timeout=0.4, command_timeout=0.3,
                ack_timeout=0.15, max_ack_retries=2, enable_motion=False, test_mode=True,
                calibration_confirmed=False, vx_full_scale=0.5, wz_full_scale=1.0,
                max_linear_speed=0.5, max_angular_speed=1.0,
                status_publish_frequency=20.0,
                arbiter_output_topic='/stm32/arbiter_command', base_frame='base_footprint',
                authority_topic='/motion/authority',
                navigation_status_topic='/navigate_to_pose/_action/status',
                expected_arbiter_node='motion_arbiter')
BASE_FEATURES = frozenset(('HOLD', 'SET_MODE', 'WALK', 'STATUS', 'HEARTBEAT'))


class GatewayCore:
    """All calls serialized by SerialGateway's lock; only its worker writes UART."""
    def __init__(self, **settings):
        unknown = set(settings) - set(DEFAULTS)
        if unknown:
            raise ValueError('unknown settings: ' + repr(unknown))
        self.config = dict(DEFAULTS, **settings)
        for key in ('read_timeout', 'write_timeout', 'reconnect_interval', 'handshake_timeout',
                    'status_timeout', 'authority_timeout', 'command_timeout', 'ack_timeout',
                    'vx_full_scale', 'wz_full_scale', 'max_linear_speed', 'max_angular_speed',
                    'status_publish_frequency'):
            if type(self.config[key]) not in (int, float) or not math.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError('invalid ' + key)
        for key in ('enable_motion', 'test_mode', 'calibration_confirmed'):
            if type(self.config[key]) is not bool:
                raise ValueError(key + ' must be bool')
        for key in ('baudrate', 'max_reconnect_attempts', 'max_ack_retries'):
            if type(self.config[key]) is not int or self.config[key] < (1 if key == 'baudrate' else 0):
                raise ValueError('invalid ' + key)
        self.parser = FrameParser()
        self.connected = self.handshake = False
        self.session = self.sequence = self.received_sequence = 0
        self.gateway_sequence = 0
        self.status = None
        self.status_at = self.connected_at = 0.0
        self.status_stamp_ns = 0
        self.features = frozenset()
        self.authority = None
        self.authority_at = 0.0
        self.max_generation = self.fenced_generation = -1
        self.nav_key = ''
        self.nav_active = False
        self.blocked_nav_keys = deque(maxlen=128)
        self.mode_requested = None
        self.pending_mode_intent = None
        self.hold_pending = False
        self.hold_metadata = None
        self.last_velocity_at = None
        self.deferred_skill = None
        self.active_skill = None
        self.ending_skill_request = None
        self.queue = deque()
        self.pending = {}
        self.request_cache = {}  # bounded per connection; never rerun a duplicate request
        self.events = deque(maxlen=256)
        self.error_code = 'DISCONNECTED'
        self._fence_reason = None
        self.session_authority_seen = False

    @property
    def motion_enabled(self):
        return (self.config['enable_motion'] and not self.config['test_mode']
                and self.config['calibration_confirmed'])

    def event(self, error_code='', **fields):
        self.events.append(dict(error_code=error_code, **fields))

    def pop_events(self):
        result = list(self.events)
        self.events.clear()
        return result

    def connect(self, now):
        self._fence('NEW_SERIAL_SESSION', now, send_hold=False)
        prior = self.session
        self.session = secrets.randbits(64) or 1
        while self.session == prior:
            self.session = secrets.randbits(64) or 1
        self.sequence = self.received_sequence = 0
        self.connected, self.handshake = True, False
        self.connected_at = now
        self.session_authority_seen = False
        self.status = None
        self.status_stamp_ns = 0
        self.features = frozenset()
        self.parser.reset()
        self.request_cache.clear()
        self.error_code = 'HANDSHAKE_PENDING'
        self._enqueue(T.HELLO, dict(protocol_version=1, host='quadruped_stm32_bridge'), now)

    def disconnect(self, reason, now):
        self._fence(reason, now, send_hold=False)
        self.connected = self.handshake = False
        self.status = None
        self.error_code = reason
        self.parser.reset()
        self.event(reason, event='link_lost')

    def _fence(self, reason, now, send_hold=True):
        self.fenced_generation = max(self.fenced_generation, self.max_generation)
        if self.nav_key and self.nav_key not in self.blocked_nav_keys:
            self.blocked_nav_keys.append(self.nav_key)
        self.nav_active = False
        self.queue.clear()
        self.pending.clear()
        self.mode_requested = None
        self.pending_mode_intent = None
        self.hold_pending = False
        self.hold_metadata = None
        self.last_velocity_at = None
        if self.deferred_skill:
            self.event(reason, event='rejected', **self.deferred_skill)
        self.deferred_skill = None
        self.active_skill = None
        self.ending_skill_request = None
        if send_hold and self.connected and self.handshake and self.motion_enabled and 'HOLD' in self.features:
            self._enqueue(T.HOLD, dict(reason=reason), now, safety=True)
        self._fence_reason = reason

    def set_authority(self, owner, mission_id, generation, lease_remaining, healthy, now):
        if (owner not in ('SAFE_HOLD', 'NAVIGATION', 'SKILL')
                or not isinstance(mission_id, str) or len(mission_id) > 128 or type(generation) is not int
                or generation < self.max_generation or type(healthy) is not bool
                or type(lease_remaining) not in (int, float) or not math.isfinite(lease_remaining)
                or lease_remaining < 0):
            self.invalidate_authority('INVALID_AUTHORITY', now)
            return False
        token = (owner, mission_id, generation)
        previous = self.authority[:3] if self.authority else None
        self.max_generation = generation
        if self.connected and not self.session_authority_seen:
            # A restarted gateway cannot adopt a lease that predates its serial session.
            self.fenced_generation = max(self.fenced_generation, generation)
            self.session_authority_seen = True
        self.authority = (*token, now + lease_remaining, healthy)
        self.authority_at = now
        if owner == 'SAFE_HOLD':
            if previous != token or self.mode_requested not in (None, 'HOLD'):
                self._fence('SAFE_HOLD_AUTHORITY', now)
            return True
        if previous != token:
            self.queue.clear()
            self.pending.clear()
            self.mode_requested = None
            self.pending_mode_intent = None
            self.last_velocity_at = None
            self.deferred_skill = None
            self.active_skill = None
            self.ending_skill_request = None
        return True

    def invalidate_authority(self, reason, now):
        self.authority = None
        if self._fence_reason != reason:
            self._fence(reason, now)
        self.event(reason, event='authority_invalid')

    def _authorized(self, owner, now):
        a = self.authority
        return bool(a and a[0] == owner and a[1] and a[2] > self.fenced_generation
                    and a[4] and now < a[3]
                    and now - self.authority_at <= self.config['authority_timeout'])

    def _fresh_status(self, now):
        s = self.status
        return bool(self.connected and self.handshake and s and s['known']
                    and now - self.status_at <= self.config['status_timeout'])

    def _safe_motion(self, owner, now):
        s = self.status
        return bool(self.motion_enabled and self._authorized(owner, now) and self._fresh_status(now)
                    and not s['estop'] and s['human_enable'] and not s['fault_code'])

    def nav_goal(self, active, key, now):
        if type(active) is not bool or not isinstance(key, str):
            return
        if active and key and key not in self.blocked_nav_keys:
            self.nav_active, self.nav_key = True, key
            self._ensure_mode('NAVIGATION', now)
        else:
            if self.nav_active and self.connected and self.handshake and self.motion_enabled:
                self._hold(dict(reason='NAVIGATION_GOAL_TERMINAL'), now)
            self.nav_active = False
            self.nav_key = key

    def _ensure_mode(self, owner, now):
        if self.hold_pending:
            return False
        if not self._safe_motion(owner, now):
            return False
        s = self.status
        if s['mode'] == owner and ((owner == 'NAVIGATION' and s['nav_active']) or
                                  (owner == 'SKILL' and s['skill_active'])):
            return True
        if self.mode_requested == owner:
            return False
        if not (s['stopped'] and s['safe_stance'] and s['control_releasable']
                and not s['nav_active'] and not s['skill_active']):
            return False
        self._enqueue(T.SET_MODE, dict(mode=owner, mission_id=self.authority[1],
                                      generation=self.authority[2]), now)
        self.mode_requested = owner
        return False

    def velocity(self, values, now, publisher_valid=True, command_token=None):
        if not publisher_valid:
            self._fence('OUTPUT_BYPASS_DETECTED', now)
            self.event('OUTPUT_BYPASS_DETECTED', event='rejected')
            return False
        if (len(values) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
                or any(values[i] != 0 for i in (1, 2, 3, 4))):
            self.event('LATERAL_OR_NONPLANAR_COMMAND', event='rejected')
            self._fence('LATERAL_OR_NONPLANAR_COMMAND', now)
            return False
        vx, wz = values[0], values[5]
        if (abs(vx) > self.config['max_linear_speed'] or abs(wz) > self.config['max_angular_speed']
                or abs(vx) > self.config['vx_full_scale'] or abs(wz) > self.config['wz_full_scale']):
            self.event('COMMAND_LIMIT_EXCEEDED', event='rejected')
            self._fence('COMMAND_LIMIT_EXCEEDED', now)
            return False
        if not self.authority or command_token != self.authority[:3]:
            self.event('OLD_COMMAND_AUTHORITY_TOKEN', event='rejected')
            return False
        owner = self.authority[0] if self.authority else ''
        if self.hold_pending or owner not in ('NAVIGATION', 'SKILL') or not self._safe_motion(owner, now):
            return False
        if owner == 'NAVIGATION' and not self.nav_active:
            return False
        s = self.status
        if s['mode'] != owner or not s['nav_active' if owner == 'NAVIGATION' else 'skill_active']:
            return False
        # Zero heartbeat during discrete skills must never cancel their trajectory.
        if owner == 'SKILL' and (self.active_skill or s['skill_state'] in ('accepted', 'running')):
            if vx or wz:
                self.event('DISCRETE_SKILL_ACTIVE', event='rejected')
                return False
            self.last_velocity_at = now
            self._supersede_velocity()
            self._enqueue(T.HEARTBEAT, dict(owner=owner, mission_id=self.authority[1],
                                           generation=self.authority[2],
                                           valid_for_ms=max(1, int(self.config['command_timeout']*1000))), now)
            return True
        self.last_velocity_at = now
        # Supersede only unsent WALK, never replay an old queued speed after mode change.
        self._supersede_velocity()
        self._enqueue(T.WALK, dict(forward=vx/self.config['vx_full_scale'],
                                  turn=wz/self.config['wz_full_scale'],
                                  owner=owner, mission_id=self.authority[1],
                                  generation=self.authority[2],
                                  valid_for_ms=max(1, int(self.config['command_timeout']*1000))), now)
        return True

    def _supersede_velocity(self):
        self.queue = deque(q for q in self.queue if q['kind'] not in (T.WALK, T.HEARTBEAT))
        self.pending = {seq: p for seq, p in self.pending.items()
                        if p['kind'] not in (T.WALK, T.HEARTBEAT)}

    def _hold(self, metadata, now):
        self.queue = deque(q for q in self.queue if q['kind'] not in (T.WALK, T.HEARTBEAT, T.SET_MODE, T.SKILL_START))
        self.pending = {seq: p for seq, p in self.pending.items()
                        if p['kind'] not in (T.WALK, T.HEARTBEAT, T.SET_MODE, T.SKILL_START)}
        self.last_velocity_at = None
        self.mode_requested = 'HOLD'
        self.pending_mode_intent = None
        self.deferred_skill = None
        self.ending_skill_request = dict(metadata) if metadata.get('request_id') else None
        self._enqueue(T.HOLD, metadata, now, safety=True)

    def skill_request(self, request, now):
        if not isinstance(request, dict):
            self.event('INVALID_SKILL_REQUEST', event='rejected')
            return False
        fields = {k: request.get(k) for k in ('request_id', 'runner_session', 'obstacle_id', 'step_index')}
        op = request.get('op', 'command')
        if (any(not isinstance(fields[k], str) or not fields[k] or len(fields[k]) > 128 for k in
                ('request_id', 'runner_session', 'obstacle_id'))
                or type(fields['step_index']) is not int or not 0 <= fields['step_index'] <= 0x7fffffff
                or op not in ('command', 'cancel', 'hold', 'stop')):
            self.event('INVALID_SKILL_REQUEST', event='rejected', **fields)
            return False
        previous = self.request_cache.get(fields['request_id'])
        if previous is not None:
            if previous != request:
                self.event('REQUEST_ID_CONFLICT', event='rejected', **fields)
            else:
                self.event('DUPLICATE_REQUEST', event='duplicate', **fields)
            return False
        if len(self.request_cache) >= 2048:
            self.event('REQUEST_CACHE_LIMIT', event='rejected', **fields)
            return False
        self.request_cache[fields['request_id']] = dict(request)
        if not self.motion_enabled or not self.connected or not self.handshake:
            self.event('MOTION_DISABLED_OR_DISCONNECTED', event='rejected', **fields)
            return False
        if op == 'stop':
            # STOP requests motor disable/unloading. It is not a safe support
            # fallback and therefore requires explicit current SKILL authority.
            if (request.get('owner') != 'SKILL' or not self._safe_motion('SKILL', now)
                    or request.get('mission_id') != self.authority[1]
                    or type(request.get('generation')) is not int
                    or request['generation'] != self.authority[2]):
                self.event('NO_STOP_AUTHORITY', event='rejected', **fields)
                return False
            if 'STOP' not in self.features:
                self.event('UNSUPPORTED', event='rejected', **fields)
                return False
            self._fence('EXPLICIT_MOTOR_STOP', now, send_hold=False)
            self._enqueue(T.STOP, fields, now, safety=True)
            return True
        if op in ('hold', 'cancel'):
            kind = {'hold': T.HOLD, 'cancel': T.SKILL_CANCEL}[op]
            if kind.name not in self.features:
                self.event('UNSUPPORTED', event='rejected', **fields)
                return False
            # Emergency support/cancel requests do not require a live lease;
            # they never assert completion or call motor STOP.
            self._hold(dict(fields, reason='SKILL_' + op.upper()), now)
            if kind != T.HOLD:
                self._enqueue(kind, fields, now, safety=True)
            return True
        command = request.get('command_id')
        if (type(command) is not int or not 1 <= command <= 18 or command == 5):
            self.event('UNSUPPORTED', event='rejected', **fields)
            return False
        if (request.get('owner') != 'SKILL' or not self._safe_motion('SKILL', now)
                or request.get('mission_id') != self.authority[1]
                or type(request.get('generation')) is not int
                or request['generation'] != self.authority[2]):
            self.event('NO_SKILL_AUTHORITY', event='rejected', **fields)
            return False
        if command > 5 and (command not in self.status['capabilities'] or 'SKILL_START' not in self.features):
            self.event('UNSUPPORTED', event='rejected', **fields)
            return False
        self.pending_mode_intent = self.authority[:3]
        self._ensure_mode('SKILL', now)
        if command <= 4:
            self.event('', event='mode_requested', **fields)
            return True
        if self.deferred_skill or self.active_skill or self.status['skill_state'] in ('accepted', 'running'):
            self.event('SKILL_BUSY', event='rejected', **fields)
            return False
        self.deferred_skill = dict(fields, command_id=command)
        self._try_skill(now)
        return True

    def _try_skill(self, now):
        if self.pending_mode_intent and self.authority and self.pending_mode_intent == self.authority[:3]:
            if self._ensure_mode('SKILL', now):
                self.pending_mode_intent = None
        if self.hold_pending:
            return
        if (self.deferred_skill and self._safe_motion('SKILL', now)
                and self.status['mode'] == 'SKILL'):
            self.active_skill = dict(self.deferred_skill)
            self._enqueue(T.SKILL_START, dict(self.active_skill, owner='SKILL',
                                            mission_id=self.authority[1], generation=self.authority[2],
                                            valid_for_ms=max(1, int(self.config['command_timeout']*1000))), now)
            self.deferred_skill = None

    def _enqueue(self, kind, payload, now, safety=False):
        if kind == T.HOLD:
            self.hold_pending = True
            self.hold_metadata = dict(payload)
        if len(self.queue) + len(self.pending) >= 128 or self.sequence >= 0xffffffff:
            self.event('TX_LIMIT_OR_SEQUENCE_EXHAUSTED', event='rejected')
            self.disconnect('TX_LIMIT_OR_SEQUENCE_EXHAUSTED', now)
            return
        self.sequence += 1
        item = dict(kind=kind, payload=dict(payload), sequence=self.sequence,
                    packet=encode_frame(kind, self.sequence, self.session, payload),
                    token=None if safety or kind == T.HELLO else self.authority[:3],
                    session=self.session, retries=0, sent_at=None, created_at=now)
        self.queue.append(item)

    def outgoing(self, now, max_packets=None):
        """Worker alone drains immutable bytes; validates authority at write boundary."""
        result = []
        while self.queue and (max_packets is None or len(result) < max_packets):
            q = self.queue.popleft()
            if not self.connected or q['session'] != self.session:
                continue
            if q['token'] is not None:
                if now - q['created_at'] > self.config['command_timeout']:
                    self.event('COMMAND_QUEUE_STALE', event='rejected', command_type=int(q['kind']))
                    continue
                if (not self.authority or q['token'] != self.authority[:3]
                        or not self._safe_motion(q['token'][0], now)):
                    continue
            q['sent_at'] = now
            self.pending[q['sequence']] = q
            result.append(q['packet'])
        return result

    def receive(self, raw, now, stamp_ns):
        prior_errors = self.parser.errors
        for frame in self.parser.feed(raw):
            if frame.session != self.session or frame.sequence <= self.received_sequence:
                self.event('OLD_SESSION_OR_DUPLICATE_SEQUENCE', event='ignored')
                continue
            self.received_sequence = frame.sequence
            if frame.kind == T.ACK:
                self._ack(frame.payload, now)
            elif frame.kind == T.STATUS and self.handshake:
                try:
                    status = validate_status(frame.payload)
                except ValueError as exc:
                    self.error_code = 'INVALID_STATUS'
                    self.status = None
                    self._fence('INVALID_STATUS', now)
                    self.event('INVALID_STATUS', event='protocol_error', detail=str(exc))
                    continue
                self.status, self.status_at, self.status_stamp_ns = status, now, stamp_ns
                hold_correlated = (not self.hold_metadata or not self.hold_metadata.get('request_id') or
                                   all(status.get(k) == self.hold_metadata.get(k) for k in
                                       ('request_id', 'runner_session', 'obstacle_id', 'step_index')))
                if (self.hold_pending and hold_correlated and status['mode'] == 'HOLD'
                        and status['stopped'] and status['safe_stance'] and status['control_releasable']
                        and not status['nav_active'] and not status['skill_active']):
                    self.hold_pending = False
                    self.hold_metadata = None
                if status['skill_state'] in ('succeeded', 'failed', 'canceled'):
                    def matches(request):
                        return request and all(status.get(k) == request.get(k) for k in
                                               ('request_id', 'runner_session', 'obstacle_id', 'step_index'))
                    if matches(self.active_skill) or (matches(self.ending_skill_request) and
                                                     not status['skill_active'] and status['stopped']):
                        self.active_skill = None
                        self.ending_skill_request = None
                self.gateway_sequence += 1
                self.error_code = (status['fault_code'] or 'ESTOP' if status['estop'] else status['fault_code'])
                if not status['known']:
                    self.error_code = 'CONTROLLER_STATE_UNKNOWN'
                if status['fault_code'] or status['estop'] or not status['known']:
                    self._fence(self.error_code or 'CONTROLLER_STATE_UNKNOWN', now)
                elif not status['human_enable']:
                    self.error_code = 'HUMAN_ENABLE_REQUIRED'
                    self._fence('HUMAN_ENABLE_REQUIRED', now)
                self._try_skill(now)
            elif frame.kind == T.FAULT:
                self.error_code = str(frame.payload.get('code', 'CONTROLLER_FAULT'))
                self.status = None
                self._fence(self.error_code, now)
                self.event(self.error_code, event='fault')
        if self.parser.errors != prior_errors:
            self.event('PROTOCOL_FRAME_ERROR', event='protocol_error', count=self.parser.errors)

    def _ack(self, payload, now):
        seq = payload.get('acked_sequence')
        if type(seq) is not int or type(payload.get('accepted')) is not bool:
            self.event('INVALID_ACK', event='protocol_error')
            return
        item = self.pending.get(seq)
        if item is None or type(payload.get('acked_type')) is not int or payload['acked_type'] != int(item['kind']):
            self.event('UNMATCHED_ACK', event='ignored')
            return
        del self.pending[seq]
        accepted = payload['accepted']
        if item['kind'] == T.HELLO:
            features = payload.get('features')
            if (not accepted or type(payload.get('protocol_version')) is not int or payload['protocol_version'] != 1
                    or not isinstance(features, list) or any(not isinstance(x, str) for x in features)
                    or not BASE_FEATURES <= set(features)):
                self.error_code = 'HANDSHAKE_UNSUPPORTED'
                self.event(self.error_code, event='rejected')
                return
            self.features = frozenset(features)
            self.handshake = True
            self.error_code = 'STATUS_PENDING'
        metadata = {k: v for k, v in item['payload'].items()
                    if k in ('request_id', 'runner_session', 'obstacle_id', 'step_index')}
        self.event('' if accepted else str(payload.get('reason', 'CONTROLLER_REJECTED')),
                   event='ack', accepted=accepted, command_type=int(item['kind']), **metadata)
        if not accepted and item['kind'] != T.HELLO:
            self._fence('CONTROLLER_REJECTED', now, send_hold=item['kind'] != T.HOLD)
            if item['kind'] == T.HOLD:
                self.handshake = False
                self.error_code = 'HOLD_UNSUPPORTED_OR_REJECTED'

    def tick(self, now):
        if not self.connected:
            return
        if not self.handshake and now - self.connected_at > self.config['handshake_timeout']:
            self.disconnect('HANDSHAKE_TIMEOUT', now)
            return
        if self.handshake and (not self.status or now-self.status_at > self.config['status_timeout']):
            if self.error_code != 'STATUS_STALE':
                self.error_code = 'STATUS_STALE'
                self._fence('STATUS_STALE', now)
        if self.authority and self.authority[0] != 'SAFE_HOLD' and (now >= self.authority[3]
                or now-self.authority_at > self.config['authority_timeout']):
            self.invalidate_authority('AUTHORITY_TIMEOUT', now)
        if self.last_velocity_at is not None and now-self.last_velocity_at > self.config['command_timeout']:
            self._fence('COMMAND_TIMEOUT', now)
            self.event('COMMAND_TIMEOUT', event='timeout')
        for seq, item in list(self.pending.items()):
            if now-item['sent_at'] < self.config['ack_timeout']:
                continue
            if item['retries'] >= self.config['max_ack_retries']:
                self.pending.pop(seq, None)
                metadata = {k: v for k, v in item['payload'].items()
                            if k in ('request_id', 'runner_session', 'obstacle_id', 'step_index')}
                self.event('ACK_TIMEOUT', event='timeout', command_type=int(item['kind']), **metadata)
                if item['kind'] == T.HELLO:
                    self.disconnect('HANDSHAKE_TIMEOUT', now)
                elif item['kind'] != T.HOLD:
                    self._fence('ACK_TIMEOUT', now)
                break
            item['retries'] += 1
            item['sent_at'] = now
            self.queue.append(item)
        if self.nav_active:
            self._ensure_mode('NAVIGATION', now)
        self._try_skill(now)

    def snapshot(self, now, stamp_ns):
        valid = self._fresh_status(now) and not self.error_code
        if not valid:
            self.gateway_sequence += 1
        return dict(connected=self.connected, handshake=self.handshake, session=self.session,
                    sequence=self.received_sequence, gateway_sequence=self.gateway_sequence,
                    stamp_ns=self.status_stamp_ns if valid else stamp_ns,
                    status=dict(self.status) if valid else None,
                    error_code='' if valid else self.error_code or 'STATUS_STALE')
