"""ROS-free, fail-closed single-owner command gate. Clocks are injected."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Switch:
    owner: str
    expected_owner: str
    mission_id: str
    request_id: str
    lease_duration: float

@dataclass(frozen=True)
class Decision:
    accepted: bool
    owner: str
    mission_id: str
    request_id: str
    reason: str

class Arbiter:
    OWNERS = ('SAFE_HOLD', 'NAVIGATION', 'SKILL')

    def __init__(self, command_timeout=0.5, max_command_age=0.3,
                 max_linear=0.5, max_angular=1.0, max_cached_requests=2048,
                 generation_counter=None):
        if not all(math.isfinite(x) and x > 0 for x in
                   (command_timeout, max_command_age, max_linear, max_angular)):
            raise ValueError('invalid arbitration limits')
        self.command_timeout, self.max_command_age = command_timeout, max_command_age
        self.max_linear, self.max_angular = max_linear, max_angular
        self.max_cached_requests = max_cached_requests
        self.owner, self.mission_id = 'SAFE_HOLD', ''
        self.generation_counter = generation_counter
        self.generation = generation_counter.next() if generation_counter else 1
        self.granted_ros = 0
        self.deadline, self.last_command_wall = 0.0, 0.0
        self.last_stamp, self.command = 0, None
        self.reason, self.cache = 'INITIAL_SAFE_HOLD', {}
        self.last_ros = None

    def hold(self, reason):
        self.generation = self.generation_counter.next() if self.generation_counter else self.generation + 1
        self.owner, self.mission_id = 'SAFE_HOLD', ''
        self.deadline, self.last_stamp, self.command = 0.0, 0, None
        self.reason = reason

    def tick(self, wall, ros, state_fresh=True, source_active=False):
        if self.last_ros is not None and ros < self.last_ros:
            self.hold('CLOCK_ROLLBACK')
        self.last_ros = ros
        if self.owner == 'SAFE_HOLD' and self.deadline > 0 and wall >= self.deadline:
            self.hold('HOLD_LEASE_EXPIRED')
        if self.owner != 'SAFE_HOLD':
            if not state_fresh:
                self.hold('STATE_STALE_OR_UNSAFE')
            elif wall >= self.deadline:
                self.hold('LEASE_EXPIRED')
            elif source_active and wall - self.last_command_wall > self.command_timeout:
                self.hold('COMMAND_HEARTBEAT_TIMEOUT')
        if self.command is not None and (wall - self.last_command_wall > self.command_timeout or
                not 0 <= (ros-self.last_stamp)/1e9 <= self.max_command_age):
            self.command = None
            if source_active:
                self.hold('COMMAND_STALE')

    def switch(self, request, wall, ros, safe_to_switch, state_fresh=True):
        self.tick(wall, ros, state_fresh)
        key = (request.mission_id, request.request_id)
        if key in self.cache:
            prior, result, granted_generation = self.cache[key]
            if prior != request:
                self.hold('REQUEST_ID_CONFLICT')
                return Decision(False, self.owner, self.mission_id, request.request_id,
                                'REQUEST_ID_CONFLICT')
            # An identical replay never renews a lease or restores old ownership.
            if (not result.accepted or granted_generation != self.generation or
                result.owner != self.owner or result.mission_id != self.mission_id):
                return Decision(False,self.owner,self.mission_id,request.request_id,
                                'REPLAY_NO_LONGER_CURRENT')
            return result
        reason = ''
        if (request.owner not in self.OWNERS or not request.mission_id or not request.request_id or
            not math.isfinite(request.lease_duration) or not 0.1 <= request.lease_duration <= 10.0):
            reason = 'INVALID_REQUEST'
        elif len(self.cache) >= self.max_cached_requests:
            self.hold('REQUEST_CACHE_LIMIT')
            reason = 'REQUEST_CACHE_LIMIT'
        elif self.mission_id and request.mission_id != self.mission_id:
            reason = 'SESSION_MISMATCH'
        elif request.expected_owner != self.owner:
            # The current session may fence its own unknown owner, but cannot
            # acquire motion on that basis.
            if not (request.owner == 'SAFE_HOLD' and (request.mission_id == self.mission_id or
                    (self.owner == 'SAFE_HOLD' and not self.mission_id))):
                reason = 'EXPECTED_OWNER_MISMATCH'
        if not reason and request.owner != 'SAFE_HOLD':
            if not state_fresh:
                reason = 'STATE_UNAVAILABLE'
            elif request.owner != self.owner and not safe_to_switch:
                reason = 'STOP_TERMINAL_OR_SUPPORT_UNCONFIRMED'
        if not reason:
            if request.owner != self.owner or request.mission_id != self.mission_id:
                self.generation = self.generation_counter.next() if self.generation_counter else self.generation + 1
                self.granted_ros, self.last_stamp = ros, 0
                self.command = None
                self.last_command_wall = wall
            self.owner, self.mission_id = request.owner, request.mission_id
            self.deadline = wall + request.lease_duration
            self.reason = 'LEASE_GRANTED' if self.owner != 'SAFE_HOLD' else 'SAFE_HOLD_FENCED'
            if self.owner == 'SAFE_HOLD':
                self.command = None
        result = Decision(not reason, self.owner, self.mission_id, request.request_id,
                          reason or self.reason)
        if len(self.cache) < self.max_cached_requests:
            self.cache[key] = (request, result, self.generation)
        return result

    def receive(self, source, owner, mission_id, generation, stamp, twist,
                wall, ros, state_fresh=True, source_active=True):
        self.tick(wall, ros, state_fresh, source_active)
        reason = ''
        if self.owner == 'SAFE_HOLD' or not state_fresh:
            reason = 'NO_MOTION_AUTHORITY'
        elif source != self.owner or owner != source:
            reason = 'UNAUTHORIZED_SOURCE'
        elif mission_id != self.mission_id or generation != self.generation:
            reason = 'OLD_SESSION_OR_GENERATION'
        elif stamp <= self.granted_ros or stamp <= self.last_stamp:
            reason = 'OLD_COMMAND_TIMESTAMP'
        elif not 0 <= (ros-stamp)/1e9 <= self.max_command_age:
            reason = 'STALE_OR_FUTURE_COMMAND'
        elif len(twist) != 6 or not all(math.isfinite(v) for v in twist):
            reason = 'NONFINITE_COMMAND'
        elif any(abs(twist[i]) > 1e-8 for i in (2, 3, 4)):
            reason = 'NONPLANAR_COMMAND'
        elif math.hypot(twist[0], twist[1]) > self.max_linear or abs(twist[5]) > self.max_angular:
            reason = 'COMMAND_LIMIT_EXCEEDED'
        elif not source_active and any(abs(v) > 1e-8 for v in twist):
            reason = 'EXECUTOR_NOT_ACTIVE'
        if reason:
            self.reason = reason
            return False
        self.last_stamp, self.last_command_wall = stamp, wall
        self.command, self.reason = tuple(twist), 'COMMAND_ACCEPTED'
        return True

    def output(self, wall, ros, state_fresh=True, source_active=False):
        self.tick(wall, ros, state_fresh, source_active)
        if self.owner == 'SAFE_HOLD' or not source_active or self.command is None:
            return (0.0,)*6
        return self.command
