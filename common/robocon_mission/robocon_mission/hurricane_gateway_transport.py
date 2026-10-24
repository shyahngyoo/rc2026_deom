"""Hurricane transport over the shared STM32 gateway, without serial ownership.

Directional commands use MotionCommand and the existing arbiter.  Discrete
requests are correlation envelopes, never evidence of motion or completion.
Only a newer, matching gateway STATUS can confirm an existing runner step.
"""
import json
import math
import time
from threading import RLock
from types import SimpleNamespace
import uuid

from go2_obstacle_interfaces.msg import MotionAuthority, MotionCommand
from std_msgs.msg import String

from .hurricane_protocol import command_number


class GatewayTransport:
    """ROS transport implementing SkillRunner.send without opening a UART.

    The v1 firmware protocol is a draft.  Capabilities default to no supported
    discrete skills.  HOLD and CANCEL request safety, but never certify it.
    """

    def __init__(self, node, context, feedback, *, callback_group=None,
                 health_ready=lambda: False, status_topic="/stm32/status",
                 event_topic="/stm32/event", request_topic="/stm32/skill_request",
                 authority_topic="/motion/authority", command_topic="/motion/skill_command",
                 status_timeout=0.3, authority_timeout=0.5,
                 forward_speed=0.08, turn_speed=0.25, base_frame="base_footprint"):
        self.node, self.context, self.feedback = node, context, feedback
        self.health_ready = health_ready
        if not base_frame:
            raise ValueError("base_frame is required")
        self.base_frame = base_frame
        self.lock = getattr(node, "lock", RLock())
        for name, value in (("status_timeout", status_timeout),
                            ("authority_timeout", authority_timeout),
                            ("forward_speed", forward_speed), ("turn_speed", turn_speed)):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
            setattr(self, name, value)
        self.authority = None
        self.authority_received = 0.0
        self.authority_stamp = 0
        self.authority_generation = 0
        self.snapshot = None
        self.status_received = 0.0
        self.gateway_sequence = -1
        self.connection_session = None
        self.retired_sessions = set()
        self.gateway_stamp = 0
        self.pending = None
        self.direction = (0.0, 0.0)
        self.lease_key = None
        self.revoked = False
        self.closed = False
        self.requests = node.create_publisher(String, request_topic, 10)
        self.commands = node.create_publisher(MotionCommand, command_topic, 10)
        self.authority_sub = node.create_subscription(
            MotionAuthority, authority_topic, self._authority, 10,
            callback_group=callback_group)
        self.status_sub = node.create_subscription(
            String, status_topic, self._status, 10, callback_group=callback_group)
        self.event_sub = node.create_subscription(
            String, event_topic, self._event, 10, callback_group=callback_group)

    def _now_ns(self):
        return self.node.get_clock().now().nanoseconds

    @staticmethod
    def _key(authority):
        return (authority.mission_id, authority.generation) if authority is not None else None

    def _fresh_stamp(self, stamp, timeout):
        return (type(stamp) is int and stamp > 0
                and -0.1 <= (self._now_ns() - stamp) / 1e9 <= timeout)

    def _authority(self, message):
        with self.lock:
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if (not self._fresh_stamp(stamp, self.authority_timeout)
                    or message.generation < self.authority_generation
                    or stamp < self.authority_stamp):
                self.authority = None
                self.direction = (0.0, 0.0)
                return
            if stamp == self.authority_stamp and message.generation == self.authority_generation:
                return  # Replayed heartbeats do not renew the monotonic lease.
            self.authority_stamp, self.authority_generation = stamp, message.generation
            self.authority = message
            self.authority_received = time.monotonic()
            if self.lease_key is not None and self._key(message) != self.lease_key:
                self.direction = (0.0, 0.0)
                self.revoked = True

    def _authority_ready(self):
        msg = self.authority
        if msg is None:
            return False
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        context = self.context()
        return (msg.owner == "SKILL" and msg.healthy and bool(msg.mission_id)
                and msg.generation > 0 and math.isfinite(msg.lease_remaining)
                and msg.lease_remaining > time.monotonic() - self.authority_received
                and (not context.get("mission_id") or msg.mission_id == context["mission_id"])
                and (not context.get("generation") or msg.generation == context["generation"])
                and self._fresh_stamp(stamp, self.authority_timeout)
                and time.monotonic() - self.authority_received <= self.authority_timeout)

    def status_ready(self):
        with self.lock:
            if self.closed or self.snapshot is None:
                return False
            envelope, state = self.snapshot, self.snapshot.get("status", {})
            return (envelope.get("connected") is True and envelope.get("handshake") is True
                    and not envelope.get("error_code")
                    and self._fresh_stamp(envelope.get("stamp_ns"), self.status_timeout)
                    and time.monotonic() - self.status_received <= self.status_timeout
                    and state.get("known") is True
                    and state.get("mode") in ("HOLD", "SKILL")
                    and state.get("estop") is False and state.get("human_enable") is True
                    and not state.get("fault_code"))

    def ready(self):
        with self.lock:
            return not self.revoked and self._authority_ready() and self.status_ready()

    def available_skills(self, profiles):
        """Firmware capability evidence is independent from a current SKILL lease."""
        with self.lock:
            if not self.status_ready():
                return []
            raw = self.snapshot["status"].get("capabilities", [])
            capabilities = {item for item in raw if type(item) is int} if isinstance(raw, list) else set()
            available = []
            for skill_id, profile in profiles.items():
                required = {command_number(step["command"]) for step in profile["steps"]
                            if step["type"] in ("mode", "jump")}
                if profile.get("calibrated") is True and required <= capabilities:
                    available.append(skill_id)
            return sorted(available)

    def _envelope(self, command_id, op):
        context = self.context()
        authority = self.authority
        return dict(request_id=uuid.uuid4().hex, command_id=command_id, op=op,
                    runner_session=context.get("runner_session", ""),
                    obstacle_id=context.get("obstacle_id", ""),
                    step_index=context.get("step_index", -1), owner="SKILL",
                    mission_id=(authority.mission_id if authority is not None else
                                context.get("mission_id", "")),
                    generation=(authority.generation if authority is not None else 0))

    def _request(self, command_id, op):
        request = self._envelope(command_id, op)
        self.pending = dict(request, sent_ns=self._now_ns())
        self.requests.publish(String(data=json.dumps(request)))
        return request

    def _publish_velocity(self, direction):
        authority = self.authority
        if authority is None:
            return
        msg = MotionCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.owner = "SKILL"
        msg.mission_id, msg.generation = authority.mission_id, authority.generation
        msg.twist.linear.x, msg.twist.angular.z = direction
        # All unused axes, including vy, retain their typed zero defaults.
        self.commands.publish(msg)

    def send(self, command, repeat=1):
        with self.lock:
            if self.closed:
                raise RuntimeError("gateway transport closed")
            number = command_number(command)
            if type(repeat) is not int or not 1 <= repeat <= 100:
                raise ValueError("repeat must be an integer in [1, 100]")
            if number in (19, 20):
                self.direction = (0.0, 0.0)
                self._publish_velocity(self.direction)
                raise RuntimeError("UNSUPPORTED: this robot cannot move laterally")
            if (self.lease_key is None and self.context().get("active")
                    and self._authority_ready()):
                self.lease_key = self._key(self.authority)
            if number == 5:
                self.direction = (0.0, 0.0)
                self._publish_velocity(self.direction)
                # A safe HOLD request remains legal after authority loss.  Its
                # actual completion still requires a correlated measured STATUS.
                return self._request(number, "hold")
            if not self.context().get("active") or not self.ready() or not self.health_ready():
                self.direction = (0.0, 0.0)
                self._publish_velocity(self.direction)
                raise RuntimeError("SKILL gateway, authority or localization is not ready")
            if self.lease_key is None:
                self.lease_key = self._key(self.authority)
            self.direction = (0.0, 0.0)
            if number in (1, 2, 3, 4):
                self.direction = {1: (0.0, self.turn_speed), 2: (self.forward_speed, 0.0),
                                  3: (0.0, -self.turn_speed), 4: (-self.forward_speed, 0.0)}[number]
                # Gateway only requests SET_MODE SKILL for 1..4.  WALK remains
                # exclusively on the arbiter's authorized output path.
                result = self._request(number, "command")
                self._publish_velocity(self.direction)
                return result
            self._publish_velocity(self.direction)
            capabilities = self.snapshot.get("status", {}).get("capabilities", [])
            if not isinstance(capabilities, list) or not any(
                    type(item) is int and item == number for item in capabilities):
                raise RuntimeError("UNSUPPORTED: STM32 command_id=" + str(number))
            # Legacy repeat encoded UART copies. One correlated request avoids
            # accidentally executing a jump multiple times with the new protocol.
            return self._request(number, "command")

    def cancel(self):
        with self.lock:
            self.direction = (0.0, 0.0)
            self._publish_velocity(self.direction)
            if not self.closed:
                self._request(5, "cancel")

    def tick(self):
        with self.lock:
            if self.closed:
                return
            active = self.context().get("active")
            if not active:
                self.direction = (0.0, 0.0)
                self.pending = None
                self.lease_key = None
                self.revoked = False
            elif not self.ready() or not self.health_ready():
                self.direction = (0.0, 0.0)
            # Renew the producer heartbeat during both waypoint motion and
            # discrete skills, without replaying SKILL_START requests.
            if active or self.direction != (0.0, 0.0):
                self._publish_velocity(self.direction)

    def _matches(self, data):
        return (self.pending is not None
                and all(data.get(key) == self.pending[key]
                        for key in ("request_id", "runner_session", "step_index", "obstacle_id")))

    def _notify(self, state, stamp, *, failed=False, error=""):
        request = self.pending
        if request is None:
            return
        succeeded = state.get("skill_state") == "succeeded" and not failed
        final_release = (state.get("stopped") is True
                         and state.get("safe_stance") is True
                         and state.get("control_releasable") is True
                         and state.get("skill_active") is False)
        if request["op"] == "hold" and not final_release:
            succeeded = False
        self.feedback(SimpleNamespace(data=json.dumps({
            "stamp_ns": stamp, "session_id": request["runner_session"],
            "obstacle_id": request["obstacle_id"], "step_index": request["step_index"],
            "event": "failed" if failed else "completed" if succeeded else "progress",
            "completed": succeeded, "safe_stance": state.get("safe_stance") is True,
            "control_released": final_release, "known": state.get("known") is True,
            "error_code": error,
        })))

    def _status(self, message):
        with self.lock:
            try:
                data = json.loads(message.data)
                sequence, state = data["gateway_sequence"], data["status"]
                stamp = data["stamp_ns"]
                session = data.get("session")
                if (type(session) is not int or not 0 < session <= 0xffffffffffffffff
                        or session in self.retired_sessions
                        or type(sequence) is not int or sequence <= 0
                        or (session == self.connection_session and sequence <= self.gateway_sequence)
                        or not self._fresh_stamp(stamp, self.status_timeout)
                        or stamp < self.gateway_stamp):
                    return
                if session != self.connection_session:
                    if self.connection_session is not None:
                        self.retired_sessions.add(self.connection_session)
                        if self.context().get("active"):
                            self.revoked = True
                            self.direction = (0.0, 0.0)
                            self._publish_velocity(self.direction)
                    self.connection_session = session
                self.gateway_sequence = sequence
                self.gateway_stamp = stamp
                if (data.get("connected") is not True or data.get("handshake") is not True
                        or data.get("error_code") or not isinstance(state, dict)):
                    self.snapshot = None
                    self.status_received = 0.0
                    self.direction = (0.0, 0.0)
                    if self.context().get("active"):
                        self.revoked = True
                    self._publish_velocity(self.direction)
                    return
                self.snapshot, self.status_received = data, time.monotonic()
                if not self.ready():
                    self.direction = (0.0, 0.0)
                    return
                if (not self._matches(state) or stamp < self.pending["sent_ns"]
                        or self.pending["mission_id"] != self.authority.mission_id
                        or self.pending["generation"] != self.authority.generation):
                    return
                failed = state.get("skill_state") in ("failed", "canceled")
                self._notify(state, stamp, failed=failed,
                             error=state.get("fault_code", "") or ("SKILL_CANCELED" if
                                   state.get("skill_state") == "canceled" else ""))
            except (ValueError, TypeError, KeyError):
                self.node.get_logger().warn("Invalid STM32 gateway skill STATUS ignored")

    def _event(self, message):
        with self.lock:
            try:
                data = json.loads(message.data)
                if not self._matches(data):
                    return
                # ACK accepted/rejected is not completion. Explicit rejects may
                # fail a task immediately; only STATUS is allowed to succeed it.
                if data.get("error_code") or data.get("accepted") is False:
                    self.direction = (0.0, 0.0)
                    self._publish_velocity(self.direction)
                    self._notify({"known": True}, self._now_ns(), failed=True,
                                 error=data.get("error_code") or "STM32_REQUEST_REJECTED")
            except (ValueError, TypeError):
                self.node.get_logger().warn("Invalid STM32 gateway event ignored")

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.cancel()
            self.closed = True
