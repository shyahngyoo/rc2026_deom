"""ROS-free atomic localization evidence and freshness rules.

No headerless Bool/Pose pairing is accepted. A fault discards accepted evidence;
recovery needs a subsequent accepted sequence measured after the fault boundary.
"""
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class Evidence:
    stamp_ns: int
    frame: str
    body_frame: str
    source_id: str
    epoch: int
    sequence: int
    accepted: bool
    covariance_known: bool
    position: tuple
    quaternion: tuple
    covariance: tuple
    reason: str = ""

    @classmethod
    def from_message(cls, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        stamp = message.header.stamp
        stamp_ns = (stamp.sec * 1000000000 + stamp.nanosec
                    if stamp.sec >= 0 and 0 <= stamp.nanosec < 1000000000 else -1)
        return cls(stamp_ns,
                   message.header.frame_id, message.body_frame, message.source_id,
                   message.epoch, message.sequence, message.accepted,
                   message.covariance_known, (p.x, p.y, p.z),
                   (q.x, q.y, q.z, q.w), tuple(message.pose.covariance), message.reason)


class EvidenceHealth:
    def __init__(self, max_age=4.0, frame="map", body_frame="livox_frame",
                 source_id="", use_sim_time=False, clock_timeout=1.0,
                 future_tolerance=0.1, wall_clock=time.monotonic):
        for name, value in (("max_age", max_age), ("clock_timeout", clock_timeout)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
        if not math.isfinite(future_tolerance) or future_tolerance < 0 or not frame or not body_frame:
            raise ValueError("invalid frame/future tolerance")
        self.max_age, self.frame, self.body_frame = max_age, frame, body_frame
        self.source_id = source_id
        self.use_sim_time, self.clock_timeout = use_sim_time, clock_timeout
        self.future_tolerance, self.wall_clock = future_tolerance, wall_clock
        self.epoch = None
        self.sequence = 0
        self.stamp_ns = None
        self.received_wall = None
        self.last_measurement_ns = 0
        self.fault_boundary_ns = 0
        self.minimum_epoch = 0
        self.last_ros_ns = None
        self.last_clock_progress_wall = None
        self.reason = "NO_ACCEPTED_EVIDENCE"
        self.phase_stamp_ns = 0
        self.phase_identity = None

    def _fault(self, reason, ros_now_ns, advance_boundary=True):
        was_healthy = self.stamp_ns is not None
        self.stamp_ns = None
        self.received_wall = None
        if advance_boundary or was_healthy:
            self.fault_boundary_ns = max(self.fault_boundary_ns, ros_now_ns)
        self.reason = reason
        return False

    def check_clock(self, ros_now_ns, wall_now=None):
        wall_now = self.wall_clock() if wall_now is None else wall_now
        if ros_now_ns <= 0:
            return self._fault("CLOCK_UNINITIALIZED", ros_now_ns)
        if self.last_ros_ns is not None and ros_now_ns < self.last_ros_ns:
            self.minimum_epoch = max(self.minimum_epoch, (self.epoch or 0) + 1)
            self.fault_boundary_ns = ros_now_ns
            self.phase_stamp_ns = ros_now_ns
            self.last_measurement_ns = 0
            self.last_ros_ns = ros_now_ns
            self.last_clock_progress_wall = wall_now
            return self._fault("CLOCK_ROLLBACK", ros_now_ns)
        if self.last_ros_ns is None or ros_now_ns > self.last_ros_ns:
            self.last_clock_progress_wall = wall_now
        self.last_ros_ns = ros_now_ns
        if self.use_sim_time and wall_now - self.last_clock_progress_wall > self.clock_timeout:
            return self._fault("CLOCK_STALLED", ros_now_ns)
        return True

    def observe(self, evidence, ros_now_ns, wall_now=None):
        wall_now = self.wall_clock() if wall_now is None else wall_now
        clock_ok = self.check_clock(ros_now_ns, wall_now)
        if not evidence.source_id or evidence.epoch < 1 or evidence.sequence < 1:
            return self._fault("INVALID_IDENTITY", ros_now_ns)
        if not self.source_id:
            self.source_id = evidence.source_id
        if evidence.source_id != self.source_id:
            return self._fault("SOURCE_CONFLICT", ros_now_ns)
        if evidence.epoch < self.minimum_epoch or (self.epoch is not None and evidence.epoch < self.epoch):
            return self._fault("OLD_EPOCH", ros_now_ns, advance_boundary=False)
        if self.epoch == evidence.epoch and evidence.sequence <= self.sequence:
            return self._fault("SEQUENCE_NOT_INCREASING", ros_now_ns, advance_boundary=False)
        if self.epoch is not None and evidence.epoch > self.epoch:
            self.fault_boundary_ns = max(self.fault_boundary_ns, self.last_measurement_ns)
            self.last_measurement_ns = 0
            self.stamp_ns, self.received_wall = None, None
            self.reason = "EPOCH_CHANGED"
        self.epoch, self.sequence = evidence.epoch, evidence.sequence
        if not clock_ok:
            return False
        if not evidence.accepted:
            return self._fault("REJECTED:" + evidence.reason, ros_now_ns)
        if evidence.frame != self.frame or evidence.body_frame != self.body_frame:
            return self._fault("FRAME_MISMATCH", ros_now_ns)
        values = (*evidence.position, *evidence.quaternion, *evidence.covariance)
        if len(evidence.position) != 3 or len(evidence.quaternion) != 4 or len(evidence.covariance) != 36 or not all(math.isfinite(x) for x in values):
            return self._fault("NONFINITE_POSE_OR_COVARIANCE", ros_now_ns)
        if abs(sum(x*x for x in evidence.quaternion) - 1.0) > 0.05:
            return self._fault("INVALID_QUATERNION", ros_now_ns)
        if evidence.covariance_known and any(evidence.covariance[i] < 0 for i in (0, 7, 14, 21, 28, 35)):
            return self._fault("NEGATIVE_COVARIANCE", ros_now_ns)
        age = (ros_now_ns - evidence.stamp_ns) / 1e9
        if evidence.stamp_ns <= 0 or not -self.future_tolerance <= age <= self.max_age:
            return self._fault("MEASUREMENT_STALE_OR_FUTURE", ros_now_ns)
        if evidence.stamp_ns <= self.last_measurement_ns:
            return self._fault("MEASUREMENT_NOT_INCREASING", ros_now_ns, advance_boundary=False)
        if evidence.stamp_ns < self.fault_boundary_ns:
            return self._fault("PRE_FAULT_MEASUREMENT", ros_now_ns, advance_boundary=False)
        self.last_measurement_ns = evidence.stamp_ns
        self.stamp_ns, self.received_wall = evidence.stamp_ns, wall_now
        self.reason = "ACCEPTED"
        return True

    def healthy(self, ros_now_ns, wall_now=None):
        wall_now = self.wall_clock() if wall_now is None else wall_now
        if not self.check_clock(ros_now_ns, wall_now) or self.stamp_ns is None:
            return False
        age = (ros_now_ns - self.stamp_ns) / 1e9
        if not -self.future_tolerance <= age <= self.max_age or wall_now - self.received_wall > self.max_age:
            return self._fault("EVIDENCE_EXPIRED", ros_now_ns)
        return True

    def require_new(self, ros_now_ns):
        self.phase_stamp_ns = ros_now_ns
        self.phase_identity = (self.source_id, self.epoch, self.sequence)

    def phase_fresh(self, ros_now_ns, wall_now=None):
        if not self.healthy(ros_now_ns, wall_now) or self.stamp_ns < self.phase_stamp_ns:
            return False
        if self.phase_identity is None:
            return True
        source, epoch, sequence = self.phase_identity
        return (not source or self.source_id == source) and (epoch is None or self.epoch > epoch or
                                            (self.epoch == epoch and self.sequence > sequence))
