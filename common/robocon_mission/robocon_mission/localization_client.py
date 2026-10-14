"""Shared TF buffer plus fresh external health; health is not relocalization."""
import math
import time
from typing import Callable, Optional
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from .async_service import AsyncServiceRunner
from .end_pose_verifier import PoseSample
from .localization_health import AcceptedPoseHealth
from .localization_evidence import Evidence, EvidenceHealth
from go2_obstacle_interfaces.msg import LocalizationEvidence


class LocalizationClient:
    def __init__(self, node, service_name, frame, base_frame, tf_timeout,
                 clock_timeout, use_sim_time, health_timeout=1.0,
                 accepted_pose_topic="", accepted_pose_max_age=4.0,
                 accepted_pose_type="covariance", accepted_pose_body_frame="livox_frame",
                 accepted_pose_source_id=""):
        self.node = node
        self.client = node.create_client(Trigger, service_name)
        self.runner = AsyncServiceRunner(self.client)
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.frame, self.base_frame = frame, base_frame
        self.tf_timeout, self.clock_timeout = tf_timeout, clock_timeout
        self.use_sim_time = use_sim_time
        self.health_timeout = health_timeout
        self.last_clock_wall = None
        self.last_clock_stamp_ns = None
        self.pending = None
        self.health_wall = 0.0
        self.health_ok = False
        self.health_reason = "health service not ready"
        self.next_poll = 0.0
        self.minimum_stamp_ns = 0
        if accepted_pose_type not in ("covariance", "evidence"):
            raise ValueError("accepted_pose_type must be covariance or evidence")
        self.structured_evidence = accepted_pose_type == "evidence"
        self.accepted = None
        self.accepted_minimum_stamp_ns = 0
        if accepted_pose_topic:
            if getattr(self, "structured_evidence", False):
                self.accepted = EvidenceHealth(accepted_pose_max_age, frame,
                    accepted_pose_body_frame, accepted_pose_source_id,
                    use_sim_time, clock_timeout)
                self.accepted_sub = node.create_subscription(
                    LocalizationEvidence, accepted_pose_topic, self._evidence, 10)
            else:
                self.accepted = AcceptedPoseHealth(accepted_pose_max_age)
                self.accepted_sub = node.create_subscription(
                    PoseWithCovarianceStamped, accepted_pose_topic, self.accepted.observe, 10)
        if use_sim_time:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.clock_sub = node.create_subscription(Clock, "/clock", self._clock, qos)

    def _evidence(self, message):
        self.accepted.observe(Evidence.from_message(message),
                              self.node.get_clock().now().nanoseconds)

    def _clock(self, message):
        stamp_ns = message.clock.sec * 1000000000 + message.clock.nanosec
        if self.last_clock_stamp_ns is None or stamp_ns > self.last_clock_stamp_ns:
            self.last_clock_wall = time.monotonic()
        elif stamp_ns < self.last_clock_stamp_ns:
            self.last_clock_wall = time.monotonic()
            self.minimum_stamp_ns = stamp_ns
            self.accepted_minimum_stamp_ns = stamp_ns
        self.last_clock_stamp_ns = stamp_ns
        if getattr(self, "structured_evidence", False) and self.accepted is not None:
            self.accepted.check_clock(stamp_ns)

    def sample(self) -> Optional[PoseSample]:
        if self.use_sim_time and (self.last_clock_wall is None or
                time.monotonic() - self.last_clock_wall > self.clock_timeout):
            return None
        try:
            tf = self.buffer.lookup_transform(self.frame, self.base_frame, Time())
            stamp = tf.header.stamp.sec * 1000000000 + tf.header.stamp.nanosec
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp <= 0 or stamp < self.minimum_stamp_ns or not -0.1 <= age <= self.tf_timeout:
                return None
            p, q = tf.transform.translation, tf.transform.rotation
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                return None
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if abs(norm - 1.0) > 0.05:
                return None
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return PoseSample((p.x, p.y, yaw), stamp)
        except Exception:
            return None

    def tf_fresh(self) -> bool:
        return self.sample() is not None

    def healthy(self) -> bool:
        accepted_fresh = self.accepted_fresh()
        return (self.health_ok and time.monotonic() - self.health_wall <=
                self.health_timeout and self.tf_fresh() and accepted_fresh)

    def accepted_fresh(self) -> bool:
        if getattr(self, "structured_evidence", False) and self.accepted is not None:
            return self.accepted.phase_fresh(self.node.get_clock().now().nanoseconds)
        return (self.accepted is None or
                (self.accepted.healthy(self.node.get_clock().now().nanoseconds) and
                 self.accepted.stamp_ns >= self.accepted_minimum_stamp_ns))

    def require_new_accepted_pose(self) -> None:
        """Do not let a rebroadcast TF reuse a pre-handoff global correction."""
        if self.accepted is not None:
            self.accepted_minimum_stamp_ns = self.node.get_clock().now().nanoseconds
            if getattr(self, "structured_evidence", False):
                self.accepted.require_new(self.accepted_minimum_stamp_ns)

    def invalidate(self, require_new_tf: bool = False) -> None:
        self.runner.cancel()
        self.health_ok, self.health_wall, self.next_poll = False, 0.0, 0.0
        if require_new_tf:
            self.minimum_stamp_ns = self.node.get_clock().now().nanoseconds
            self.require_new_accepted_pose()

    def check(self, token: int, timeout: float, done: Callable) -> None:
        if self.pending is not None:
            raise RuntimeError("localization request already pending")
        self.invalidate()
        self.pending = dict(token=token, done=done, deadline=time.monotonic() + timeout)

    def _finish(self, op, outcome):
        if self.pending is not op:
            return
        self.pending = None
        op["done"](op["token"], outcome, True, None)

    def tick(self) -> None:
        self.runner.tick()
        now = time.monotonic()
        op = self.pending
        if op is not None:
            if self.healthy():
                self._finish(op, "succeeded")
            elif now >= op["deadline"]:
                self._finish(op, f"health_timeout:{self.health_reason}")
        if self.runner.pending is None and now >= self.next_poll:
            self.next_poll = now + min(0.2, self.health_timeout / 3)
            op_at_start = self.pending
            def observed(_token, outcome, _terminal, result):
                self.health_ok = outcome == "succeeded" and bool(result.success)
                self.health_wall = time.monotonic()
                self.health_reason = (getattr(result, "message", "") or outcome)
                if not self.health_ok and self.structured_evidence and self.accepted is not None:
                    self.accepted._fault("HEALTH_SERVICE:" + self.health_reason,
                                         self.node.get_clock().now().nanoseconds)
                if self.pending is op_at_start and op_at_start is not None:
                    if self.healthy():
                        self._finish(op_at_start, "succeeded")
                    elif result is not None and not result.success:
                        self._finish(op_at_start, "health_failed")
            self.runner.start(0, Trigger.Request(), self.health_timeout, observed,
                              lambda r: r is not None)

    def cancel(self) -> None:
        op, self.pending = self.pending, None
        self.runner.cancel()
        if op:
            op["done"](op["token"], "canceled", True, None)

