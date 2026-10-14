"""Simulation ICP health service based on actual accepted poses, not a mock."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger


class AcceptedPoseHealth:
    def __init__(self, max_age):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError("pose_max_age must be finite and positive")
        self.max_age = max_age
        self.stamp_ns = None
        self.received_wall = None

    def observe(self, message):
        stamp = message.header.stamp
        pose = message.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        covariance = message.pose.covariance
        quaternion_norm = sum(v*v for v in values[3:])
        if message.header.frame_id != "map" or stamp_ns <= 0 or \
                not all(math.isfinite(value) for value in (*values, *covariance)) or \
                abs(quaternion_norm - 1.0) > 0.1 or \
                any(covariance[i] < 0 for i in (0, 7, 14, 21, 28, 35)):
            self.stamp_ns = None
            self.received_wall = None
            return
        self.stamp_ns = stamp_ns
        self.received_wall = time.monotonic()

    def healthy(self, ros_now_ns):
        if self.stamp_ns is None:
            return False
        ros_age = (ros_now_ns - self.stamp_ns) / 1e9
        return (-0.1 <= ros_age <= self.max_age and
                time.monotonic() - self.received_wall <= self.max_age)


class LocalizationHealth(Node):
    def __init__(self):
        super().__init__("simulation_localization_health")
        self.declare_parameter("pose_topic", "/localization/pose")
        self.declare_parameter("pose_max_age", 4.0)
        self.health = AcceptedPoseHealth(self.get_parameter("pose_max_age").value)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value,
            self.health.observe, 10,
        )
        self.service = self.create_service(
            Trigger, "/mission/localization_ok", self._respond,
        )

    def _respond(self, _request, response):
        response.success = self.health.healthy(self.get_clock().now().nanoseconds)
        response.message = ("fresh accepted ICP pose" if response.success
                            else "accepted ICP pose missing, invalid or stale")
        return response


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LocalizationHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
