"""Atomic GICP evidence adapter. Headerless legacy topics are diagnostics only."""
import copy
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from go2_obstacle_interfaces.msg import LocalizationEvidence
from .localization_core import Evidence, EvidenceHealth


class LocalizationAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__("localization_evidence_adapter", **kwargs)
        for key, value in {
            "input_topic": "/gicp/evidence", "output_topic": "/localization/evidence",
            "health_service": "/mission/localization_ok", "map_frame": "map",
            "body_frame": "livox_frame", "source_id": "", "evidence_max_age": 4.0,
            "clock_timeout": 1.0, "future_tolerance": 0.1,
        }.items():
            self.declare_parameter(key, value)
        param = lambda key: self.get_parameter(key).value
        sim_time = self.has_parameter("use_sim_time") and param("use_sim_time")
        self.health = EvidenceHealth(param("evidence_max_age"), param("map_frame"),
                                     param("body_frame"), param("source_id"),
                                     sim_time, param("clock_timeout"), param("future_tolerance"))
        self.publisher = self.create_publisher(LocalizationEvidence, param("output_topic"), 10)
        self.subscription = self.create_subscription(
            LocalizationEvidence, param("input_topic"), self._observe, 10)
        self.service = self.create_service(Trigger, param("health_service"), self._respond)
        self.last_output = None
        # ROS timers stop with /clock; expiry must continue on the monotonic clock.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self._tick, clock=self.steady_clock)

    def _observe(self, message):
        accepted = self.health.observe(Evidence.from_message(message),
                                       self.get_clock().now().nanoseconds)
        output = copy.deepcopy(message)
        output.accepted = accepted
        if not accepted:
            output.reason = self.health.reason
        self.publisher.publish(output)
        self.last_output = output

    def _tick(self):
        valid = self.health.healthy(self.get_clock().now().nanoseconds)
        if not valid and self.last_output is not None and self.last_output.accepted:
            output = copy.deepcopy(self.last_output)
            output.accepted = False
            output.reason = self.health.reason
            # Replaying the identity as an invalidation intentionally cannot
            # become new accepted evidence or consume a producer sequence.
            self.publisher.publish(output)
            self.last_output = output

    def _respond(self, _request, response):
        response.success = self.health.healthy(self.get_clock().now().nanoseconds)
        response.message = self.health.reason
        return response


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LocalizationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
