"""Real ROS evidence/health/TF test with synthetic odometry; no hardware."""
import os
import time
import pytest
os.environ.setdefault('ROS_DOMAIN_ID', '178')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from nav_msgs.msg import Odometry
from go2_obstacle_interfaces.msg import LocalizationEvidence
from tf2_ros import Buffer, TransformListener
from go2_mission_backends.odometry_localization_node import OdometryLocalizationNode
from go2_mission_backends.localization_adapter import LocalizationAdapter
from go2_mission_backends.robot_state_adapter import RobotStateAdapter


def test_fixed_alignment_health_tf_and_latched_outage():
    context = Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    def params(**values):
        return [Parameter(k, value=v) for k, v in values.items()]
    source = OdometryLocalizationNode(context=context, parameter_overrides=params(
        alignment_confirmed=True, odom_in_map=[2., 3., 0., 0., 0., 0.]))
    adapter = LocalizationAdapter(context=context, parameter_overrides=params(
        input_topic='/odometry/evidence', evidence_max_age=.5))
    state = RobotStateAdapter(context=context)
    probe = Node('odometry_only_test', context=context)
    nodes = [source, adapter, state, probe]
    for node in nodes:
        executor.add_node(node)
    publisher = probe.create_publisher(Odometry, '/Odometry', 10)
    evidence = []
    probe.create_subscription(LocalizationEvidence, '/localization/evidence', evidence.append, 10)
    buffer = Buffer(node=probe)
    listener = TransformListener(buffer, probe)
    sending = True
    def publish():
        if not sending:
            return
        msg = Odometry()
        msg.header.stamp = probe.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'livox_frame'
        msg.pose.pose.orientation.w = 1.
        msg.pose.pose.position.x = .25
        publisher.publish(msg)
    probe.create_timer(.03, publish)
    def wait(predicate, timeout=3.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        assert predicate(), [(e.accepted, e.reason) for e in evidence[-5:]]
    try:
        wait(lambda: evidence and evidence[-1].accepted and state.status['global_ready'])
        assert evidence[-1].source_id.startswith('odometry:')
        assert not evidence[-1].covariance_known
        assert evidence[-1].pose.pose.position.x == pytest.approx(2.25)
        wait(lambda: buffer.can_transform('map', 'base_footprint', Time()))
        tf = buffer.lookup_transform('map', 'base_footprint', Time())
        assert tf.transform.translation.x == pytest.approx(2.25)
        assert tf.transform.translation.y == pytest.approx(3.)
        assert adapter.health.healthy(probe.get_clock().now().nanoseconds)
        sending = False
        wait(lambda: evidence and not evidence[-1].accepted and bool(source.core.fault))
        assert not adapter.health.healthy(probe.get_clock().now().nanoseconds)
        count = len(evidence)
        sending = True
        wait(lambda: len(evidence) > count+3)
        assert all(not e.accepted for e in evidence[count:])
        assert probe.count_publishers('/gicp/evidence') == 0
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        context.shutdown()
