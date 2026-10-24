"""Publish fixed map->odom and explicitly odometric localization evidence."""
import copy
import time
import uuid
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster
from go2_obstacle_interfaces.msg import LocalizationEvidence
from .odometry_localization import OdometryLocalization
from .robot_state import rotation_quaternion
from .robot_state_adapter import fill_pose


class OdometryLocalizationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('odometry_localization', **kwargs)
        for name, value in dict(odom_in_map=[0.]*6, alignment_confirmed=False,
                max_input_age=.5, max_linear_speed=5., max_angular_speed=10.).items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        if not param('alignment_confirmed'):
            raise ValueError('explicit measured odom_in_map alignment is required')
        self.core = OdometryLocalization(param('odom_in_map'), param('max_input_age'),
                                        param('max_linear_speed'), param('max_angular_speed'))
        self.source_id = 'odometry:' + uuid.uuid4().hex
        self.sequence = 0
        self.last_output = None
        self.publisher = self.create_publisher(LocalizationEvidence, '/odometry/evidence', 10)
        self.broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, self.core.map_odom[:3, 3])
        q = rotation_quaternion(self.core.map_odom[:3, :3])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
        self.broadcaster.sendTransform(tf)
        self.create_subscription(Odometry, '/Odometry', self.observe, qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def observe(self, msg):
        evidence = LocalizationEvidence()
        evidence.header.stamp = msg.header.stamp
        evidence.header.frame_id, evidence.body_frame = 'map', 'livox_frame'
        evidence.source_id, evidence.epoch = self.source_id, 1
        self.sequence += 1
        evidence.sequence = self.sequence
        # Absolute map accuracy is unobservable in this mode. Never advertise
        # zero covariance or claim that an ICP match has been accepted.
        evidence.covariance_known = False
        evidence.pose.covariance = [1e6 if i % 7 == 0 else 0. for i in range(36)]
        evidence.pose.pose.orientation.w = 1.
        try:
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'livox_frame':
                self.core.reject('INPUT_FRAME_MISMATCH')
            if msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 10**9:
                self.core.reject('INVALID_TIMESTAMP_FIELDS')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            pose = self.core.observe(msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec,
                self.get_clock().now().nanoseconds, time.monotonic(),
                [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            fill_pose(evidence.pose.pose, pose)
            evidence.accepted = True
            evidence.reason = 'ODOMETRY_ONLY_FIXED_ALIGNMENT_NO_DRIFT_CORRECTION'
        except ValueError as error:
            evidence.accepted = False
            evidence.reason = str(error)
        self.publisher.publish(evidence)
        self.last_output = evidence

    def tick(self):
        reason = self.core.check(self.get_clock().now().nanoseconds, time.monotonic())
        if reason and self.last_output is not None and self.last_output.accepted:
            output = copy.deepcopy(self.last_output)
            self.sequence += 1
            output.sequence = self.sequence
            output.accepted, output.reason = False, reason
            self.publisher.publish(output)
            self.last_output = output


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdometryLocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
