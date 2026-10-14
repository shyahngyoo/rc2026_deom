"""Isolated real rclpy Topic/TF/watchdog tests; no hardware or Gazebo.

Run with ROS_DOMAIN_ID=128 ROS_LOCALHOST_ONLY=1; all input/output topic and
frame names are additionally unique to each fixture. No /cmd_vel is published.
"""
import json
import os
import time
import uuid
os.environ['ROS_DOMAIN_ID'] = '128'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np
import pytest
rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from go2_mission_backends.robot_state import (body_pose, installation_matrix,
    navigation_transforms, planar_pose, rotation_quaternion, pose_matrix)
from go2_mission_backends.robot_state_adapter import RobotStateAdapter, fill_pose, transform_matrix


class Harness:
    def __init__(self, overrides=None):
        self.context = Context()
        rclpy.init(context=self.context)
        self.tag = 'state_'+uuid.uuid4().hex[:8]
        self.params = dict(source_topic='/'+self.tag+'/input', odom_topic='/'+self.tag+'/odom',
            status_topic='/'+self.tag+'/status', source_odom_frame=self.tag+'_source',
            source_imu_frame=self.tag+'_imu', nav_odom_frame=self.tag+'_nav',
            footprint_frame=self.tag+'_foot', base_frame=self.tag+'_base', map_frame=self.tag+'_map',
            velocity_mode='child',max_input_age=.3,input_wall_timeout=.3,
            imu_in_base_xyz=[.4,-.2,.5],imu_in_base_rpy=[.2,.1,-.6])
        self.params.update(overrides or {})
        self.adapter = RobotStateAdapter(context=self.context,namespace=self.tag,
            parameter_overrides=[Parameter(k,value=v) for k,v in self.params.items()])
        self.observer = Node(self.tag+'_observer',context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.adapter); self.executor.add_node(self.observer)
        self.source = self.observer.create_publisher(Odometry,self.params['source_topic'],10)
        self.odom = []; self.status = []; self.transforms = []
        self.observer.create_subscription(Odometry,self.params['odom_topic'],self.odom.append,10)
        self.observer.create_subscription(String,self.params['status_topic'],
            lambda msg:self.status.append(json.loads(msg.data)),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.observer.create_subscription(TFMessage,'/tf',lambda m:self.transforms.extend(m.transforms),20)
        self.static_tf = StaticTransformBroadcaster(self.observer)
        self.dynamic_tf = TransformBroadcaster(self.observer)
        self.extrinsic = installation_matrix(self.params['imu_in_base_xyz'],self.params['imu_in_base_rpy'])
        self.spin(.08)

    def close(self):
        self.executor.remove_node(self.adapter); self.executor.remove_node(self.observer)
        self.adapter.destroy_node(); self.observer.destroy_node()
        self.executor.shutdown(); self.context.shutdown()

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=min(.01,end-time.monotonic()))

    def wait(self,predicate,timeout=1.):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate(): return
            self.executor.spin_once(timeout_sec=.01)
        assert predicate(), self.status[-1] if self.status else 'No status'

    def add_global(self,matrix=None,stamp=None,static=True):
        msg = TransformStamped()
        msg.header.stamp = stamp or self.observer.get_clock().now().to_msg()
        msg.header.frame_id = self.params['map_frame']
        msg.child_frame_id = self.params['source_odom_frame']
        matrix = np.eye(4) if matrix is None else matrix
        msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z = map(float,matrix[:3,3])
        msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w = map(float,rotation_quaternion(matrix[:3,:3]))
        (self.static_tf if static else self.dynamic_tf).sendTransform(msg)
        self.spin(.05)
        return msg

    def message(self,base=None,stamp=None):
        msg = Odometry()
        msg.header.stamp = stamp or self.observer.get_clock().now().to_msg()
        msg.header.frame_id = self.params['source_odom_frame']
        msg.child_frame_id = self.params['source_imu_frame']
        base = np.eye(4) if base is None else base
        fill_pose(msg.pose.pose,base @ self.extrinsic)
        msg.pose.covariance = (np.eye(6)*.02).ravel().tolist()
        msg.twist.covariance = (np.eye(6)*.03).ravel().tolist()
        return msg

    def send(self,message):
        self.source.publish(message)
        self.spin(.03)

    def latest_tf(self,parent,child):
        messages = [m for m in self.transforms if m.header.frame_id == self.params[parent] and m.child_frame_id == self.params[child]]
        assert messages
        return transform_matrix(messages[-1].transform)


@pytest.fixture
def harness_factory():
    harnesses = []
    def make(overrides=None):
        h = Harness(overrides); harnesses.append(h); return h
    yield make
    for h in reversed(harnesses): h.close()


def test_ros_actual_odom_and_tf_preserve_tilted_global_body(harness_factory):
    h = harness_factory()
    global_tf = installation_matrix([2,3,.6],[.3,-.2,.4])
    h.add_global(global_tf)
    base = installation_matrix([1,-2,.7],[.2,-.3,.8])
    msg = h.message(base)
    h.send(msg)
    h.wait(lambda:len(h.odom)>0 and any(s['global_ready'] for s in h.status))
    odom = h.odom[-1]
    assert odom.header.frame_id == h.params['nav_odom_frame']
    assert odom.child_frame_id == h.params['footprint_frame']
    actual_pose = pose_matrix([odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z],
        [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])
    np.testing.assert_allclose(actual_pose,planar_pose(base),atol=1e-9)
    m = h.latest_tf('map_frame','nav_odom_frame')
    f = h.latest_tf('nav_odom_frame','footprint_frame')
    b = h.latest_tf('footprint_frame','base_frame')
    np.testing.assert_allclose(m @ f,planar_pose(global_tf @ base),atol=1e-9)
    np.testing.assert_allclose(m @ f @ b,global_tf @ base,atol=1e-9)
    assert not any(t.child_frame_id == h.params['source_imu_frame'] for t in h.transforms)


def test_ros_missing_exact_tf_keeps_local_odom_but_no_global_tf(harness_factory):
    h = harness_factory()
    # Dynamic TF exists only at an earlier timestamp. Latest Time(0) would
    # wrongly make this pass; exact-time lookup must refuse extrapolation.
    h.add_global(static=False)
    h.send(h.message())
    h.wait(lambda:len(h.odom)>0 and any(s['reason']=='GLOBAL_TF_UNAVAILABLE' for s in h.status))
    assert h.status[-1]['local_odom_valid'] and not h.status[-1]['global_ready']
    assert not any(t.child_frame_id == h.params['nav_odom_frame'] for t in h.transforms)
    h.add_global(static=True)
    h.send(h.message())
    h.wait(lambda:h.status[-1]['global_ready'])


def test_ros_frame_timestamp_stale_and_watchdog_refuse_output(harness_factory):
    h = harness_factory()
    h.add_global()
    h.send(h.message())
    h.wait(lambda:len(h.odom)>0)
    count = len(h.odom)
    wrong = h.message(); wrong.child_frame_id = 'wrong_imu'
    h.send(wrong)
    assert h.status[-1]['reason'] == 'INPUT_FRAME_MISMATCH' and len(h.odom) == count
    stale = h.message(stamp=Time(nanoseconds=h.observer.get_clock().now().nanoseconds-1000000000).to_msg())
    h.send(stale)
    assert h.status[-1]['reason'] == 'STALE_INPUT' and len(h.odom) == count
    invalid_stamp = h.message(); invalid_stamp.header.stamp.nanosec = 1500000000
    h.send(invalid_stamp)
    assert h.status[-1]['reason'] == 'INVALID_TIMESTAMP_FIELDS' and len(h.odom) == count
    future = h.message(stamp=Time(nanoseconds=h.observer.get_clock().now().nanoseconds+1000000000).to_msg())
    h.send(future)
    assert h.status[-1]['reason'] == 'FUTURE_INPUT' and len(h.odom) == count
    good = h.message(); h.send(good); h.wait(lambda:len(h.odom)>count)
    count = len(h.odom); h.send(good)
    assert h.status[-1]['reason'] == 'NON_INCREASING_TIMESTAMP' and len(h.odom) == count
    h.wait(lambda:h.status[-1]['reason'] in ('INPUT_WALL_TIMEOUT','STALE_INPUT'),timeout=1.)
    assert not h.status[-1]['local_odom_valid'] and not h.status[-1]['global_ready']


def test_ros_pose_derivation_ignores_zero_fastlio_twist(harness_factory):
    h = harness_factory(dict(velocity_mode='pose_derived',velocity_min_span=.04))
    h.add_global()
    origin = h.observer.get_clock().now().nanoseconds
    first = h.message(); first.twist.covariance = [0.]*36
    h.send(first)
    assert len(h.odom) == 0 and h.status[-1]['reason'] == 'VELOCITY_WARMUP'
    h.spin(.03)
    second = h.message(); elapsed = (Time.from_msg(second.header.stamp).nanoseconds-origin)*1e-9
    fill_pose(second.pose.pose,installation_matrix([.2*elapsed,.3*elapsed,0],[0,0,0]) @ h.extrinsic)
    second.twist.covariance = [0.]*36
    h.send(second)
    h.wait(lambda:len(h.odom)>0)
    assert h.odom[-1].twist.twist.linear.x == pytest.approx(.2,abs=.015)
    assert h.odom[-1].twist.twist.linear.y == pytest.approx(.3,abs=.015)


def test_ros_unknown_measured_twist_refused(harness_factory):
    h = harness_factory()
    h.add_global()
    msg = h.message(); msg.twist.covariance = [0.]*36
    h.send(msg)
    assert not h.odom
    assert h.status[-1]['reason'] == 'MEASURED_TWIST_COVARIANCE_UNKNOWN'


def test_ros_sim_clock_pause_expires_and_rollback_requires_new_samples(harness_factory):
    h = harness_factory(dict(use_sim_time=True,velocity_mode='pose_derived',velocity_min_span=.04))
    clock_pub = h.observer.create_publisher(ClockMessage,'/clock',10)
    def clock(ns):
        clock_pub.publish(ClockMessage(clock=Time(nanoseconds=ns).to_msg()))
        h.wait(lambda:h.adapter.get_clock().now().nanoseconds == ns)
        h.spin(.07)
    clock(1000000000); h.add_global()
    h.send(h.message(stamp=Time(nanoseconds=1000000000).to_msg()))
    clock(1100000000)
    h.send(h.message(stamp=Time(nanoseconds=1100000000).to_msg()))
    h.wait(lambda:len(h.odom)>0)
    count = len(h.odom)
    h.wait(lambda:h.status[-1]['reason']=='INPUT_WALL_TIMEOUT',timeout=1.)
    assert not h.status[-1]['global_ready']
    clock(500000000)
    assert h.status[-1]['reason'] in ('CLOCK_ROLLBACK','WAITING_FOR_INPUT')
    h.send(h.message(stamp=Time(nanoseconds=500000000).to_msg()))
    assert len(h.odom) == count and h.status[-1]['reason'] == 'VELOCITY_WARMUP'
    clock(600000000)
    h.send(h.message(stamp=Time(nanoseconds=600000000).to_msg()))
    h.wait(lambda:len(h.odom)>count)


def test_ros_twist_has_actual_reference_point_compensation(harness_factory):
    h = harness_factory()
    h.add_global()
    base = installation_matrix([1,2,.7],[0,0,.8])
    msg = h.message(base)
    vb, wb = np.array([.2,.3,0]), np.array([0,0,.7])
    vi = h.extrinsic[:3,:3].T @ (vb+np.cross(wb,h.extrinsic[:3,3]))
    wi = h.extrinsic[:3,:3].T @ wb
    msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z = map(float,vi)
    msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z = map(float,wi)
    h.send(msg); h.wait(lambda:len(h.odom)>0)
    actual = h.odom[-1].twist.twist
    np.testing.assert_allclose([actual.linear.x,actual.linear.y,actual.linear.z],vb,atol=1e-9)
    np.testing.assert_allclose([actual.angular.x,actual.angular.y,actual.angular.z],wb,atol=1e-9)


def test_ros_global_tf_interruption_never_refreshes_old_map_transform(harness_factory):
    h = harness_factory()
    stamp = h.observer.get_clock().now().to_msg()
    h.add_global(stamp=stamp,static=False)
    h.send(h.message(stamp=stamp))
    h.wait(lambda:h.status[-1]['global_ready'])
    old_count = len([t for t in h.transforms if t.child_frame_id == h.params['nav_odom_frame']])
    h.spin(.06)
    h.send(h.message())
    h.wait(lambda:h.status[-1]['reason']=='GLOBAL_TF_UNAVAILABLE')
    new_count = len([t for t in h.transforms if t.child_frame_id == h.params['nav_odom_frame']])
    assert old_count == new_count
    assert h.status[-1]['local_odom_valid'] and not h.status[-1]['global_ready']
