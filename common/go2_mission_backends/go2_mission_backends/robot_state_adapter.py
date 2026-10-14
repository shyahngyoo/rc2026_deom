#!/usr/bin/env python3
"""Adapt IMU-referenced FAST-LIO odometry without changing upstream TF parents.

The local odom branch remains usable when global TF is unavailable; status
explicitly distinguishes local_pose_valid/local_odom_valid/global_ready. No
map TF is synthesized from a missing query, and stale transforms are never
rebroadcast with fresh stamps. Mission must additionally require fresh GICP
accepted evidence; TF availability alone is not localization acceptance.
"""
import json
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException
from .robot_state import (InputGuard, PoseVelocityEstimator, body_pose,
    checked_covariance, installation_matrix, navigation_transforms,
    planar_pose_covariance, planar_twist, pose_matrix, rotation_quaternion,
    transform_pose_covariance, transform_twist)


PARAMETER_DEFAULTS = {
    'source_topic': '/Odometry', 'odom_topic': '/odom',
    'status_topic': '/robot_state/status', 'source_odom_frame': 'odom',
    'source_imu_frame': 'livox_frame', 'nav_odom_frame': 'nav_odom',
    'footprint_frame': 'base_footprint', 'base_frame': 'base_link',
    'map_frame': 'map', 'imu_in_base_xyz': [0., 0., 0.],
    'imu_in_base_rpy': [0., 0., 0.], 'ground_z': 0.,
    'velocity_mode': 'pose_derived', 'max_input_age': .5,
    'future_tolerance': .05, 'input_wall_timeout': .5,
    'watchdog_period': .05, 'unknown_covariance': 1e6,
    'velocity_window': .3, 'velocity_min_span': .08, 'velocity_max_gap': .5,
    'velocity_max_linear_speed': 5., 'velocity_max_angular_speed': 10.,
    'derived_linear_variance': .25, 'derived_angular_variance': .25,
}


def stamp_nanoseconds(stamp):
    return stamp.sec*1000000000+stamp.nanosec


def message_pose(pose):
    p, q = pose.position, pose.orientation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def transform_matrix(transform):
    p, q = transform.translation, transform.rotation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def fill_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3,3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, rotation_quaternion(matrix[:3,:3]))


class RobotStateAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_state_adapter', **kwargs)
        for name, default in PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.params = {name: self.get_parameter(name).value for name in PARAMETER_DEFAULTS}
        p = self.params
        try:
            frames = [p[n] for n in ('map_frame', 'source_odom_frame', 'source_imu_frame',
                       'nav_odom_frame', 'footprint_frame', 'base_frame')]
            if any(not isinstance(f,str) or not f or f.startswith('/') or any(c.isspace() for c in f) for f in frames) or len(set(frames)) != len(frames):
                raise ValueError('INVALID_OR_CONFLICTING_FRAMES')
            if p['source_topic'] == p['odom_topic'] or not all(p[n] for n in ('source_topic','odom_topic','status_topic')):
                raise ValueError('INVALID_OR_CONFLICTING_TOPICS')
            if p['velocity_mode'] not in ('pose_derived', 'child', 'world'):
                raise ValueError('INVALID_VELOCITY_MODE')
            if not math.isfinite(p['ground_z']) or not math.isfinite(p['unknown_covariance']) or p['unknown_covariance'] <= 0 or not math.isfinite(p['watchdog_period']) or p['watchdog_period'] <= 0:
                raise ValueError('INVALID_PARAMETERS')
            self.imu_in_base = installation_matrix(p['imu_in_base_xyz'],p['imu_in_base_rpy'])
            self.guard = InputGuard(p['max_input_age'],p['future_tolerance'],p['input_wall_timeout'])
            self.velocity = PoseVelocityEstimator(p['velocity_window'],p['velocity_min_span'],
                p['velocity_max_gap'],p['velocity_max_linear_speed'],p['velocity_max_angular_speed'],
                p['derived_linear_variance'],p['derived_angular_variance'])
        except (ValueError, TypeError) as error:
            self.get_logger().error(json.dumps({'module':'robot_state','state':'FAULT','reason':str(error)}))
            raise
        self.odom_publisher = self.create_publisher(Odometry,p['odom_topic'],20)
        self.status_publisher = self.create_publisher(String,p['status_topic'],
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.status = {'module':'robot_state','state':'WAITING','reason':'WAITING_FOR_INPUT',
            'input_stamp_ns':0,'local_pose_valid':False,'local_odom_valid':False,
            'global_ready':False,'velocity_mode':p['velocity_mode'],
            'covariance_unknown':True}
        self.last_status_reason = None
        self.create_subscription(Odometry,p['source_topic'],self.on_odometry,qos_profile_sensor_data)
        # Wall watchdog must continue to expire input if /clock pauses.
        self.watchdog = self.create_timer(p['watchdog_period'],self.on_watchdog,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    def publish_status(self):
        data = json.dumps(self.status,sort_keys=True)
        self.status_publisher.publish(String(data=data))
        if self.last_status_reason != self.status['reason']:
            self.last_status_reason = self.status['reason']
            self.get_logger().info(data)

    def fault(self, reason, reset_velocity=True):
        if reset_velocity:
            self.velocity.reset()
        self.status.update(state='FAULT',reason=reason,local_pose_valid=False,
                           local_odom_valid=False,global_ready=False)
        self.publish_status()

    def on_watchdog(self):
        reason = self.guard.expired(self.get_clock().now().nanoseconds,time.monotonic())
        if reason:
            self.fault(reason)
        else:
            self.publish_status()

    def make_transform(self,parent,child,matrix,stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float,matrix[:3,3])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float,rotation_quaternion(matrix[:3,:3]))
        return t

    def on_odometry(self, source):
        p = self.params
        stamp_ns = stamp_nanoseconds(source.header.stamp)
        self.status['input_stamp_ns'] = stamp_ns
        if source.header.stamp.sec < 0 or not 0 <= source.header.stamp.nanosec < 1000000000:
            self.fault('INVALID_TIMESTAMP_FIELDS')
            return
        if source.header.frame_id != p['source_odom_frame'] or source.child_frame_id != p['source_imu_frame']:
            self.fault('INPUT_FRAME_MISMATCH')
            return
        reason = self.guard.accept(stamp_ns,self.get_clock().now().nanoseconds,time.monotonic())
        if reason:
            self.fault(reason)
            return
        try:
            source_pose = message_pose(source.pose.pose)
            base = body_pose(source_pose,self.imu_in_base)
            pose_cov, unknown_pose = checked_covariance(source.pose.covariance,p['unknown_covariance'])
            body_cov = transform_pose_covariance(pose_cov,source_pose,base)
            footprint, local_body, _ = navigation_transforms(base,ground_z=p['ground_z'])
            planar_cov = planar_pose_covariance(body_cov,base,p['unknown_covariance'])
            if p['velocity_mode'] == 'pose_derived':
                derivative = self.velocity.update(stamp_ns,base)
                base_twist = derivative.base_twist if derivative is not None else None
                twist_cov = derivative.covariance if derivative is not None else None
                unknown_twist = derivative is None
            else:
                t = source.twist.twist
                base_twist, j = transform_twist([t.linear.x,t.linear.y,t.linear.z],
                    [t.angular.x,t.angular.y,t.angular.z],base,self.imu_in_base,p['velocity_mode'])
                twist_cov, unknown_twist = checked_covariance(source.twist.covariance,p['unknown_covariance'])
                # Explicit measured mode requires covariance evidence. In particular,
                # all-zero FAST-LIO twist/covariance cannot masquerade as stopped.
                if unknown_twist:
                    raise ValueError('MEASURED_TWIST_COVARIANCE_UNKNOWN')
                twist_cov = j @ twist_cov @ j.T
            planar_velocity, planar_velocity_cov = None, None
            if base_twist is not None:
                planar_velocity, j = planar_twist(base_twist,base,footprint)
                planar_velocity_cov = j @ twist_cov @ j.T
                for index in (2,3,4):
                    planar_velocity_cov[index,index] = p['unknown_covariance']
        except (ValueError, TypeError, np.linalg.LinAlgError) as error:
            self.fault(str(error))
            return

        global_tf = None
        global_error = None
        try:
            # Never use latest Time(0) for an old pose: interpolation must refer
            # to precisely this measurement, avoiding mixed-time transforms.
            tf = self.tf_buffer.lookup_transform(p['map_frame'],p['source_odom_frame'],
                    Time(nanoseconds=stamp_ns,clock_type=self.get_clock().clock_type))
            map_source = transform_matrix(tf.transform)
            footprint, local_body, global_tf = navigation_transforms(base,map_source,p['ground_z'])
        except (TransformException, ValueError) as error:
            global_error = 'GLOBAL_TF_UNAVAILABLE'

        transforms = [self.make_transform(p['nav_odom_frame'],p['footprint_frame'],footprint,source.header.stamp),
                      self.make_transform(p['footprint_frame'],p['base_frame'],local_body,source.header.stamp)]
        if global_tf is not None:
            transforms.append(self.make_transform(p['map_frame'],p['nav_odom_frame'],global_tf,source.header.stamp))
        self.broadcaster.sendTransform(transforms)
        if planar_velocity is not None:
            odom = Odometry()
            odom.header.stamp = source.header.stamp
            odom.header.frame_id, odom.child_frame_id = p['nav_odom_frame'],p['footprint_frame']
            fill_pose(odom.pose.pose,footprint)
            odom.pose.covariance = planar_cov.ravel().tolist()
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float,planar_velocity[:3])
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float,planar_velocity[3:])
            odom.twist.covariance = planar_velocity_cov.ravel().tolist()
            self.odom_publisher.publish(odom)
        reason = self.velocity.reason if planar_velocity is None else (global_error or 'READY')
        self.status.update(state='READY' if planar_velocity is not None and global_tf is not None else 'DEGRADED',
            reason=reason,local_pose_valid=True,local_odom_valid=planar_velocity is not None,
            global_ready=planar_velocity is not None and global_tf is not None,
            covariance_unknown=bool(unknown_pose or unknown_twist))
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotStateAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
