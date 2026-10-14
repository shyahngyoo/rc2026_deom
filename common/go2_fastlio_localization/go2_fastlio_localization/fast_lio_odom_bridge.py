#!/usr/bin/env python3
"""Expose FAST-LIO odometry as Nav2's planar odom interface.

This node deliberately has no Gazebo input.  It derives ``/odom`` and the
``odom -> base_footprint`` TF only from FAST-LIO's ``/Odometry`` estimate.
"""

import copy
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .planar_velocity import PlanarVelocityEstimator


def yaw_from_quaternion(quaternion):
    """Return the planar yaw component of a ROS quaternion."""
    components = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    norm_sq = sum(v*v for v in components)
    if not all(math.isfinite(v) for v in components) or not math.isfinite(norm_sq) or norm_sq < 1e-12:
        raise ValueError('invalid odometry quaternion')
    sin_yaw = 2.0 * (quaternion.w*quaternion.z+quaternion.x*quaternion.y)/norm_sq
    cos_yaw = 1.0-2.0*(quaternion.y*quaternion.y+quaternion.z*quaternion.z)/norm_sq
    return math.atan2(sin_yaw, cos_yaw)


class FastLioOdomBridge(Node):
    def __init__(self):
        super().__init__("fast_lio_odom_bridge")

        self.declare_parameter("source_topic", "/Odometry")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("velocity_window", 1.0)
        self.declare_parameter("velocity_min_span", 0.08)
        self.declare_parameter("velocity_max_gap", 0.5)
        self.declare_parameter("velocity_max_linear_speed", 3.0)
        self.declare_parameter("velocity_max_angular_speed", 6.0)

        self.velocity_estimator = PlanarVelocityEstimator(
            self.get_parameter("velocity_window").value,
            self.get_parameter("velocity_min_span").value,
            self.get_parameter("velocity_max_gap").value,
            self.get_parameter("velocity_max_linear_speed").value,
            self.get_parameter("velocity_max_angular_speed").value,
        )

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        source_topic = self.get_parameter("source_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.publisher = self.create_publisher(Odometry, odom_topic, 20)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, source_topic, self.on_odometry, 20)

        self.get_logger().info(
            f"Bridging {source_topic} to {odom_topic} and "
            f"{self.odom_frame} -> {self.base_frame}; body twist from "
            f"FAST-LIO pose slopes, window={self.velocity_estimator.window:.3f}s (no Gazebo truth)"
        )

    def on_odometry(self, source):
        stamp_ns = source.header.stamp.sec*1000000000+source.header.stamp.nanosec
        position = source.pose.pose.position
        try:
            yaw = yaw_from_quaternion(source.pose.pose.orientation)
            if (stamp_ns <= 0 or not source.header.frame_id or not source.child_frame_id or
                    not all(math.isfinite(v) for v in (position.x, position.y))):
                raise ValueError('invalid odometry stamp or position')
        except ValueError:
            self.velocity_estimator.reset()
            return
        velocity = self.velocity_estimator.update(
            stamp_ns, position.x, position.y, yaw,
            (source.header.frame_id, source.child_frame_id),
        )
        half_yaw = 0.5 * yaw

        odom = Odometry()
        odom.header = copy.deepcopy(source.header)
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = source.pose.pose.position.x
        odom.pose.pose.position.y = source.pose.pose.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        if velocity is not None:
            # Odometry twist is expressed in child_frame_id, not in odom/map.
            odom.twist.twist.linear.x = velocity.vx
            odom.twist.twist.linear.y = velocity.vy
            odom.twist.twist.angular.z = velocity.wz
            odom.twist.covariance[0] = velocity.linear_variance
            odom.twist.covariance[7] = velocity.linear_variance
            odom.twist.covariance[35] = velocity.angular_variance
            for index in (14, 21, 28):
                odom.twist.covariance[index] = 1e6

        # FAST-LIO's covariance is not always populated.  Give Nav2 a finite,
        # conservative planar covariance rather than forwarding all-zero data.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.08
        # Continue TF during warm-up, but do not publish an unknown derivative
        # as fresh zero odometry: Mission StopMonitor would treat it as stopped.
        if velocity is not None:
            self.publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.rotation.z = odom.pose.pose.orientation.z
        transform.transform.rotation.w = odom.pose.pose.orientation.w
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = FastLioOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
