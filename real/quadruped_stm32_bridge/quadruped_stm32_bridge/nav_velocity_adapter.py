"""Bind each fresh Nav2 command to the current arbiter lease; never open UART."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from go2_obstacle_interfaces.msg import MotionAuthority, MotionCommand
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


def velocity_reason(values, max_linear=0.5, max_angular=1.0):
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        return 'NONFINITE_COMMAND'
    # Exact zero is the hardware contract, including small lateral requests.
    if values[1] != 0.0:
        return 'LATERAL_MOTION_UNSUPPORTED'
    if any(v != 0.0 for v in values[2:5]):
        return 'NONPLANAR_COMMAND'
    if abs(values[0]) > max_linear or abs(values[5]) > max_angular:
        return 'COMMAND_LIMIT_EXCEEDED'
    return ''


class NavVelocityAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('nav_velocity_adapter', **kwargs)
        defaults = dict(input_topic='/onboard/nav_cmd_raw', output_topic='/motion/nav_command',
            authority_topic='/motion/authority', base_frame='base_footprint',
            authority_timeout=0.3, command_timeout=0.2, max_linear_speed=0.5,
            max_angular_speed=1.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['input_topic'] == self.p['output_topic']:
            raise ValueError('raw and typed topics must differ')
        for key in ('authority_timeout', 'command_timeout', 'max_linear_speed', 'max_angular_speed'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError('invalid '+key)
        self.authority = None
        self.authority_at = self.command_at = 0.0
        self.last_stamp = 0
        self.authority_stamp = self.authority_generation = 0
        self.timed_out = True
        self.last_ros = None
        self.pub = self.create_publisher(MotionCommand, self.p['output_topic'], 1)
        self.create_subscription(MotionAuthority, self.p['authority_topic'], self._authority, 1)
        self.create_subscription(Twist, self.p['input_topic'], self._velocity, 1)
        self.create_timer(0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _authority(self, message):
        stamp = message.header.stamp.sec*10**9 + message.header.stamp.nanosec
        if (stamp <= 0 or not 0 <= message.header.stamp.nanosec < 10**9
                or message.generation < self.authority_generation or stamp < self.authority_stamp):
            self.authority = None
            self.timed_out = True
            return
        if stamp == self.authority_stamp:
            return  # Duplicate heartbeats cannot renew a short monotonic lease.
        self.authority_stamp, self.authority_generation = stamp, message.generation
        key = (message.owner, message.mission_id, message.generation)
        old = self.authority
        if old is None or key != (old.owner, old.mission_id, old.generation):
            self.timed_out = True
            self.command_at = 0.0
        self.authority, self.authority_at = message, time.monotonic()

    def _authorized(self):
        a = self.authority
        if a is None:
            return False
        now, wall = self.get_clock().now().nanoseconds, time.monotonic()
        stamp = a.header.stamp.sec*10**9 + a.header.stamp.nanosec
        elapsed = wall-self.authority_at
        return bool(a.owner == 'NAVIGATION' and a.healthy and a.mission_id and a.generation > 0
            and math.isfinite(a.lease_remaining) and a.lease_remaining > elapsed
            and 0 <= elapsed <= self.p['authority_timeout']
            and stamp > 0 and 0 <= (now-stamp)/1e9 <= self.p['authority_timeout'])

    def _publish(self, twist):
        if not self._authorized():
            return False
        stamp = self.get_clock().now().nanoseconds
        if stamp <= self.last_stamp:
            return False
        a = self.authority
        message = MotionCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.p['base_frame']
        message.owner, message.mission_id, message.generation = a.owner, a.mission_id, a.generation
        message.twist = twist
        self.last_stamp = stamp
        self.pub.publish(message)
        return True

    def _velocity(self, message):
        v = (message.linear.x, message.linear.y, message.linear.z,
             message.angular.x, message.angular.y, message.angular.z)
        reason = velocity_reason(v, self.p['max_linear_speed'], self.p['max_angular_speed'])
        if reason:
            self.get_logger().error(reason+'; command rejected')
            self._publish(Twist())
            self.timed_out = True
            return
        if not self._authorized():
            self.get_logger().debug('NO_FRESH_NAVIGATION_AUTHORITY')
            return
        if self._publish(message):
            self.command_at, self.timed_out = time.monotonic(), False

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros is not None and now < self.last_ros:
            self.authority = None
            self.last_stamp = 0
            self.authority_stamp = self.authority_generation = 0
            self.timed_out = True
        self.last_ros = now
        if not self.timed_out and time.monotonic()-self.command_at > self.p['command_timeout']:
            # Fence the last command once, then stop heartbeating. The arbiter
            # independently expires its command and lease; never replay Twist.
            self._publish(Twist())
            self.timed_out = True


def main(args=None):
    rclpy.init(args=args)
    node = NavVelocityAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
