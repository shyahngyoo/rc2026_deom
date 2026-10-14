"""Fresh measured odometry stop detection; no velocity commands."""
import math
import time
from nav_msgs.msg import Odometry

class StopMonitor:
    def __init__(self, node, odom_timeout, linear_limit, angular_limit,
                 stable_duration):
        self.node = node
        self.odom_timeout = odom_timeout
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.stable_duration = stable_duration
        self.last_wall = None
        self.last_stamp_ns = None
        self.stable_since = None
        self.subscription = node.create_subscription(Odometry, "/odom",
                                                     self._odom, 10)

    def _odom(self, msg):
        now = time.monotonic()
        if self.last_wall is not None and now - self.last_wall > self.odom_timeout:
            self.stable_since = None
        self.last_wall = now
        previous_stamp_ns = self.last_stamp_ns
        self.last_stamp_ns = (msg.header.stamp.sec * 1000000000 +
                              msg.header.stamp.nanosec)
        if previous_stamp_ns is not None and self.last_stamp_ns <= previous_stamp_ns:
            self.stable_since = None
            return
        twist = msg.twist.twist
        slow = (all(math.isfinite(v) for v in (twist.linear.x, twist.linear.y,
                 twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z))
                and math.sqrt(twist.linear.x**2 + twist.linear.y**2 +
                              twist.linear.z**2) <=
                self.linear_limit and abs(twist.angular.z) <=
                self.angular_limit and abs(twist.angular.x) <= self.angular_limit
                and abs(twist.angular.y) <= self.angular_limit)
        if slow:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None

    def reset(self):
        """Require a new continuous stop interval after navigation or a skill succeeds."""
        self.stable_since = None

    def ready(self):
        now = time.monotonic()
        if self.last_wall is None or now - self.last_wall > self.odom_timeout:
            self.stable_since = None
            return False
        ros_now = self.node.get_clock().now().nanoseconds
        age = (ros_now - self.last_stamp_ns) / 1e9
        if self.last_stamp_ns <= 0 or age > self.odom_timeout or age < -0.1:
            self.stable_since = None
            return False
        return (self.stable_since is not None and
                now - self.stable_since >= self.stable_duration)



