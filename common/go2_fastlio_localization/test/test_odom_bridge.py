from pathlib import Path
import math
import copy
import sys
import time
import pytest
import rclpy
from nav_msgs.msg import Odometry
from go2_fastlio_localization.fast_lio_odom_bridge import FastLioOdomBridge


class Capture:
    def __init__(self):
        self.messages = []
    def publish(self, message):
        self.messages.append(message)
    sendTransform = publish


@pytest.fixture
def bridge():
    rclpy.init()
    node = FastLioOdomBridge()
    node.publisher = Capture()
    node.broadcaster = Capture()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def message(t, x=0., y=0., yaw=0.):
    msg = Odometry()
    stamp = round(t*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
    msg.header.frame_id, msg.child_frame_id = 'camera_init', 'body'
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z = math.sin(yaw/2)
    msg.pose.pose.orientation.w = math.cos(yaw/2)
    return msg


def test_ros_message_velocity_frames_covariance_and_input_unchanged(bridge):
    bridge.on_odometry(message(1., yaw=math.pi/2))
    assert not bridge.publisher.messages  # Unknown speed must not look stopped.
    assert len(bridge.broadcaster.messages) == 1
    source = message(1.1, y=.02, yaw=math.pi/2)
    bridge.on_odometry(source)
    out = bridge.publisher.messages[-1]
    assert source.header.frame_id == 'camera_init'
    assert out.header.frame_id == 'odom' and out.child_frame_id == 'base_footprint'
    assert out.twist.twist.linear.x == pytest.approx(.2)
    assert out.twist.twist.linear.y == pytest.approx(0., abs=1e-12)
    assert out.twist.covariance[0] > 0 and out.twist.covariance[35] > 0
    assert out.twist.covariance[14] == 1e6


def test_angular_feedback_and_reset_does_not_publish_false_stop(bridge):
    bridge.on_odometry(message(1., yaw=3.13))
    bridge.on_odometry(message(1.1, yaw=3.18))
    assert bridge.publisher.messages[-1].twist.twist.angular.z == pytest.approx(.5)
    count = len(bridge.publisher.messages)
    bridge.on_odometry(message(1.1, yaw=3.18))
    bridge.on_odometry(message(2., yaw=3.6))
    assert len(bridge.publisher.messages) == count
    bridge.on_odometry(message(2.1, yaw=3.65))
    assert bridge.publisher.messages[-1].twist.twist.angular.z == pytest.approx(.5)


@pytest.mark.parametrize('component', [0., math.nan, math.inf, 1e308])
def test_invalid_pose_does_not_publish_or_broadcast(bridge, component):
    invalid = message(1.)
    invalid.pose.pose.orientation.w = component
    bridge.on_odometry(invalid)
    bridge.on_odometry(message(1.1, x=math.nan))
    assert not bridge.publisher.messages and not bridge.broadcaster.messages


def test_mission_stop_monitor_receives_motion_and_requires_stable_stop(bridge):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robocon_mission'))
    from robocon_mission.mission_manager import StopMonitor
    monitor = StopMonitor(bridge, .5, .03, .05, 1.)
    now = bridge.get_clock().now().nanoseconds*1e-9
    bridge.on_odometry(message(now-.1))
    assert not bridge.publisher.messages
    bridge.on_odometry(message(now, yaw=.04))
    moving = bridge.publisher.messages[-1]
    monitor._odom(moving)
    assert monitor.stable_since is None and not monitor.ready()
    stopped = copy.deepcopy(moving)
    stopped.twist.twist.angular.z = 0.
    stopped.header.stamp = bridge.get_clock().now().to_msg()
    monitor._odom(stopped)
    assert not monitor.ready()  # It must wait for the one-second stop interval.
    monitor.stable_since = time.monotonic()-1.1
    assert monitor.ready()
