from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest

from robocon_mission import localization_health
from robocon_mission.localization_health import AcceptedPoseHealth


def pose(stamp=10, frame="map"):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = frame
    message.header.stamp.sec = stamp
    message.pose.pose.orientation.w = 1.0
    return message


def test_health_requires_accepted_pose_and_checks_both_clocks(monkeypatch):
    wall = [100.0]
    monkeypatch.setattr(localization_health.time, "monotonic", lambda: wall[0])
    health = AcceptedPoseHealth(4.0)
    assert not health.healthy(10_000_000_000)
    health.observe(pose())
    assert health.healthy(11_000_000_000)
    assert not health.healthy(15_000_000_000)
    assert not health.healthy(9_000_000_000)
    wall[0] += 4.1
    assert not health.healthy(10_000_000_000)  # ROS time frozen, wall age expired.


@pytest.mark.parametrize("stamp,frame", [(0, "map"), (10, "odom")])
def test_invalid_pose_revokes_health(stamp, frame):
    health = AcceptedPoseHealth(4.0)
    health.observe(pose())
    assert health.healthy(10_000_000_000)
    health.observe(pose(stamp, frame))
    assert not health.healthy(10_000_000_000)


def test_nonfinite_pose_is_not_healthy():
    health = AcceptedPoseHealth(4.0)
    message = pose()
    message.pose.pose.position.x = float("nan")
    health.observe(message)
    assert not health.healthy(10_000_000_000)


@pytest.mark.parametrize("invalid", ["quaternion", "negative_covariance", "nan_covariance"])
def test_malformed_accepted_pose_is_not_trusted(invalid):
    health = AcceptedPoseHealth(4.0)
    message = pose()
    if invalid == "quaternion":
        message.pose.pose.orientation.w = 0.0
    elif invalid == "negative_covariance":
        message.pose.covariance[0] = -1.0
    else:
        message.pose.covariance[7] = float("nan")
    health.observe(message)
    assert not health.healthy(10_000_000_000)
