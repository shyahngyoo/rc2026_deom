from types import SimpleNamespace
from geometry_msgs.msg import PoseWithCovarianceStamped
from robocon_mission.localization_client import LocalizationClient
from robocon_mission.localization_health import AcceptedPoseHealth
from robocon_mission import localization_client


def test_old_global_correction_is_not_made_new_by_fresh_tf(monkeypatch):
    clock = [10_000_000_000]
    monkeypatch.setattr(localization_client.time, 'monotonic', lambda: 100.)
    client = LocalizationClient.__new__(LocalizationClient)
    client.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=clock[0])))
    client.accepted = AcceptedPoseHealth(4.)
    client.accepted_minimum_stamp_ns = 0
    client.health_ok, client.health_wall, client.health_timeout = True, 100., 1.
    client.tf_fresh = lambda: True
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 9
    pose.pose.pose.orientation.w = 1.
    client.accepted.observe(pose)
    assert client.healthy()
    client.require_new_accepted_pose()
    assert not client.healthy()
    pose.header.stamp.sec = 10
    pose.header.stamp.nanosec = 100_000_000
    clock[0] = 10_200_000_000
    client.accepted.observe(pose)
    assert client.healthy()
