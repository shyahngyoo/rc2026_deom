from dataclasses import replace
import math
import pytest
from go2_mission_backends.localization_core import Evidence, EvidenceHealth


def sample(stamp=10.0, seq=1, epoch=1, accepted=True, **changes):
    e = Evidence(int(stamp*1e9), "map", "livox_frame", "instance-a", epoch, seq,
                 accepted, False, (1., 2., 0.), (0., 0., 0., 1.), (0.,)*36)
    return replace(e, **changes)


def test_normal_and_source_pin():
    h = EvidenceHealth(wall_clock=lambda: 0.)
    assert h.observe(sample(), int(10.1e9))
    assert h.healthy(int(10.2e9))
    assert h.source_id == "instance-a"
    assert h.observe(sample(11., 2), int(11.1e9))


@pytest.mark.parametrize("changes,reason", [
    ({"frame": "odom"}, "FRAME_MISMATCH"),
    ({"body_frame": "base_link"}, "FRAME_MISMATCH"),
    ({"quaternion": (0., 0., 0., 0.)}, "INVALID_QUATERNION"),
    ({"position": (math.nan, 0., 0.)}, "NONFINITE"),
    ({"covariance": (math.inf,)*36}, "NONFINITE"),
    ({"covariance_known": True, "covariance": (-1.,)+(0.,)*35}, "NEGATIVE"),
    ({"source_id": ""}, "INVALID_IDENTITY"),
    ({"epoch": 0}, "INVALID_IDENTITY"),
    ({"sequence": 0}, "INVALID_IDENTITY"),
    ({"stamp_ns": 0}, "MEASUREMENT_STALE"),
    ({"stamp_ns": int(5e9)}, "MEASUREMENT_STALE"),
    ({"stamp_ns": int(12e9)}, "MEASUREMENT_STALE"),
])
def test_untrusted_samples_never_become_healthy(changes, reason):
    h = EvidenceHealth()
    assert not h.observe(sample(**changes), int(10.1e9))
    assert not h.healthy(int(10.1e9))
    assert reason in h.reason


def test_rejection_clears_cache_and_requires_post_fault_measurement():
    h = EvidenceHealth()
    assert h.observe(sample(), int(10.1e9))
    assert not h.observe(sample(10.2, 2, accepted=False, reason="NO_INLIERS"), int(10.2e9))
    assert h.stamp_ns is None
    assert not h.observe(sample(10.15, 3), int(10.25e9))
    assert not h.healthy(int(10.25e9))
    assert h.observe(sample(10.3, 4), int(10.35e9))
    assert h.healthy(int(10.35e9))


@pytest.mark.parametrize("bad", [sample(10.2, 1), sample(9.9, 2), sample(10.2, 2, epoch=0),
                                  sample(10.2, 2, source_id="old-process")])
def test_duplicate_disordered_and_foreign_source_fault(bad):
    h = EvidenceHealth()
    assert h.observe(sample(), int(10.1e9))
    assert not h.observe(bad, int(10.3e9))
    assert not h.healthy(int(10.3e9))
    assert h.observe(sample(10.4, 4), int(10.5e9))


def test_old_epoch_cannot_override_new_epoch():
    h = EvidenceHealth()
    assert h.observe(sample(), int(10.1e9))
    assert not h.observe(sample(10.2, 2, epoch=2, accepted=False), int(10.2e9))
    assert h.observe(sample(10.3, 3, epoch=2), int(10.4e9))
    assert not h.observe(sample(10.5, 99, epoch=1), int(10.6e9))
    assert not h.healthy(int(10.6e9))


def test_expiry_ros_age_and_wall_age_clears_cache():
    wall = [0.]
    h = EvidenceHealth(max_age=1., wall_clock=lambda: wall[0])
    assert h.observe(sample(), int(10.1e9))
    wall[0] = 1.2
    assert not h.healthy(int(10.2e9))
    assert h.stamp_ns is None
    assert not h.observe(sample(10.15, 2), int(10.3e9))
    assert h.observe(sample(10.4, 3), int(10.5e9))
    assert not h.healthy(int(12e9))


def test_phase_needs_both_new_sequence_and_measurement_stamp():
    h = EvidenceHealth()
    assert h.observe(sample(), int(10.1e9))
    h.require_new(int(10.2e9))
    assert not h.phase_fresh(int(10.2e9))
    # Delayed pre-stage data can be a valid tracking update, never stage proof.
    assert h.observe(sample(10.15, 2), int(10.25e9))
    assert not h.phase_fresh(int(10.25e9))
    assert h.observe(sample(10.3, 3), int(10.4e9))
    assert h.phase_fresh(int(10.4e9))


def test_phase_before_first_message_can_be_satisfied():
    h = EvidenceHealth()
    h.require_new(int(10e9))
    assert h.observe(sample(10.1), int(10.2e9))
    assert h.phase_fresh(int(10.2e9))


def test_simulated_clock_stall_requires_new_measurement():
    wall = [0.]
    h = EvidenceHealth(use_sim_time=True, clock_timeout=.5, wall_clock=lambda: wall[0])
    assert h.observe(sample(), int(10e9))
    wall[0] = .6
    assert not h.healthy(int(10e9))
    assert h.reason == "CLOCK_STALLED"
    wall[0] = .7
    assert not h.observe(sample(10., 2), int(10.1e9))
    assert h.observe(sample(10.2, 3), int(10.3e9))


def test_clock_rollback_requires_new_epoch_and_blocks_old_delayed_data():
    h = EvidenceHealth()
    assert h.observe(sample(), int(10.1e9))
    h.require_new(int(10.2e9))
    assert not h.healthy(int(2e9))
    assert not h.observe(sample(2.1, 2), int(2.2e9))
    assert h.observe(sample(2.3, 3, epoch=2), int(2.4e9))
    assert h.phase_fresh(int(2.4e9))
    assert not h.observe(sample(10.3, 50, epoch=1), int(2.5e9))


@pytest.mark.parametrize("kwargs", [{"max_age": 0}, {"max_age": math.nan},
                                    {"clock_timeout": -1}, {"future_tolerance": math.inf},
                                    {"frame": ""}, {"body_frame": ""}])
def test_bad_configuration(kwargs):
    with pytest.raises(ValueError):
        EvidenceHealth(**kwargs)


@pytest.mark.parametrize("sec,nanosec", [(-1,11000000000), (9,1000000000)])
def test_message_timestamp_must_be_canonical(sec,nanosec):
    from types import SimpleNamespace as NS
    m=NS(header=NS(stamp=NS(sec=sec,nanosec=nanosec),frame_id="map"), body_frame="livox_frame",
         source_id="canonical-source",epoch=1,sequence=1,accepted=True,covariance_known=False,reason="",
         pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.)),covariance=[0.]*36))
    e=Evidence.from_message(m)
    assert e.stamp_ns == -1
    h=EvidenceHealth()
    assert not h.observe(e,int(10e9))
    assert not h.healthy(int(10e9))


def test_delayed_pre_fault_samples_do_not_move_recovery_boundary_forever():
    h=EvidenceHealth()
    assert h.observe(sample(1.0,1),int(1.02e9))
    assert not h.observe(sample(1.10,2,accepted=False),int(1.10e9))
    assert not h.observe(sample(1.08,3),int(1.15e9))
    assert h.fault_boundary_ns == int(1.10e9)
    assert h.observe(sample(1.12,4),int(1.20e9))
    assert h.healthy(int(1.20e9))
