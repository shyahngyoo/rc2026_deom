import math
import numpy as np
import pytest
from go2_mission_backends.odometry_localization import OdometryLocalization


def sample(core, stamp=1_000_000_000, now=None, wall=1., xyz=(0., 0., 0.), q=(0., 0., 0., 1.)):
    return core.observe(stamp, stamp if now is None else now, wall, xyz, q)


def test_fixed_alignment_rotates_and_translates_without_snapping_first_sample():
    core = OdometryLocalization([2., 3., .4, 0., 0., math.pi/2])
    pose = sample(core, xyz=(1., 0., 0.))
    np.testing.assert_allclose(pose[:3, 3], [2., 4., .4])
    pose = sample(core, stamp=1_100_000_000, wall=1.1, xyz=(1.1, 0., 0.))
    np.testing.assert_allclose(pose[:3, 3], [2., 4.1, .4])


@pytest.mark.parametrize('args,reason', [
    (dict(stamp=1_000_000_000), 'NON_INCREASING_TIMESTAMP'),
    (dict(stamp=900_000_000), 'CLOCK_ROLLBACK'),
    (dict(stamp=1_100_000_000, xyz=(3., 0., 0.)), 'ODOMETRY_DISCONTINUITY'),
    (dict(stamp=1_100_000_000, q=(0., 0., 1., 0.)), 'ODOMETRY_DISCONTINUITY'),
    (dict(stamp=1_100_000_000, xyz=(float('nan'), 0., 0.)), 'INVALID_POSITION'),
    (dict(stamp=1_100_000_000, now=1_010_000_000), 'FUTURE_INPUT'),
    (dict(stamp=1_100_000_000, wall=1.6), 'INPUT_WALL_TIMEOUT'),
])
def test_bad_session_latches_until_explicit_restart(args, reason):
    core = OdometryLocalization([0.]*6)
    sample(core)
    with pytest.raises(ValueError, match=reason):
        sample(core, **args)
    with pytest.raises(ValueError, match=reason):
        sample(core, stamp=1_200_000_000, wall=1.2)


def test_watchdog_expires_without_more_callbacks():
    core = OdometryLocalization([0.]*6)
    assert core.check(1_000_000_000, 1.) == 'WAITING_FOR_INPUT'
    sample(core)
    assert core.check(1_200_000_000, 1.2) is None
    assert core.check(1_200_000_000, 1.6) == 'INPUT_WALL_TIMEOUT'
    with pytest.raises(ValueError, match='INPUT_WALL_TIMEOUT'):
        sample(core, stamp=1_300_000_000, wall=1.7)


@pytest.mark.parametrize('alignment', [[], [0.]*5, [float('nan')]*6])
def test_invalid_alignment(alignment):
    with pytest.raises(ValueError):
        OdometryLocalization(alignment)
