import math
import pytest
from go2_fastlio_localization.planar_velocity import PlanarVelocityEstimator


def sample(estimator, t, x=0., y=0., yaw=0., frame=('camera_init', 'body')):
    return estimator.update(round(t*1e9), x, y, yaw, frame)


def test_body_axes_and_irregular_stamps():
    estimator = PlanarVelocityEstimator()
    assert sample(estimator, 1., yaw=math.pi/2) is None
    for dt in [.03, .11, .19, .30]:
        velocity = sample(estimator, 1.+dt, y=.2*dt, yaw=math.pi/2)
    assert velocity.vx == pytest.approx(.2)
    assert velocity.vy == pytest.approx(0., abs=1e-12)
    assert velocity.wz == pytest.approx(0.)


@pytest.mark.parametrize('omega', [.5, -.5])
def test_yaw_wrap_and_direction(omega):
    estimator = PlanarVelocityEstimator()
    initial = math.copysign(math.pi-.03, omega)
    for i in range(8):
        yaw = initial+omega*.1*i
        velocity = sample(estimator, 1.+.1*i, yaw=math.atan2(math.sin(yaw), math.cos(yaw)))
    assert velocity.wz == pytest.approx(omega)


def test_stopped_after_window_and_positive_covariance():
    estimator = PlanarVelocityEstimator()
    for i in range(22):
        velocity = sample(estimator, 1.+i*.1, x=.2*min(i*.1, .3))
    assert velocity.vx == pytest.approx(0., abs=1e-12)
    assert velocity.wz == pytest.approx(0.)
    assert velocity.linear_variance > 0 and velocity.angular_variance > 0


def test_duplicates_rewind_gap_frames_and_invalid_values():
    estimator = PlanarVelocityEstimator()
    assert sample(estimator, 1.) is None
    assert sample(estimator, 1.1, x=.01).vx == pytest.approx(.1)
    assert sample(estimator, 1.1, x=99.) is None
    assert sample(estimator, 1.2, x=.02).vx == pytest.approx(.1)
    assert sample(estimator, .5) is None
    assert sample(estimator, .6, x=.01).vx == pytest.approx(.1)
    assert sample(estimator, 2.) is None
    assert sample(estimator, 2.1, frame=('new', 'body')) is None
    assert sample(estimator, 2.2, x=math.nan) is None
    assert sample(estimator, 2.3) is None
    assert sample(estimator, 2.4).vx == pytest.approx(0.)


def test_pose_discontinuity_does_not_become_velocity():
    estimator = PlanarVelocityEstimator()
    sample(estimator, 1.)
    sample(estimator, 1.1, x=.02)
    assert sample(estimator, 1.2, x=10.) is None
    assert sample(estimator, 1.3, x=10.02).vx == pytest.approx(.2)


@pytest.mark.parametrize('kwargs', [{'window': 0.}, {'window': math.nan},
                                   {'min_span': 1.0}, {'max_gap': 0.}])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        PlanarVelocityEstimator(**kwargs)


def test_long_window_does_not_allow_a_long_message_gap():
    estimator = PlanarVelocityEstimator(window=1.0, max_gap=.5)
    for i in range(12):
        velocity = sample(estimator, 1.+.1*i, x=.2*.1*i)
    assert velocity.vx == pytest.approx(.2)
    assert sample(estimator, 3., x=.4) is None
