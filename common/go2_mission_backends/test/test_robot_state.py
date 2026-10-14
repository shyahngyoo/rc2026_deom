import math
import numpy as np
import pytest
from go2_mission_backends.robot_state import (
    InputGuard, PoseVelocityEstimator, body_pose, checked_covariance,
    installation_matrix, navigation_transforms, planar_pose,
    planar_pose_covariance, planar_twist, pose_matrix, quaternion_rotation,
    rotation_quaternion, skew, transform_pose_covariance, transform_twist)


@pytest.mark.parametrize('yaw', [0., .7, math.pi, -2.8])
def test_rotation_quaternion_roundtrip(yaw):
    r = installation_matrix([0,0,0],[.2,-.3,yaw])[:3,:3]
    np.testing.assert_allclose(quaternion_rotation(rotation_quaternion(r)),r,atol=1e-12)


def test_pose_installation_handles_roll_pitch_and_offset():
    b = installation_matrix([1.,2.,.7],[.3,-.2,.8])
    extrinsic = installation_matrix([.4,-.2,.5],[.2,.1,-.6])
    np.testing.assert_allclose(body_pose(b @ extrinsic,extrinsic),b,atol=1e-12)


@pytest.mark.parametrize('direction', [[1.,0.,0.],[-1.,0.,0.],[0.,1.,0.],[0.,-1.,0.]])
def test_forward_reverse_lateral_twist(direction):
    b = installation_matrix([2.,3.,.7],[.1,-.2,.9])
    extrinsic = installation_matrix([.4,-.2,.5],[.2,.1,-.6])
    expected = np.r_[direction,[0.,0.,0.]]
    imu_velocity = extrinsic[:3,:3].T @ np.array(direction)
    child, _ = transform_twist(imu_velocity,[0,0,0],b,extrinsic,'child')
    world, _ = transform_twist(b[:3,:3] @ direction,[0,0,0],b,extrinsic,'world')
    np.testing.assert_allclose(child,expected,atol=1e-12)
    np.testing.assert_allclose(world,expected,atol=1e-12)


def test_rotation_lever_arm_cancelled_with_rotated_imu():
    b = installation_matrix([1,2,.7],[.1,-.2,.9])
    e = installation_matrix([.4,-.2,.5],[.2,.1,-.6])
    omega = np.array([.3,-.2,.8])
    imu_linear = e[:3,:3].T @ np.cross(omega,e[:3,3])
    imu_angular = e[:3,:3].T @ omega
    twist, _ = transform_twist(imu_linear,imu_angular,b,e,'child')
    np.testing.assert_allclose(twist[:3],np.zeros(3),atol=1e-12)
    np.testing.assert_allclose(twist[3:],omega,atol=1e-12)


def test_combined_translation_and_rotation_world_convention():
    b = installation_matrix([1,2,.7],[.1,-.2,.9])
    e = installation_matrix([.4,-.2,.5],[.2,.1,-.6])
    vb, wb = np.array([.3,-.7,.2]), np.array([.1,.2,.6])
    vim = b[:3,:3] @ (vb+np.cross(wb,e[:3,3]))
    wim = b[:3,:3] @ wb
    result, j = transform_twist(vim,wim,b,e,'world')
    np.testing.assert_allclose(result,np.r_[vb,wb],atol=1e-12)
    np.testing.assert_allclose(j @ np.r_[vim,wim],result,atol=1e-12)


def test_tilted_global_frame_preserves_horizontal_navigation_and_full_body():
    local = installation_matrix([3,1,.6],[.1,-.3,.8])
    global_correction = installation_matrix([5,-2,.8],[.3,-.2,.4])
    footprint, footprint_body, map_nav = navigation_transforms(local,global_correction,.05)
    global_body = global_correction @ local
    global_footprint = planar_pose(global_body,.05)
    np.testing.assert_allclose(map_nav @ footprint,global_footprint,atol=1e-12)
    np.testing.assert_allclose(map_nav @ footprint @ footprint_body,global_body,atol=1e-12)
    np.testing.assert_allclose(map_nav[2,:],[0,0,1,0],atol=1e-12)
    np.testing.assert_allclose(global_footprint[2,:],[0,0,1,.05],atol=1e-12)


def test_missing_global_tf_returns_only_local_branch():
    local = installation_matrix([3,1,.6],[.1,-.3,.8])
    footprint, footprint_body, map_nav = navigation_transforms(local)
    assert map_nav is None
    np.testing.assert_allclose(footprint @ footprint_body,local,atol=1e-12)


def test_pose_covariance_installation_jacobian():
    b = installation_matrix([1,2,.7],[.2,-.1,.9])
    e = installation_matrix([.4,-.2,.5],[.2,.1,-.6])
    imu = b @ e
    c = np.diag([.1,.2,.3,.04,.05,.06])
    result = transform_pose_covariance(c,imu,b)
    offset = b[:3,3]-imu[:3,3]
    j = np.eye(6); j[:3,3:] = -skew(offset)
    np.testing.assert_allclose(result,j @ c @ j.T)
    assert np.all(np.diag(result)[:3] >= np.diag(c)[:3])


def test_planar_yaw_rate_includes_tilt():
    b = installation_matrix([1,2,.7],[.4,-.3,.9])
    twist = np.r_[.4,.3,.2,.2,-.1,.7]
    projected, _ = planar_twist(twist,b,planar_pose(b))
    # Euler yaw rate = (sin(roll)*wy + cos(roll)*wz)/cos(pitch).
    expected = (math.sin(.4)*twist[4]+math.cos(.4)*twist[5])/math.cos(-.3)
    assert projected[5] == pytest.approx(expected)
    assert projected[2] == projected[3] == projected[4] == 0.


def test_unknown_and_bad_covariance_are_distinct():
    cov, unknown = checked_covariance([0.]*36)
    assert unknown and np.all(np.diag(cov) == 1e6)
    bad = np.eye(6); bad[0,0] = -1
    with pytest.raises(ValueError,match='COVARIANCE'):
        checked_covariance(bad)
    projected = planar_pose_covariance(cov,np.eye(4))
    assert projected[2,2] == projected[3,3] == projected[4,4] == 1e6


def test_pose_derived_rotation_does_not_turn_imu_orbit_into_body_translation():
    e = installation_matrix([.4,.2,.5],[.1,.2,.3])
    estimator = PoseVelocityEstimator(window=.5,min_span=.08)
    result = None
    for i in range(11):
        b = installation_matrix([1,2,.7],[0,0,.5*i*.02])
        result = estimator.update(1000000000+i*20000000,body_pose(b @ e,e))
    assert result is not None
    np.testing.assert_allclose(result.base_twist[:3],np.zeros(3),atol=1e-12)
    assert result.base_twist[5] == pytest.approx(.5)


def test_pose_derived_translation_and_reset():
    estimator = PoseVelocityEstimator()
    assert estimator.update(1000000000,np.eye(4)) is None
    result = estimator.update(1100000000,installation_matrix([.02,.03,0],[0,0,0]))
    np.testing.assert_allclose(result.base_twist,[.2,.3,0,0,0,0],atol=1e-12)
    assert estimator.update(2000000000,np.eye(4)) is None  # gap resets window
    assert estimator.update(1900000000,np.eye(4)) is None  # backward stamp resets


@pytest.mark.parametrize('stamp,now,expected', [
    (0,1000000000,'INVALID_TIMESTAMP'),
    (1200000000,1000000000,'FUTURE_INPUT'),
    (1000000000,2000000000,'STALE_INPUT'),
    (1000000000,0,'CLOCK_UNAVAILABLE')])
def test_stamp_errors(stamp,now,expected):
    assert InputGuard().accept(stamp,now,1.) == expected


def test_order_duplicate_clock_rollback_and_wall_timeout():
    guard = InputGuard()
    assert guard.accept(1000000000,1000000000,1.) is None
    assert guard.accept(1000000000,1100000000,1.1) == 'NON_INCREASING_TIMESTAMP'
    assert guard.accept(900000000,1100000000,1.1) == 'NON_INCREASING_TIMESTAMP'
    assert guard.expired(1100000000,1.6) == 'INPUT_WALL_TIMEOUT'
    assert guard.accept(500000000,500000000,1.7) == 'CLOCK_ROLLBACK'
    assert guard.accept(510000000,510000000,1.8) is None


@pytest.mark.parametrize('values', [[0,0,0,0],[float('nan'),0,0,1],[0,0,0,2]])
def test_invalid_quaternion(values):
    with pytest.raises(ValueError):
        pose_matrix([0,0,0],values)


def test_invalid_configuration_and_yaw_singularity():
    with pytest.raises(ValueError):
        installation_matrix([0,0],[0,0,0])
    with pytest.raises(ValueError):
        InputGuard(max_age=-1)
    with pytest.raises(ValueError):
        PoseVelocityEstimator(window=.01,min_span=.1)
    with pytest.raises(ValueError,match='SINGULAR'):
        planar_pose(installation_matrix([0,0,0],[0,math.pi/2,0]))
