"""ROS-free rigid-body and planar navigation adaptation.

``imu_in_base`` is T_base_imu. Pose covariance uses fixed-axis infinitesimal
rotation in source odom; twist covariance uses [vx,vy,vz,wx,wy,wz] in the
explicitly selected input frame. Unknown/all-zero covariance is not certainty.
The installation transform is treated as calibrated; its uncertainty must be
included upstream or added to the configured conservative covariance.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


def skew(v):
    x, y, z = np.asarray(v, dtype=float)
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('INVALID_QUATERNION')
    norm = float(np.linalg.norm(q))
    if norm < 1e-12 or abs(norm - 1.) > .05:
        raise ValueError('INVALID_QUATERNION_NORM')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rotation_quaternion(rotation):
    # Eigenvector formulation avoids the singular trace branch at pi radians.
    r = np.asarray(rotation, dtype=float)
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                  [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                  [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], r.trace()]]) / 3.
    values, vectors = np.linalg.eigh(k)
    q = vectors[:, np.argmax(values)]
    return q if q[3] >= 0 else -q


def pose_matrix(position, quaternion):
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError('INVALID_POSITION')
    t = np.eye(4)
    t[:3, :3] = quaternion_rotation(quaternion)
    t[:3, 3] = position
    return t


def installation_matrix(xyz, rpy):
    xyz, rpy = np.asarray(xyz, dtype=float), np.asarray(rpy, dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.all(np.isfinite(np.r_[xyz, rpy])):
        raise ValueError('INVALID_INSTALLATION_EXTRINSIC')
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                 [-sp, cp*sr, cp*cr]]
    t[:3, 3] = xyz
    return t


def inverse_rigid(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ t[:3, 3]
    return result


def yaw_gradient(rotation):
    # d(yaw)/d(world-fixed infinitesimal rotation), including body tilt.
    r = rotation
    denominator = r[0,0]**2 + r[1,0]**2
    if denominator < 1e-6:
        raise ValueError('YAW_PROJECTION_SINGULAR')
    return np.array([-r[2,0]*r[0,0]/denominator,
                     -r[2,0]*r[1,0]/denominator, 1.])


def planar_pose(t, ground_z=0.):
    yaw_gradient(t[:3, :3])
    yaw = math.atan2(t[1,0], t[0,0])
    return installation_matrix([t[0,3], t[1,3], ground_z], [0., 0., yaw])


def body_pose(odom_imu, imu_in_base):
    return odom_imu @ inverse_rigid(imu_in_base)


def navigation_transforms(odom_base, map_source=None, ground_z=0.):
    """Return local footprint, footprint-body, map-nav_odom.

    map-nav_odom is horizontal. The footprint-body transform is computed from
    the GLOBAL body pose when available, preserving both global planar Nav2
    position and exact global 3D base_link pose even when map-source has tilt.
    With missing global TF, only the independently usable local branch exists.
    """
    local_footprint = planar_pose(odom_base, ground_z)
    if map_source is None:
        return local_footprint, inverse_rigid(local_footprint) @ odom_base, None
    global_base = map_source @ odom_base
    global_footprint = planar_pose(global_base, ground_z)
    return (local_footprint,
            inverse_rigid(global_footprint) @ global_base,
            global_footprint @ inverse_rigid(local_footprint))


def checked_covariance(values, unknown_variance=1e6):
    c = np.asarray(values, dtype=float)
    if c.size != 36 or not np.all(np.isfinite(c)):
        raise ValueError('INVALID_COVARIANCE')
    c = c.reshape(6, 6)
    if np.all(c == 0.):
        return np.eye(6)*unknown_variance, True
    if not np.allclose(c, c.T, atol=1e-7) or np.linalg.eigvalsh(c).min() < -1e-8:
        raise ValueError('INVALID_COVARIANCE_PSD')
    return .5*(c+c.T), False


def transform_pose_covariance(covariance, odom_imu, odom_base):
    j = np.eye(6)
    j[:3, 3:] = -skew(odom_base[:3,3]-odom_imu[:3,3])
    return j @ covariance @ j.T


def planar_pose_covariance(covariance, odom_base, unknown_variance=1e6):
    j = np.zeros((6,6))
    j[0,0] = j[1,1] = 1.
    j[5,3:] = yaw_gradient(odom_base[:3,:3])
    result = j @ covariance @ j.T
    # A 2D projection does not measure physical height/roll/pitch.
    for index in (2, 3, 4):
        result[index,index] = unknown_variance
    return result


def transform_twist(linear, angular, odom_base, imu_in_base, convention):
    """Velocity at IMU origin -> velocity at base origin, expressed in base.

    child: both vectors in IMU; world: both vectors in source odom.
    Input pose-derived mode must not invoke this with FAST-LIO default zeros.
    """
    if convention == 'child':
        rotation = imu_in_base[:3,:3]
    elif convention == 'world':
        rotation = odom_base[:3,:3].T
    else:
        raise ValueError('UNKNOWN_TWIST_CONVENTION')
    values = np.r_[linear, angular].astype(float)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError('INVALID_TWIST')
    j = np.zeros((6,6))
    j[:3,:3] = j[3:,3:] = rotation
    j[:3,3:] = skew(imu_in_base[:3,3]) @ rotation
    return j @ values, j


def planar_twist(base_twist, odom_base, local_footprint):
    j = np.zeros((6,6))
    rotation = odom_base[:3,:3]
    j[:2,:3] = (local_footprint[:3,:3].T @ rotation)[:2,:]
    j[5,3:] = yaw_gradient(rotation) @ rotation
    return j @ base_twist, j


def rotation_vector(rotation):
    q = rotation_quaternion(rotation)
    sine = np.linalg.norm(q[:3])
    if sine < 1e-10:
        return 2*q[:3]
    return q[:3] * (2*math.atan2(sine, q[3])/sine)


@dataclass
class DerivedVelocity:
    base_twist: np.ndarray
    covariance: np.ndarray


class PoseVelocityEstimator:
    """Windowed full-3D pose slopes; unknown warm-up never becomes zero twist."""
    def __init__(self, window=.3, min_span=.08, max_gap=.5,
                 max_linear_speed=5., max_angular_speed=10.,
                 linear_variance=.25, angular_variance=.25):
        values = [window, min_span, max_gap, max_linear_speed,
                  max_angular_speed, linear_variance, angular_variance]
        if not all(math.isfinite(v) and v > 0 for v in values) or min_span > window:
            raise ValueError('INVALID_VELOCITY_PARAMETERS')
        self.window, self.min_span, self.max_gap = window, min_span, max_gap
        self.max_linear_speed, self.max_angular_speed = max_linear_speed, max_angular_speed
        self.linear_variance, self.angular_variance = linear_variance, angular_variance
        self.samples = deque()
        self.reason = 'VELOCITY_WARMUP'

    def reset(self):
        self.samples.clear()
        self.reason = 'VELOCITY_WARMUP'

    def update(self, stamp_ns, odom_base):
        if self.samples:
            gap = (stamp_ns-self.samples[-1][0])*1e-9
            if gap <= 0 or gap > self.max_gap:
                self.reset()
        self.samples.append((stamp_ns, odom_base.copy()))
        while len(self.samples) > 2 and (stamp_ns-self.samples[0][0])*1e-9 > self.window:
            self.samples.popleft()
        if len(self.samples) < 2 or (stamp_ns-self.samples[0][0])*1e-9 < self.min_span:
            return None
        stamps = np.array([(t-stamp_ns)*1e-9 for t, _ in self.samples])
        times = stamps-stamps.mean()
        denominator = float(times @ times)
        positions = np.array([t[:3,3] for _,t in self.samples])
        first_rotation = self.samples[0][1][:3,:3]
        angles = np.array([rotation_vector(t[:3,:3] @ first_rotation.T)
                           for _,t in self.samples])
        velocity = times @ positions / denominator
        omega = times @ angles / denominator
        if np.linalg.norm(velocity) > self.max_linear_speed or np.linalg.norm(omega) > self.max_angular_speed:
            self.reset()
            self.reason = 'VELOCITY_OUT_OF_RANGE'
            return None
        r = odom_base[:3,:3].T
        twist = np.r_[r @ velocity, r @ omega]
        # Conservative floor, enlarged by regression residuals; not calibrated
        # sensor accuracy. Height/attitude noise is retained before projection.
        pos_residual = positions-positions.mean(axis=0)-times[:,None]*velocity
        ang_residual = angles-angles.mean(axis=0)-times[:,None]*omega
        pos_var = self.linear_variance + float(np.sum(pos_residual**2))/denominator
        ang_var = self.angular_variance + float(np.sum(ang_residual**2))/denominator
        self.reason = 'POSE_DERIVED_VELOCITY'
        return DerivedVelocity(twist, np.diag([pos_var]*3+[ang_var]*3))


class InputGuard:
    """ROS-time input validation plus monotonic receipt watchdog."""
    def __init__(self, max_age=.5, future_tolerance=.05, wall_timeout=.5):
        if not all(math.isfinite(v) for v in [max_age, future_tolerance, wall_timeout]) or max_age <= 0 or future_tolerance < 0 or wall_timeout <= 0:
            raise ValueError('INVALID_TIME_PARAMETERS')
        self.max_age, self.future_tolerance, self.wall_timeout = max_age, future_tolerance, wall_timeout
        self.last_stamp = None
        self.last_ros = None
        self.last_wall = None

    def clock(self, now_ns):
        if now_ns <= 0:
            return 'CLOCK_UNAVAILABLE'
        previous = self.last_ros
        self.last_ros = now_ns
        if previous is not None and now_ns < previous:
            self.last_stamp = None
            self.last_wall = None
            return 'CLOCK_ROLLBACK'
        return None

    def accept(self, stamp_ns, now_ns, wall_now):
        error = self.clock(now_ns)
        if error:
            return error
        if stamp_ns <= 0:
            return 'INVALID_TIMESTAMP'
        age = (now_ns-stamp_ns)*1e-9
        if age < -self.future_tolerance:
            return 'FUTURE_INPUT'
        if age > self.max_age:
            return 'STALE_INPUT'
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            return 'NON_INCREASING_TIMESTAMP'
        self.last_stamp, self.last_wall = stamp_ns, wall_now
        return None

    def expired(self, now_ns, wall_now):
        error = self.clock(now_ns)
        if error:
            return error
        if self.last_wall is None:
            return 'WAITING_FOR_INPUT'
        if wall_now-self.last_wall > self.wall_timeout:
            return 'INPUT_WALL_TIMEOUT'
        if (now_ns-self.last_stamp)*1e-9 > self.max_age:
            return 'STALE_INPUT'
        return None
