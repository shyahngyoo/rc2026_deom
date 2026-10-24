"""Fixed map/odom alignment and continuity checks; never estimates a map match."""
import math
import numpy as np
from .robot_state import InputGuard, installation_matrix, pose_matrix, rotation_vector


class OdometryLocalization:
    def __init__(self, odom_in_map, max_age=.5, max_linear_speed=5., max_angular_speed=10.):
        if len(odom_in_map) != 6:
            raise ValueError('odom_in_map requires measured XYZ/RPY')
        self.map_odom = installation_matrix(odom_in_map[:3], odom_in_map[3:])
        if not all(math.isfinite(v) and v > 0 for v in (max_linear_speed, max_angular_speed)):
            raise ValueError('invalid continuity limits')
        self.guard = InputGuard(max_age, .05, max_age)
        self.max_linear_speed, self.max_angular_speed = max_linear_speed, max_angular_speed
        self.previous = None
        self.fault = ''

    def reject(self, reason):
        # A broken odometry session cannot be repaired by silently accepting a
        # new origin. Restart the full stack after checking the fixed alignment.
        self.fault = self.fault or reason
        raise ValueError(self.fault)

    def check(self, now_ns, wall_now):
        if self.fault:
            return self.fault
        reason = self.guard.expired(now_ns, wall_now)
        if reason and self.previous is not None:
            self.fault = reason
        return reason

    def observe(self, stamp_ns, now_ns, wall_now, position, quaternion):
        if self.fault:
            self.reject(self.fault)
        if self.previous is not None:
            reason = self.check(now_ns, wall_now)
            if reason:
                self.reject(reason)
        reason = self.guard.accept(stamp_ns, now_ns, wall_now)
        if reason:
            self.reject(reason)
        try:
            pose = pose_matrix(position, quaternion)
        except ValueError as error:
            self.reject(str(error))
        if self.previous is not None:
            last_stamp, last_pose = self.previous
            dt = (stamp_ns-last_stamp)*1e-9
            linear = np.linalg.norm(pose[:3, 3]-last_pose[:3, 3])/dt
            angular = np.linalg.norm(rotation_vector(pose[:3, :3] @ last_pose[:3, :3].T))/dt
            if linear > self.max_linear_speed or angular > self.max_angular_speed:
                self.reject('ODOMETRY_DISCONTINUITY')
        self.previous = stamp_ns, pose
        return self.map_odom @ pose
