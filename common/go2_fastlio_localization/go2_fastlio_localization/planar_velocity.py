"""Planar body velocity from continuous odometry poses, never map corrections."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarVelocity:
    vx: float
    vy: float
    wz: float
    linear_variance: float
    angular_variance: float


class PlanarVelocityEstimator:
    """Fit pose slopes over a short stamp-based window and rotate into body axes.

    A missing/invalid estimate is None, not a zero-speed observation. Reset on
    clock rewind, frame change, long gaps or implausible pose discontinuities.
    """

    def __init__(self, window=1.0, min_span=0.08, max_gap=0.5,
                 max_linear_speed=3.0, max_angular_speed=6.0):
        values = (window, min_span, max_gap, max_linear_speed, max_angular_speed)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('velocity estimator limits must be finite and positive')
        if min_span >= window:
            raise ValueError('require min_span < window')
        self.window, self.min_span, self.max_gap = window, min_span, max_gap
        self.max_linear_speed, self.max_angular_speed = max_linear_speed, max_angular_speed
        self.reset()

    def reset(self):
        self.samples = deque()
        self.frame = None
        self.previous_yaw = None

    def update(self, stamp_ns, x, y, yaw, frame):
        if stamp_ns <= 0 or not frame or not all(math.isfinite(v) for v in (x, y, yaw)):
            self.reset()
            return None
        if self.samples:
            dt = (stamp_ns-self.samples[-1][0])*1e-9
            if frame != self.frame or dt < 0 or dt > self.max_gap:
                self.reset()
            elif dt < .001:
                # Duplicate/nearly duplicate stamps cannot provide a derivative.
                return None
            else:
                last = self.samples[-1]
                delta_yaw = math.atan2(math.sin(yaw-self.previous_yaw),
                                       math.cos(yaw-self.previous_yaw))
                if (math.hypot(x-last[1], y-last[2])/dt > self.max_linear_speed or
                        abs(delta_yaw)/dt > self.max_angular_speed):
                    self.reset()
        unwrapped_yaw = (self.samples[-1][3] + math.atan2(
            math.sin(yaw-self.previous_yaw), math.cos(yaw-self.previous_yaw))
            if self.samples else yaw)
        self.frame, self.previous_yaw = frame, yaw
        self.samples.append((stamp_ns, x, y, unwrapped_yaw))
        while len(self.samples) > 1 and (stamp_ns-self.samples[0][0])*1e-9 > self.window + 1e-9:
            self.samples.popleft()
        if len(self.samples) < 2 or (stamp_ns-self.samples[0][0])*1e-9 < self.min_span:
            return None
        origin = self.samples[0][0]
        times = [(s[0]-origin)*1e-9 for s in self.samples]
        mean_time = sum(times)/len(times)
        centered = [t-mean_time for t in times]
        denominator = sum(t*t for t in centered)

        def slope(column):
            values = [s[column] for s in self.samples]
            mean = sum(values)/len(values)
            derivative = sum(t*(v-mean) for t, v in zip(centered, values))/denominator
            residual = sum((v-mean-derivative*t)**2 for t, v in zip(centered, values))
            variance = residual/max(1, len(values)-2)/denominator
            return derivative, variance

        vx_world, var_x = slope(1)
        vy_world, var_y = slope(2)
        wz, var_wz = slope(3)
        c, s = math.cos(yaw), math.sin(yaw)
        return PlanarVelocity(c*vx_world+s*vy_world, -s*vx_world+c*vy_world, wz,
                              max(var_x+var_y, .02**2), max(var_wz, .03**2))
