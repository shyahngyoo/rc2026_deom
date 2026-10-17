"""Map only fresh, measured STATUS into telemetry. Arbiter owns state services."""
import json
import math
import time

import rclpy
from go2_obstacle_interfaces.msg import MotionTelemetry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String


BOOL_FIELDS = ('known', 'stopped', 'safe_stance', 'control_releasable',
               'nav_active', 'skill_active', 'estop', 'human_enable')
MODES = ('HOLD', 'NAVIGATION', 'SKILL', 'DISABLED')


def measured_status(snapshot):
    if snapshot.get('connected') is not True or snapshot.get('handshake') is not True:
        raise ValueError('CONTROLLER_DISCONNECTED_OR_UNCONFIRMED')
    status = snapshot.get('status')
    if not isinstance(status, dict) or any(type(status.get(k)) is not bool for k in BOOL_FIELDS):
        raise ValueError('INVALID_MEASURED_STATUS')
    if status.get('mode') not in MODES or not isinstance(status.get('fault_code'), str):
        raise ValueError('INVALID_MODE_OR_FAULT')
    if not isinstance(status.get('obstacle_id', ''), str):
        raise ValueError('INVALID_OBSTACLE_ID')
    if status['nav_active'] and status['skill_active']:
        raise ValueError('CONTROL_CONFLICT')
    if status['nav_active'] and status['mode'] != 'NAVIGATION':
        raise ValueError('NAV_MODE_MISMATCH')
    if status['skill_active'] and status['mode'] != 'SKILL':
        raise ValueError('SKILL_MODE_MISMATCH')
    return status


class TelemetryAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('stm32_telemetry_adapter', **kwargs)
        for key, value in dict(input_topic='/stm32/status', output_topic='/motion/telemetry',
                source_id='stm32', base_frame='base_footprint', status_timeout=0.3).items():
            self.declare_parameter(key, value)
        self.timeout = self.get_parameter('status_timeout').value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('invalid status_timeout')
        self.source_id = self.get_parameter('source_id').value
        if not self.source_id:
            raise ValueError('source_id required')
        self.sequence = self.gateway_sequence = self.last_stamp = 0
        self.at = 0.0
        self.valid = False
        self.last_ros = None
        self.pub = self.create_publisher(MotionTelemetry, self.get_parameter('output_topic').value, 10)
        self.create_subscription(String, self.get_parameter('input_topic').value, self._status, 10)
        self.create_timer(0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _publish(self, status=None, error='', stamp_ns=None):
        self.sequence += 1
        msg = MotionTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        if stamp_ns is not None:
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
        msg.header.frame_id = self.get_parameter('base_frame').value
        msg.source_id, msg.sequence = self.source_id, self.sequence
        msg.known = bool(status and status['known'] and not error)
        # Invalid state cannot assert measured stop, support or release.
        if msg.known:
            for key in ('nav_active', 'skill_active', 'stopped', 'safe_stance', 'control_releasable'):
                setattr(msg, key, status[key])
            msg.obstacle_id = status.get('obstacle_id', '')
        msg.error_code = error or ('' if msg.known else 'LOW_LEVEL_STATE_UNCONFIRMED')
        self.pub.publish(msg)

    def _status(self, message):
        try:
            if len(message.data) > 16384:
                raise ValueError('STATUS_TOO_LARGE')
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('INVALID_SNAPSHOT')
            sequence, stamp = data.get('gateway_sequence'), data.get('stamp_ns')
            now = self.get_clock().now().nanoseconds
            if type(sequence) is not int or sequence <= self.gateway_sequence:
                raise ValueError('REPLAYED_GATEWAY_STATUS')
            self.gateway_sequence = sequence
            if type(stamp) is not int or stamp <= self.last_stamp or stamp <= 0 or not 0 <= (now-stamp)/1e9 <= self.timeout:
                raise ValueError('STALE_GATEWAY_STATUS')
            self.last_stamp = stamp
            status = measured_status(data)
            error = data.get('error_code', '') or status['fault_code']
            if not isinstance(error, str):
                raise ValueError('INVALID_FAULT_CODE')
            if status['estop']:
                error = 'PHYSICAL_ESTOP'
            elif not status['human_enable']:
                error = 'HUMAN_ENABLE_REQUIRED'
            self.at, self.valid = time.monotonic(), not error and status['known']
            self._publish(status, error, stamp)
        except (ValueError, TypeError, KeyError) as error:
            self.valid = False
            self._publish(error=str(error))

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros is not None and now < self.last_ros:
            self.valid = False
            self.last_stamp = 0
        self.last_ros = now
        if not self.valid or time.monotonic()-self.at > self.timeout or not 0 <= (now-self.last_stamp)/1e9 <= self.timeout:
            self.valid = False
            self._publish(error='STATUS_STALE_OR_UNAVAILABLE')


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(error='TELEMETRY_ADAPTER_SHUTDOWN')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
