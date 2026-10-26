"""Single serial owner for CyberGear key-based commissioning and automation."""
import json
import math
import time
import yaml
import serial
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from .cybergear_core import StatusParser, Runner, encode_key, validate_plan


class CyberGearNode(Node):
    def __init__(self):
        super().__init__('cybergear_bridge')
        defaults = dict(port='', baudrate=115200, plan_file='', enable_motion=False,
                        status_timeout=0.5, pose_timeout=0.5, distance_tolerance=0.12,
                        heading_tolerance=0.15, key_interval=0.2,
                        map_frame='map', base_frame='base_footprint')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        for key in ('status_timeout', 'pose_timeout', 'distance_tolerance', 'heading_tolerance', 'key_interval'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(key + ' must be finite and positive')
        self.runner = None
        if self.cfg['plan_file']:
            with open(self.cfg['plan_file'], encoding='utf-8') as f:
                plan = yaml.safe_load(f)
                steps = validate_plan(plan)
                if plan.get('frame'):
                    self.cfg['map_frame'] = plan['frame']
            self.runner = Runner(steps, self.cfg['distance_tolerance'],
                                 self.cfg['heading_tolerance'], self.cfg['key_interval'])
        self.link = None
        self.parser = StatusParser()
        self.status = None
        self.status_time = -math.inf
        self.next_connect = 0.0
        self.manual_mode = None
        self.manual_deadline = 0.0
        self.pub = self.create_publisher(String, '/cybergear/status', 10)
        self.create_subscription(String, '/cybergear/key', self.on_key, 10)
        self.create_service(Trigger, '/cybergear/start', self.start)
        self.create_service(Trigger, '/cybergear/stop', self.stop)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_timer(0.02, self.tick)
        self.last_publish = 0.0
        self.get_logger().info('CyberGear key mode; remote must enable AUTO. No v1 handshake. Motion=' + str(self.cfg['enable_motion']))

    def ready(self, now):
        return bool(self.link and self.status and now-self.status_time <= self.cfg['status_timeout']
                    and self.status['mode'] != 'z' and self.status['fault'] == 0)

    def send(self, key):
        if not self.cfg['enable_motion'] or not self.link:
            return
        packet = encode_key(key)
        if self.link.write(packet) != len(packet):
            raise serial.SerialException('short serial write; command not retried')

    def on_key(self, msg):
        try:
            key = json.loads(msg.data)
            encode_key(key)
            if not self.cfg['enable_motion']:
                raise ValueError('enable_motion is false')
            if key == 5:
                if self.runner:
                    self.runner.stop()
                self.manual_mode = None
                self.send(5)
                return
            if self.runner and self.runner.active:
                raise ValueError('task running; stop it before manual keys')
            if not self.ready(time.monotonic()):
                raise ValueError('waiting for initialized, fault-free AUTO status')
            from .cybergear_core import MODE_KEYS
            if self.manual_mode is not None:
                raise ValueError('waiting for mode feedback; see /cybergear/status')
            self.send(key)
            if key in MODE_KEYS:
                self.manual_mode = MODE_KEYS[key]
                self.manual_deadline = time.monotonic() + 5.0
                self.mode_sent = time.monotonic()
        except (ValueError, serial.SerialException, OSError) as exc:
            self.get_logger().error(str(exc))
            if isinstance(exc, (serial.SerialException, OSError)):
                self.disconnect()

    def start(self, request, response):
        if not self.cfg['enable_motion']:
            response.message = 'enable_motion is false'
        elif not self.runner:
            response.message = 'plan_file is empty'
        elif self.runner.active:
            response.message = 'task already running'
        elif self.manual_mode is not None:
            response.message = 'waiting for manual mode feedback'
        elif not self.ready(time.monotonic()):
            response.message = 'waiting for initialized, fault-free AUTO status'
        else:
            self.runner.start()
            response.success = True
            response.message = 'started from step 0'
        return response

    def stop(self, request, response):
        if self.runner:
            self.runner.stop()
        self.manual_mode = None
        try:
            self.send(5)
            response.success = True
            response.message = 'task stopped; key 5 attempted (not a physical stop acknowledgement)'
        except (serial.SerialException, OSError) as exc:
            response.message = str(exc)
            self.disconnect()
        return response

    def disconnect(self):
        if self.runner:
            self.runner.stop('serial_disconnected')
        if self.link:
            self.link.close()
        self.link = None
        self.status = None
        self.status_time = -math.inf
        self.manual_mode = None
        self.parser = StatusParser()
        self.next_connect = time.monotonic() + 1.0

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.cfg['map_frame'], self.cfg['base_frame'], Time())
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if not 0 <= age <= self.cfg['pose_timeout']:
                return None
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            result = (t.transform.translation.x, t.transform.translation.y, yaw)
            return result if all(math.isfinite(v) for v in result) else None
        except Exception:
            return None

    def tick(self):
        now = time.monotonic()
        try:
            if not self.link and now >= self.next_connect:
                if not self.cfg['port']:
                    self.next_connect = now + 1
                else:
                    self.link = serial.Serial(self.cfg['port'], self.cfg['baudrate'], timeout=0,
                                              write_timeout=0.05, exclusive=True)
                    self.link.reset_input_buffer()
            if self.link:
                for status in self.parser.feed(self.link.read(min(4096, self.link.in_waiting))):
                    self.status, self.status_time = status, now
                if self.manual_mode is not None:
                    if (self.status and self.status_time > self.mode_sent
                            and self.status['mode'] == self.manual_mode):
                        self.manual_mode = None
                    elif now > self.manual_deadline or not self.ready(now):
                        self.manual_mode = None
                        self.get_logger().error('manual mode confirmation failed')
                if self.runner and self.runner.active:
                    if not self.ready(now):
                        keys = self.runner.stop('status_unavailable_or_fault')
                    else:
                        keys = self.runner.tick(now, self.status, self.status_time, self.pose())
                    for key in keys:
                        self.send(key)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(str(exc))
            self.disconnect()
        if now - self.last_publish >= 0.1:
            self.last_publish = now
            self.pub.publish(String(data=json.dumps(dict(connected=self.link is not None,
                ready=self.ready(now), motion_enabled=self.cfg['enable_motion'],
                status=self.status, status_age_s=round(now-self.status_time, 3) if self.status else None,
                pending_mode=self.manual_mode, task=self.runner.state if self.runner else 'no_plan',
                step=self.runner.index if self.runner else None,
                obstacle=(self.runner.steps[self.runner.index].get('obstacle','') if self.runner and self.runner.index<len(self.runner.steps) else ''),
                action=(self.runner.steps[self.runner.index].get('label','') if self.runner and self.runner.index<len(self.runner.steps) else '')), allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CyberGearNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try:
                node.send(5)
            except (serial.SerialException, OSError):
                pass
            node.disconnect()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
