"""CyberGear 2026-10-26 key protocol and deterministic task runner (no ROS)."""
import math

MODE_KEYS = {9: 'a', 10: 'c', 11: 'd', 12: 'e', 13: 'f',
             14: 'g', 15: 'h', 16: 'j', 17: 'k', 18: 'b'}
ACTION_KEYS = {1, 2, 3, 4, 5, 6, 7, 8, 19, 20}


def encode_key(key):
    if type(key) is not int or key not in ACTION_KEYS | MODE_KEYS.keys():
        raise ValueError('key must be 1..20; startup/auto keys are remote-only')
    return bytes((0x53, key, 0x45))


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (7 if crc & 128 else 0)) & 255
    return crc


class StatusParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 8:
            f = self.buffer[:8]
            # USART3 has no remote AUTO_FLAG in the mode byte.
            if (f[:2] == b'\xa5\x5a' and f[7] == 13
                    and (f[2] == ord('z') or ord('a') <= f[2] <= ord('m'))
                    and f[3] <= 15 and crc8(f[:6]) == f[6]):
                result.append(dict(mode=chr(f[2]), fault=f[3], motor_id=f[4], detail=f[5]))
                del self.buffer[:8]
            else:
                del self.buffer[0]
        return result


def number(value, name, positive=False):
    if type(value) not in (float, int) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError('invalid ' + name)
    return float(value)


def validate_plan(data):
    if not isinstance(data, dict) or not isinstance(data.get('steps'), list) or not data['steps']:
        raise ValueError('plan needs a nonempty steps list')
    steps = []
    for raw in data['steps']:
        if not isinstance(raw, dict):
            raise ValueError('step must be a mapping')
        s = dict(raw)
        kind = s.get('type')
        allowed = {'mode': {'type', 'key', 'timeout_s'},
                   'key': {'type', 'key', 'duration_s'},
                   'wait': {'type', 'duration_s'},
                   'waypoint': {'type', 'x', 'y', 'yaw', 'timeout_s', 'mode_key', 'reverse', 'heading'},
                   'local_waypoint': {'type','origin','target','mode_key','timeout_s','reverse','heading','require_yaw'}}
        if kind not in allowed or set(s) - allowed[kind] - {'obstacle','label'}:
            raise ValueError('unknown step type or field: ' + str(s))
        if kind in ('mode', 'key'):
            encode_key(s.get('key'))
            if s['key'] not in (MODE_KEYS if kind == 'mode' else ACTION_KEYS):
                raise ValueError('wrong key for step type')
        if kind in ('key', 'wait'):
            s['duration_s'] = number(s.get('duration_s'), 'duration_s', True)
        if kind in ('mode', 'waypoint', 'local_waypoint'):
            s['timeout_s'] = number(s.get('timeout_s', 5 if kind == 'mode' else 60), 'timeout_s', True)
        if kind == 'local_waypoint':
            if not isinstance(s.get('origin'),list) or len(s['origin'])!=3:
                raise ValueError('local waypoint requires origin [x,y,yaw radians]')
            s['origin']=[number(v,'origin') for v in s['origin']]
            if not isinstance(s.get('target'),list) or len(s['target'])!=3:
                raise ValueError('local waypoint requires target [x,y,yaw radians]')
            s['target']=[None if v is None else number(v,'target') for v in s['target']]
            if 'require_yaw' in s and type(s['require_yaw']) is not bool:raise ValueError('require_yaw must be bool')
        if kind == 'waypoint':
            for k in ('x', 'y'):
                s[k] = number(s.get(k), k)
            if 'yaw' in s:
                s['yaw'] = number(s['yaw'], 'yaw')
        if kind in ('waypoint','local_waypoint'):
            if 'reverse' in s and type(s['reverse']) is not bool:raise ValueError('reverse must be bool')
            if 'heading' in s:number(s['heading'],'heading',True)
            s.setdefault('mode_key', 9)
            if type(s['mode_key']) is not int or s['mode_key'] not in MODE_KEYS:
                raise ValueError('invalid waypoint mode_key')
        steps.append(s)
    return steps


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Runner:
    """Returns keys; key steps transmit once, completion is timing/pose, never ACK."""
    def __init__(self, steps, distance=0.12, heading=0.15, interval=0.2):
        self.steps = steps
        self.distance, self.heading, self.interval = distance, heading, interval
        self.active = False
        self.index = 0
        self.entered = None
        self.last_key = None
        self.last_send = -math.inf
        self.state = 'idle'
        self.mode_wait = False

    def start(self):
        self.active = True
        self.index = 0
        self.entered = None
        self.last_key = None
        self.last_send = -math.inf
        self.state = 'running'

    def stop(self, reason='stopped'):
        self.active = False
        self.state = reason
        return [5]

    def tick(self, now, status, status_time, pose=None):
        if not self.active:
            return []
        if self.index >= len(self.steps):
            return self.stop('completed')
        s = self.steps[self.index]
        first = self.entered is None
        if first:
            self.entered = now
            self.last_key = None
            self.mode_wait = s['type'] in ('waypoint','local_waypoint')
            self.resolved = None
        elapsed = now - self.entered
        kind = s['type']
        if kind in ('mode', 'waypoint', 'local_waypoint') and elapsed > s['timeout_s']:
            return self.stop('step_timeout')
        if kind == 'mode':
            if first:
                return [s['key']]
            if status_time > self.entered and status['mode'] == MODE_KEYS[s['key']]:
                self.advance()
            return []
        if kind in ('key', 'wait'):
            if elapsed >= s['duration_s']:
                self.advance()
                return []
            return [s['key']] if first and kind == 'key' else []
        if first:
            return [s['mode_key']]
        if self.mode_wait:
            if status_time <= self.entered or status['mode'] != MODE_KEYS[s['mode_key']]:
                return []
            self.mode_wait = False
        if status['mode'] != MODE_KEYS[s['mode_key']]:
            return self.stop('unexpected_mode')
        if pose is None:
            return self.stop('localization_unavailable')
        x, y, yaw = pose
        if kind == 'local_waypoint':
            if self.resolved is None:
                ox,oy,angle=s['origin']; c,sn=math.cos(angle),math.sin(angle)
                lx=c*(x-ox)+sn*(y-oy);ly=-sn*(x-ox)+c*(y-oy)
                tx,ty,ta=s['target'];tx=lx if tx is None else tx;ty=ly if ty is None else ty
                self.resolved=dict(s,x=ox+c*tx-sn*ty,y=oy+sn*tx+c*ty)
                if s.get('require_yaw',True):self.resolved['yaw']=yaw if ta is None else wrap(angle+ta)
            s=self.resolved
        dx, dy = s['x'] - x, s['y'] - y
        reached = math.hypot(dx, dy) <= self.distance
        heading=s.get('heading',self.heading)
        target_angle=math.atan2(dy,dx)+(math.pi if s.get('reverse',False) else 0)
        error = wrap((s.get('yaw', yaw) if reached else target_angle) - yaw)
        if reached and abs(error) <= heading:
            self.advance()
            return [5]
        key = (1 if error > 0 else 3) if abs(error) > heading else (4 if s.get('reverse',False) else 2)
        if key != self.last_key and now - self.last_send >= self.interval:
            self.last_key, self.last_send = key, now
            return [key]
        return []

    def advance(self):
        self.index += 1
        self.entered = None
