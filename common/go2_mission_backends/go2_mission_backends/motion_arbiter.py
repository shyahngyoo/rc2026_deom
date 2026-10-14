"""Enforced command gate, compatible Mission services, measured state backend."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from go2_obstacle_interfaces.msg import ControlState, MotionAuthority, MotionCommand, MotionTelemetry
from go2_obstacle_interfaces.srv import SetControlMode, GetMotionState
from robocon_mission.stop_monitor import StopMonitor
from .arbiter_core import Arbiter, Switch

class MotionArbiter(Node):
    def __init__(self, **kwargs):
        super().__init__('motion_arbiter', **kwargs)
        defaults = dict(nav_input='/motion/nav_command', skill_input='/motion/skill_command',
            output_topic='/backend_sim/cmd_vel', telemetry_topic='/motion/telemetry',
            enable_output=False, simulation_mode=False, state_timeout=0.5,
            command_timeout=0.5, max_command_age=0.3, max_linear_speed=0.5,
            max_angular_speed=1.0, stable_duration=0.15,
            stable_linear_velocity=0.03, stable_angular_velocity=0.05,
            telemetry_source_id='', base_frame='base_footprint', odom_frame='nav_odom', output_frequency=20.0)
        for k,v in defaults.items(): self.declare_parameter(k,v)
        p=lambda k:self.get_parameter(k).value
        if len({p('nav_input'),p('skill_input'),p('output_topic')}) != 3:
            raise ValueError('command input/output topics must be distinct')
        if p('simulation_mode') and not p('output_topic').startswith('/backend_sim/'):
            raise ValueError('simulation output must remain under /backend_sim/')
        for k in ('state_timeout','output_frequency','stable_duration'):
            if not math.isfinite(p(k)) or p(k)<=0: raise ValueError('invalid '+k)
        self.enabled=p('enable_output');self.output_topic=p('output_topic')
        self.bypass_detected=False;self.next_publisher_check=0.0
        self.state_timeout=p('state_timeout')
        self.telemetry_source=p('telemetry_source_id')
        self.base_frame=p('base_frame');self.odom_frame=p('odom_frame')
        self.core=Arbiter(p('command_timeout'),p('max_command_age'),p('max_linear_speed'),p('max_angular_speed'))
        self.telemetry=None; self.telemetry_at=0.0; self.last_telemetry_seq=0; self.last_telemetry_stamp=0
        self.odom_stamp=0; self.odom_at=0.0; self.last_ros=None
        self.stop=StopMonitor(self,self.state_timeout,p('stable_linear_velocity'),p('stable_angular_velocity'),p('stable_duration'))
        self.output_pub=self.create_publisher(Twist,p('output_topic'),1)
        self.control_pub=self.create_publisher(ControlState,'/motion/control_state',10)
        self.authority_pub=self.create_publisher(MotionAuthority,'/motion/authority',10)
        self.status_pub=self.create_publisher(String,'/motion/status',10)
        self.create_subscription(MotionTelemetry,p('telemetry_topic'),self._telemetry,10)
        self.create_subscription(Odometry,'/odom',self._odom,20)
        self.create_subscription(MotionCommand,p('nav_input'),lambda m:self._command('NAVIGATION',m),1)
        self.create_subscription(MotionCommand,p('skill_input'),lambda m:self._command('SKILL',m),1)
        self.create_service(SetControlMode,'/motion/set_control_mode',self._switch)
        self.create_service(GetMotionState,'/motion/get_state',self._state)
        self.create_service(Trigger,'/motion/emergency_hold',self._emergency)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=self.create_timer(1/p('output_frequency'),self._tick,clock=self.steady)

    def _now(self): return self.get_clock().now().nanoseconds
    @staticmethod
    def _stamp(header):
        if header.stamp.sec < 0 or not 0 <= header.stamp.nanosec < 1000000000:
            return -1
        return header.stamp.sec*1000000000+header.stamp.nanosec
    def _odom(self,m):
        stamp=self._stamp(m.header)
        vals=(m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z,
              m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z)
        if m.header.frame_id!=self.odom_frame or m.child_frame_id!=self.base_frame or stamp<=self.odom_stamp or not all(math.isfinite(v) for v in vals):
            self.odom_at=0.0; self.core.hold('ODOMETRY_INVALID'); return
        self.odom_stamp,self.odom_at=stamp,time.monotonic()
    def _telemetry(self,m):
        stamp=self._stamp(m.header); age=(self._now()-stamp)/1e9
        if not self.telemetry_source: self.telemetry_source=m.source_id
        if (not m.source_id or m.source_id!=self.telemetry_source or m.sequence<=self.last_telemetry_seq
            or stamp<=self.last_telemetry_stamp or stamp<=0 or not 0<=age<=self.state_timeout):
            self.telemetry=None; self.core.hold('TELEMETRY_INVALID'); return
        self.last_telemetry_seq=m.sequence;self.last_telemetry_stamp=stamp
        self.telemetry,self.telemetry_at=m,time.monotonic()
        if m.nav_active and m.skill_active:
            self.telemetry=None;self.core.hold('CONTROL_CONFLICT');return
        if not m.known or m.error_code:
            self.core.hold('LOW_LEVEL_UNKNOWN_OR_FAULT')
    def fresh(self):
        now=time.monotonic(); ros=self._now(); t=self.telemetry
        return bool(self.enabled and not self.bypass_detected and t and t.known and not t.error_code
          and now-self.telemetry_at<=self.state_timeout and now-self.odom_at<=self.state_timeout
          and 0<self.odom_stamp<=ros and (ros-self.odom_stamp)/1e9<=self.state_timeout
          and 0<= (ros-self._stamp(t.header))/1e9<=self.state_timeout)
    def active(self):
        t=self.telemetry
        return bool(t and ((self.core.owner=='NAVIGATION' and t.nav_active) or
                           (self.core.owner=='SKILL' and t.skill_active)))
    def safe_switch(self):
        t=self.telemetry
        return bool(self.fresh() and self.stop.ready() and t.stopped and
                    not t.nav_active and not t.skill_active and t.control_releasable and t.safe_stance)
    def _command(self,source,m):
        if m.header.frame_id != self.base_frame:
            self.core.reason='COMMAND_FRAME_MISMATCH';return
        v=m.twist
        accepted=self.core.receive(source,m.owner,m.mission_id,m.generation,self._stamp(m.header),
            (v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z),
            time.monotonic(),self._now(),self.fresh(),self.active())
        if not accepted: self.get_logger().debug(self.core.reason)
    def _switch(self,req,res):
        if not self.enabled and req.owner!='SAFE_HOLD':
            res.accepted=False; res.owner=self.core.owner; res.mission_id=self.core.mission_id
            res.request_id=req.request_id; res.message='OUTPUT_DISABLED'; return res
        d=self.core.switch(Switch(req.owner,req.expected_owner,req.mission_id,req.request_id,req.lease_duration),
                           time.monotonic(),self._now(),self.safe_switch(),self.fresh())
        res.accepted,res.owner,res.mission_id,res.request_id,res.message=d.accepted,d.owner,d.mission_id,d.request_id,d.reason
        self._tick(); return res
    def _state(self,req,res):
        t=self.telemetry
        matches=bool(t and (not req.obstacle_id or t.obstacle_id==req.obstacle_id))
        res.known=bool(self.fresh() and matches)
        res.obstacle_id=t.obstacle_id if t else ''
        res.skill_active=t.skill_active if t else False
        res.safe_stance=bool(res.known and t.safe_stance)
        res.control_releasable=bool(res.known and t.safe_stance and t.control_releasable and not t.skill_active and not t.nav_active
                                    and self.stop.ready() and t.stopped)
        res.error_code=t.error_code if res.known else 'LOW_LEVEL_STATE_UNCONFIRMED'
        res.message='measured backend state' if res.known else 'stale/unknown/mismatched backend state'
        return res
    def _emergency(self,_req,res):
        self.core.hold('EMERGENCY_FENCE');self._tick()
        res.success=True;res.message='commands fenced; physical support/estop not asserted';return res
    def _tick(self):
        wall,ros=time.monotonic(),self._now()
        if self.last_ros is not None and ros < self.last_ros:
            self.odom_stamp=0;self.odom_at=0.0;self.last_telemetry_stamp=0;self.telemetry=None;self.stop.reset()
        self.last_ros=ros
        if wall>=self.next_publisher_check:
            self.next_publisher_check=wall+.2
            infos=self.get_publishers_info_by_topic(self.output_topic)
            if len(infos)>1 or any(i.node_name!=self.get_name() for i in infos):
                self.bypass_detected=True;self.core.hold('OUTPUT_BYPASS_DETECTED')
        values=self.core.output(wall,ros,self.fresh(),self.active())
        msg=Twist();msg.linear.x,msg.linear.y,msg.linear.z=values[:3]
        msg.angular.x,msg.angular.y,msg.angular.z=values[3:]
        self.output_pub.publish(msg)
        stamp=self.get_clock().now().to_msg()
        cs=ControlState(stamp=stamp,owner=self.core.owner,mission_id=self.core.mission_id,healthy=self.fresh())
        self.control_pub.publish(cs)
        a=MotionAuthority();a.header.stamp=stamp;a.owner=self.core.owner;a.mission_id=self.core.mission_id
        a.generation=self.core.generation;a.lease_remaining=max(0.0,self.core.deadline-wall)
        a.healthy=self.fresh();a.reason=self.core.reason;self.authority_pub.publish(a)
        data=dict(owner=a.owner,mission_id=a.mission_id,generation=a.generation,
            lease_remaining=a.lease_remaining,healthy=a.healthy,reason=a.reason,
            measured_stopped=self.stop.ready(),source_active=self.active(),output=list(values),
            output_enabled=self.enabled,bypass_detected=self.bypass_detected)
        self.status_pub.publish(String(data=json.dumps(data)))

def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);node=MotionArbiter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.core.hold('NODE_SHUTDOWN');node._tick();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
