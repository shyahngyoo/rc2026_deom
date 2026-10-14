"""SIMULATION ONLY: kinematic plant, measured sensors and navigation Action.

Pose evolves ONLY from final arbiter output. No Action teleports a target pose.
The test skill is a separate executor; this node owns independent telemetry.
"""
import json
import math
import threading
import time
import uuid
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from go2_obstacle_interfaces.msg import LocalizationEvidence, MotionAuthority, MotionCommand, MotionTelemetry
from .robot_state import installation_matrix, pose_matrix, rotation_quaternion

class SimulationBackend(Node):
    def __init__(self, **kwargs):
        super().__init__('backend_simulation',**kwargs)
        for k,v in dict(simulation_mode=False, imu_in_base_xyz=[.3,.1,.2],
            imu_in_base_rpy=[0.,0.,.25], plant_frequency=30.0, output_watchdog=.25,
            nav_speed=.35, nav_yaw_speed=.7, nav_gain=2.0, nav_xy_tolerance=.015,
            nav_yaw_tolerance=.02, nav_timeout=20.0, publish_clock=False).items():
            self.declare_parameter(k,v)
        p=lambda k:self.get_parameter(k).value
        if not p('simulation_mode'):raise ValueError('explicit simulation_mode=true required')
        self.params={k:p(k) for k in ('plant_frequency','output_watchdog','nav_speed','nav_yaw_speed',
            'nav_gain','nav_xy_tolerance','nav_yaw_tolerance','nav_timeout','publish_clock')}
        if any(not math.isfinite(v) or v<=0 for k,v in self.params.items() if k!='publish_clock'):
            raise ValueError('invalid plant limits')
        self.extrinsic=installation_matrix(p('imu_in_base_xyz'),p('imu_in_base_rpy'))
        self.lock=threading.RLock();self.pose=[0.,0.,0.]
        self.command=(0.,0.,0.);self.command_at=0.;self.last_tick=time.monotonic()
        self.measured_velocity=(0.,0.,0.);self.command_history=[]
        self.nav_active=False;self.skill_active=False;self.obstacle_id=''
        self.skill_mode='';self.low_known=True;self.safe_stance=True;self.releasable=True
        self.authority=None;self.authority_at=0.;self.sequence=0
        self.source_id=uuid.uuid4().hex;self.evidence_sequence=0;self.evidence_epoch=1
        self.evidence_enabled=True;self.evidence_rejected=False;self.telemetry_enabled=True
        self.odom_enabled=True;self.freeze_clock=False;self.nav_requests=[];self.events=[]
        self.clock_ns=10_000_000_000;self.last_sensor_stamp=0
        self.cb=ReentrantCallbackGroup()
        self.odom_pub=self.create_publisher(Odometry,'/Odometry',20)
        self.telemetry_pub=self.create_publisher(MotionTelemetry,'/motion/telemetry',10)
        self.evidence_pub=self.create_publisher(LocalizationEvidence,'/gicp/evidence',10)
        self.pose_pub=self.create_publisher(PoseStamped,'/gicp/pose',10)
        self.valid_pub=self.create_publisher(Bool,'/gicp/valid',10)
        self.nav_cmd_pub=self.create_publisher(MotionCommand,'/motion/nav_command',1)
        self.status_pub=self.create_publisher(String,'/backend_sim/plant_status',10)
        self.create_subscription(Twist,'/backend_sim/cmd_vel',self._output,1,callback_group=self.cb)
        self.create_subscription(MotionAuthority,'/motion/authority',self._authority,10,callback_group=self.cb)
        self.create_subscription(String,'/backend_sim/skill_state',self._skill_state,10,callback_group=self.cb)
        self.tf=TransformBroadcaster(self);self.static_tf=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.
        self.static_tf.sendTransform(t)
        self.nav_server=ActionServer(self,NavigateToPose,'/navigate_to_pose',self._nav_execute,
            goal_callback=self._nav_goal,cancel_callback=lambda h:CancelResponse.ACCEPT,callback_group=self.cb)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=self.create_timer(1/p('plant_frequency'),self._tick,clock=self.steady,callback_group=self.cb)
        if p('publish_clock'):
            self.clock_pub=self.create_publisher(ClockMsg,'/clock',10)
            self.clock_timer=self.create_timer(1/p('plant_frequency'),self._clock_tick,clock=self.steady,callback_group=self.cb)
    def _clock_tick(self):
        if not self.freeze_clock:self.clock_ns+=int(1e9/self.params['plant_frequency'])
        m=ClockMsg();m.clock.sec=self.clock_ns//10**9;m.clock.nanosec=self.clock_ns%10**9
        self.clock_pub.publish(m)
    def _authority(self,m):self.authority=m;self.authority_at=time.monotonic()
    def _output(self,m):
        values=(m.linear.x,m.linear.y,m.angular.z)
        if not all(math.isfinite(v) for v in values):return
        with self.lock:
            self.command=values;self.command_at=time.monotonic()
            self.command_history.append((self.command_at,*values))
    def _skill_state(self,m):
        try:data=json.loads(m.data)
        except ValueError:return
        a=self.authority
        if not a or data.get('generation')!=a.generation:return
        event=data.get('event',data.get('phase','')).lower()
        self.events.append(data)
        self.obstacle_id=data.get('obstacle_id','');self.skill_mode=data.get('mode','success')
        if event in ('running','started'):
            self.skill_active=True;self.releasable=False;self.low_known=True;self.safe_stance=True
        elif event in ('finishing','released','cancelled','canceled','completed','stopped'):
            self.skill_active=False;self.releasable=self.skill_mode not in ('release_failure','cancel_unconfirmed')
        elif event in ('fault','crash'):
            self.skill_active=False;self.low_known=False;self.releasable=False
    def _authorized(self,owner):
        a=self.authority
        return bool(a and a.owner==owner and a.healthy and a.lease_remaining>0
                    and time.monotonic()-self.authority_at<.5)
    def _nav_goal(self,request):
        if request.pose.header.frame_id!='map' or self.nav_active or not self._authorized('NAVIGATION'):
            return GoalResponse.REJECT
        p,q=request.pose.pose.position,request.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    def _publish_nav(self,v):
        a=self.authority
        if not a:return
        m=MotionCommand();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='base_footprint'
        m.owner='NAVIGATION';m.mission_id=a.mission_id;m.generation=a.generation
        m.twist.linear.x,m.twist.linear.y,m.twist.angular.z=v;self.nav_cmd_pub.publish(m)
    def _nav_execute(self,handle):
        self.nav_active=True;self.nav_requests.append(handle.request)
        start=time.monotonic();p=handle.request.pose.pose.position;q=handle.request.pose.pose.orientation
        target=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        result=NavigateToPose.Result()
        try:
            while rclpy.ok(context=self.context):
                if handle.is_cancel_requested:
                    self._publish_nav((0.,0.,0.));handle.canceled();return result
                if not self._authorized('NAVIGATION') or time.monotonic()-start>self.params['nav_timeout']:
                    self._publish_nav((0.,0.,0.));handle.abort();return result
                with self.lock:x,y,yaw=self.pose
                dx,dy=target[0]-x,target[1]-y
                angle=math.atan2(math.sin(target[2]-yaw),math.cos(target[2]-yaw))
                if math.hypot(dx,dy)<=self.params['nav_xy_tolerance'] and abs(angle)<=self.params['nav_yaw_tolerance']:
                    self._publish_nav((0.,0.,0.));handle.succeed();return result
                vx=self.params['nav_gain']*(math.cos(yaw)*dx+math.sin(yaw)*dy)
                vy=self.params['nav_gain']*(-math.sin(yaw)*dx+math.cos(yaw)*dy)
                speed=math.hypot(vx,vy);f=min(1.,self.params['nav_speed']/max(speed,1e-12))
                wz=max(-self.params['nav_yaw_speed'],min(self.params['nav_yaw_speed'],self.params['nav_gain']*angle))
                self._publish_nav((vx*f,vy*f,wz))
                fb=NavigateToPose.Feedback();fb.distance_remaining=math.hypot(dx,dy);handle.publish_feedback(fb)
                time.sleep(.02)
        finally:self.nav_active=False
    def _tick(self):
        wall=time.monotonic();dt=min(.1,max(0.,wall-self.last_tick));self.last_tick=wall
        stamp=self.get_clock().now().to_msg()
        stamp_ns=stamp.sec*10**9+stamp.nanosec
        if stamp_ns<=self.last_sensor_stamp:return
        if self.params['publish_clock'] and self.last_sensor_stamp:
            dt=min(.1,(stamp_ns-self.last_sensor_stamp)/1e9)
        self.last_sensor_stamp=stamp_ns
        with self.lock:
            vx,vy,wz=self.command if wall-self.command_at<self.params['output_watchdog'] else (0.,0.,0.)
            x,y,yaw=self.pose
            self.pose=[x+(math.cos(yaw)*vx-math.sin(yaw)*vy)*dt,
                       y+(math.sin(yaw)*vx+math.cos(yaw)*vy)*dt,yaw+wz*dt]
            self.measured_velocity=(vx,vy,wz)
            x,y,yaw=self.pose
        base=pose_matrix([x,y,0.],[0.,0.,math.sin(yaw/2),math.cos(yaw/2)])
        imu=base@self.extrinsic
        q=rotation_quaternion(imu[:3,:3])
        o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='livox_frame'
        o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=map(float,imu[:3,3])
        o.pose.pose.orientation.x,o.pose.pose.orientation.y,o.pose.pose.orientation.z,o.pose.pose.orientation.w=map(float,q)
        if self.odom_enabled:self.odom_pub.publish(o)
        t=TransformStamped();t.header=o.header;t.child_frame_id='livox_frame'
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,imu[:3,3])
        t.transform.rotation=o.pose.pose.orientation;self.tf.sendTransform(t)
        self.sequence+=1
        if self.telemetry_enabled:
            m=MotionTelemetry();m.header.stamp=stamp;m.header.frame_id='base_footprint'
            m.source_id=self.source_id;m.sequence=self.sequence;m.known=self.low_known
            m.nav_active=self.nav_active;m.skill_active=self.skill_active;m.obstacle_id=self.obstacle_id
            m.stopped=math.hypot(vx,vy)<=.03 and abs(wz)<=.05
            m.safe_stance=self.safe_stance;m.control_releasable=self.releasable
            m.error_code='' if self.low_known else 'SIMULATED_LOW_LEVEL_FAULT'
            self.telemetry_pub.publish(m)
        if self.evidence_enabled:
            self.evidence_sequence+=1
            e=LocalizationEvidence();e.header.stamp=stamp;e.header.frame_id='map';e.body_frame='livox_frame'
            e.source_id='sim_gicp_'+self.source_id;e.epoch=self.evidence_epoch;e.sequence=self.evidence_sequence
            e.accepted=not self.evidence_rejected;e.covariance_known=False;e.pose.pose=o.pose.pose
            e.reason='SIMULATED_ACCEPTED' if e.accepted else 'SIMULATED_REJECTED';self.evidence_pub.publish(e)
            ps=PoseStamped();ps.header=e.header;ps.pose=e.pose.pose;self.pose_pub.publish(ps)
            self.valid_pub.publish(Bool(data=e.accepted))
        self.status_pub.publish(String(data=json.dumps(dict(simulation_only=True,pose=self.pose,
            measured_velocity=self.measured_velocity,nav_active=self.nav_active,skill_active=self.skill_active,
            command_watchdog=wall-self.command_at>=self.params['output_watchdog']))))
    def destroy_node(self):
        self.nav_server.destroy();return super().destroy_node()

def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);node=SimulationBackend()
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
