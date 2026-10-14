"""Read-only live sensor/localization diagnostics; this release cannot arm motors."""
import json
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from go2_obstacle_interfaces.msg import MotionTelemetry
from .doctor import static_report
from .profile import DEFAULT_CONFIG


class OnboardDoctor(Node):
    def __init__(self,**kwargs):
        super().__init__('onboard_doctor',**kwargs)
        for name,value in dict(config_dir=DEFAULT_CONFIG,mode='localization',max_age=.5).items():self.declare_parameter(name,value)
        if self.get_parameter('use_sim_time').value:raise ValueError('onboard doctor requires use_sim_time=false')
        self.base=static_report(self.get_parameter('config_dir').value,self.get_parameter('mode').value)
        self.age=float(self.get_parameter('max_age').value)
        if not math.isfinite(self.age) or self.age<=0:raise ValueError('max_age must be positive')
        self.odom=self.telemetry=None;self.odom_at=self.telemetry_at=0.;self.telemetry_seq=0
        self.health=self.preflight=None;self.health_at=self.preflight_at=0.
        self.health_future=self.preflight_future=None
        self.health_requested=self.preflight_requested=0.
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.create_subscription(Odometry,'/odom',self._odom,10)
        self.create_subscription(MotionTelemetry,'/motion/telemetry',self._telemetry,10)
        self.health_client=self.create_client(Trigger,'/mission/localization_ok')
        self.preflight_client=self.create_client(Trigger,'/mission/preflight')
        self.publisher=self.create_publisher(String,'/onboard/doctor_status',10)
        self.create_service(Trigger,'/onboard/doctor',self._response)
        self.timer=self.create_timer(.2,self._tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _odom(self,msg):self.odom=msg;self.odom_at=time.monotonic()
    def _telemetry(self,msg):
        if msg.header.frame_id!='base_footprint' or not msg.source_id or msg.sequence<=self.telemetry_seq:
            self.telemetry=None;return
        self.telemetry_seq=msg.sequence;self.telemetry=msg;self.telemetry_at=time.monotonic()
    def stamp_fresh(self,stamp):
        ns=stamp.sec*10**9+stamp.nanosec
        return stamp.sec>=0 and 0<=stamp.nanosec<10**9 and ns>0 and -.05<=(self.get_clock().now().nanoseconds-ns)/1e9<=self.age
    def report(self):
        result=json.loads(json.dumps(self.base));now=time.monotonic()
        odom=self.odom
        odom_ok=bool(odom and now-self.odom_at<=self.age and self.stamp_fresh(odom.header.stamp) and
            odom.header.frame_id=='nav_odom' and odom.child_frame_id=='base_footprint')
        if odom_ok:
            t=odom.twist.twist;p=odom.pose.pose.position;q=odom.pose.pose.orientation
            vals=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z]
            odom_ok=all(math.isfinite(v) for v in vals) and abs(sum(v*v for v in vals[3:7])-1.)<.1
        tf_ok=False
        try:
            tf=self.buffer.lookup_transform('map','base_footprint',Time())
            t=tf.transform;values=[t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
            tf_ok=self.stamp_fresh(tf.header.stamp) and all(math.isfinite(v) for v in values) and abs(sum(v*v for v in values[3:])-1.)<.1
        except TransformException:pass
        health_ok=bool(self.health and self.health.success and now-self.health_at<1.)
        telemetry=self.telemetry
        low=bool(telemetry and now-self.telemetry_at<=self.age and self.stamp_fresh(telemetry.header.stamp) and
            telemetry.known and telemetry.safe_stance and telemetry.control_releasable and not telemetry.error_code)
        result.update(localization_ready=bool(result.get('configuration_valid') and odom_ok and tf_ok and health_ok),
            live_checks=dict(odometry=odom_ok,map_to_footprint_tf=tf_ok,accepted_localization=health_ok,
                             measured_low_level_state=low,preflight=bool(self.preflight and self.preflight.success and now-self.preflight_at<1.)),
            direct_motor_publishers=[p.node_name for p in self.get_publishers_info_by_topic('/cmd_vel')],
            status='NOT_READY',mission_allowed=False,motion_enabled=False)
        if result['direct_motor_publishers']:result['reasons'].append('UNEXPECTED_DIRECT_MOTOR_PUBLISHER')
        result['reasons']=sorted(set(result['reasons']))
        result['safety_boundary']='No real low-level/skill integration is provided by this release. Receiving a topic cannot arm motors.'
        return result
    def _tick(self):
        for name in ['health','preflight']:
            client=getattr(self,name+'_client');pending=getattr(self,name+'_future')
            if pending is not None and not pending.done() and time.monotonic()-getattr(self,name+'_requested')>1.:
                pending.cancel();setattr(self,name+'_future',None);setattr(self,name,None)
                pending=None
            if pending is not None and pending.done():
                try:setattr(self,name,pending.result());setattr(self,name+'_at',time.monotonic())
                except Exception:setattr(self,name,None)
                setattr(self,name+'_future',None)
            if getattr(self,name+'_future') is None and client.service_is_ready():
                setattr(self,name+'_future',client.call_async(Trigger.Request()));setattr(self,name+'_requested',time.monotonic())
        self.publisher.publish(String(data=json.dumps(self.report(),sort_keys=True)))
    def _response(self,_request,response):
        result=self.report();response.success=False;response.message=json.dumps(result,sort_keys=True);return response


def main(args=None):
    rclpy.init(args=args);node=None
    try:node=OnboardDoctor();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
