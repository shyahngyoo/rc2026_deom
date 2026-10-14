"""Nonblocking startup-only evidence check for the existing Mission.

Each RPC is independently timed on a monotonic clock. This node never grants
motion or starts Mission. Runtime supervision remains with Mission and the
motion arbiter; startup checks may legitimately become false during motion.
"""
import json
import math
import time
import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from go2_obstacle_interfaces.msg import MotionAuthority
from go2_obstacle_interfaces.srv import GetMotionState
from robocon_mission.async_service import AsyncServiceRunner
from robocon_mission.route_manager import RouteManager


DEFAULTS = dict(service_name='/mission/preflight',route_file='',
    localization_ok_service='/mission/localization_ok',
    robot_state_topic='/robot_state/status',odom_topic='/odom',
    authority_topic='/motion/authority',motion_state_service='/motion/get_state',
    nav_action_name='/navigate_to_pose',skills_ready_service='/skills/ready',
    output_topic='/backend_sim/cmd_vel',expected_output_node='motion_arbiter',
    map_frame='map',base_frame='base_footprint',odom_frame='nav_odom',
    require_distinct_endpoints=False,minimum_exit_distance=.1,
    topic_max_age=.5,adapter_status_max_age=.5,rpc_timeout=.3,rpc_max_age=.5,
    poll_period=.1,tf_max_age=.5,future_tolerance=.05,required_nav_nodes=[])


def valid_stamp(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        return 0
    return stamp.sec*1000000000+stamp.nanosec


def recent_stamp(stamp_ns,now_ns,max_age,future_tolerance=.05):
    return bool(stamp_ns>0 and now_ns>0 and -future_tolerance <= (now_ns-stamp_ns)*1e-9 <= max_age)


def route_requirements(route,registered_skills,require_distinct=False,minimum_distance=.1):
    reasons=[]
    for task in route.tasks:
        if task.type!='obstacle':continue
        if task.skill_id not in registered_skills:
            reasons.append('SKILL_NOT_REGISTERED:'+task.skill_id)
        if require_distinct and math.hypot(task.end_pose[0]-task.approach_pose[0],
                task.end_pose[1]-task.approach_pose[1]) < minimum_distance:
            reasons.append('ENTRANCE_EXIT_NOT_DISTINCT:'+task.id)
    return sorted(set(reasons))


class MissionPreflight(Node):
    def __init__(self,**kwargs):
        super().__init__('mission_preflight',**kwargs)
        for k,v in DEFAULTS.items():
            if k=='required_nav_nodes':
                self.declare_parameter(k,v,ParameterDescriptor(dynamic_typing=True))
            else:self.declare_parameter(k,v)
        self.params={k:self.get_parameter(k).value for k in DEFAULTS}
        p=self.params
        # Humble's YAML/CLI parser represents an explicit [] override as
        # NOT_SET (value None), whereas direct Python defaults retain [].
        # Both mean no optional lifecycle assertions; scalars remain invalid.
        if p['required_nav_nodes'] is None:
            p['required_nav_nodes']=[]
        if not isinstance(p['required_nav_nodes'],list) or not all(isinstance(n,str) for n in p['required_nav_nodes']):
            raise ValueError('required_nav_nodes must be a string array')
        for name in ('minimum_exit_distance','topic_max_age','adapter_status_max_age',
                     'rpc_timeout','rpc_max_age','poll_period','tf_max_age'):
            if not math.isfinite(p[name]) or p[name]<=0:raise ValueError('invalid '+name)
        if not math.isfinite(p['future_tolerance']) or p['future_tolerance']<0:
            raise ValueError('invalid future_tolerance')
        if not p['route_file']:raise ValueError('route_file is required')
        self.route=None;self.route_error=''
        try:
            self.route=RouteManager.load(p['route_file'])
            if self.route.frame_id!=p['map_frame']:
                raise ValueError('route/map_frame mismatch')
        except Exception as error:
            self.route=None;self.route_error='ROUTE_INVALID:'+str(error)
        self.adapter=self.odom=self.authority=None
        self.adapter_at=self.odom_at=self.authority_at=0.
        self.last_ros=None;self.token=0
        self.rpc={};self.runners={};self.rpc_clients={};self.last_poll={}
        self._add_rpc('localization',Trigger,p['localization_ok_service'],Trigger.Request)
        self._add_rpc('skills',Trigger,p['skills_ready_service'],Trigger.Request)
        self._add_rpc('motion',GetMotionState,p['motion_state_service'],GetMotionState.Request)
        self.nav_nodes=[n for n in p['required_nav_nodes'] if n]
        for node in self.nav_nodes:
            self._add_rpc('nav_node:'+node,GetState,node.rstrip('/')+'/get_state',GetState.Request)
        self.nav=ActionClient(self,NavigateToPose,p['nav_action_name'])
        self.tf_buffer=Buffer(node=self);self.tf_listener=TransformListener(self.tf_buffer,self)
        self.create_subscription(String,p['robot_state_topic'],self._adapter,10)
        self.create_subscription(Odometry,p['odom_topic'],self._odom,20)
        self.create_subscription(MotionAuthority,p['authority_topic'],self._authority,10)
        self.status_pub=self.create_publisher(String,'/mission/preflight_status',10)
        self.create_service(Trigger,p['service_name'],self._check)
        self.timer=self.create_timer(p['poll_period'],self._tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._tick()

    def _add_rpc(self,key,srv,name,request_factory):
        client=self.create_client(srv,name)
        self.rpc_clients[key]=client;self.runners[key]=AsyncServiceRunner(client)
        self.rpc[key]=dict(valid=False,at=0.,result=None,reason='WAITING_FOR_RESPONSE')
        self.last_poll[key]=0.
        self.rpc[key]['request_factory']=request_factory

    def _adapter(self,m):
        try:
            data=json.loads(m.data)
            if not isinstance(data,dict):raise ValueError()
            self.adapter=data;self.adapter_at=time.monotonic()
        except (ValueError,TypeError):self.adapter=None;self.adapter_at=0.

    def _odom(self,m):self.odom=m;self.odom_at=time.monotonic()
    def _authority(self,m):self.authority=m;self.authority_at=time.monotonic()

    def _rpc_done(self,key,token,outcome,terminal,result):
        self.rpc[key].update(valid=outcome=='succeeded' and terminal,at=time.monotonic(),
                             result=result,reason=outcome)

    def _tick(self):
        now=self.get_clock().now().nanoseconds;wall=time.monotonic()
        if self.last_ros is not None and now < self.last_ros:
            self.adapter=self.odom=self.authority=None
            for key,runner in self.runners.items():
                runner.cancel();self.rpc[key].update(valid=False,at=0.,result=None,reason='CLOCK_ROLLBACK')
        self.last_ros=now
        for key,runner in self.runners.items():
            runner.tick()
            if runner.pending is None and wall-self.last_poll[key]>=self.params['poll_period']:
                self.token+=1;self.last_poll[key]=wall
                runner.start(self.token,self.rpc[key]['request_factory'](),self.params['rpc_timeout'],
                    lambda t,o,term,r,k=key:self._rpc_done(k,t,o,term,r),lambda r:r is not None)
                runner.tick()
        self.status_pub.publish(String(data=json.dumps(self.evaluate(),sort_keys=True)))

    def _rpc_fresh(self,key,wall):
        state=self.rpc[key]
        return state['valid'] and wall-state['at']<=self.params['rpc_max_age']

    def evaluate(self):
        p=self.params;wall=time.monotonic();now=self.get_clock().now().nanoseconds
        reasons=[];checks={}
        def record(name,ok,reason):
            checks[name]=bool(ok)
            if not ok:reasons.append(reason)
        def stamp_ok(stamp,limit=None):
            return recent_stamp(valid_stamp(stamp),now,limit or p['topic_max_age'],p['future_tolerance'])
        record('clock',now>0 and (self.last_ros is None or now>=self.last_ros),
               'CLOCK_ROLLBACK' if self.last_ros is not None and now<self.last_ros else 'CLOCK_UNAVAILABLE')
        record('route',self.route is not None,self.route_error or 'ROUTE_INVALID')
        a=self.adapter
        valid_adapter=bool(a and a.get('global_ready') is True and a.get('local_odom_valid') is True
            and wall-self.adapter_at<=p['adapter_status_max_age']
            and isinstance(a.get('input_stamp_ns'),int) and not isinstance(a.get('input_stamp_ns'),bool)
            and recent_stamp(a['input_stamp_ns'],now,p['adapter_status_max_age'],p['future_tolerance']))
        record('robot_state',valid_adapter,'ROBOT_STATE_UNAVAILABLE_OR_STALE')
        o=self.odom
        if o:
            t=o.twist.twist;q=o.pose.pose.orientation;pos=o.pose.pose.position
            numbers=(t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z,
                     pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w)
            norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            odom_ok=(o.header.frame_id==p['odom_frame'] and o.child_frame_id==p['base_frame']
                and wall-self.odom_at<=p['topic_max_age'] and stamp_ok(o.header.stamp)
                and all(math.isfinite(x) for x in numbers) and abs(norm-1.)<=.05)
        else:odom_ok=False
        record('odometry',odom_ok,'ODOMETRY_UNAVAILABLE_OR_INVALID')
        a=self.authority
        authority_ok=bool(a and a.healthy and a.owner=='SAFE_HOLD' and not a.mission_id
            and a.generation>0 and wall-self.authority_at<=p['topic_max_age'] and stamp_ok(a.header.stamp))
        record('authority',authority_ok,'AUTHORITY_NOT_INITIAL_SAFE_HOLD')
        tf_ok=False
        try:
            tf=self.tf_buffer.lookup_transform(p['map_frame'],p['base_frame'],Time())
            t=tf.transform;v=(t.translation.x,t.translation.y,t.translation.z,
                t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
            norm=math.sqrt(sum(x*x for x in v[3:]))
            tf_ok=all(math.isfinite(x) for x in v) and abs(norm-1.)<=.05 and stamp_ok(tf.header.stamp,p['tf_max_age'])
        except TransformException:pass
        record('tf',tf_ok,'NAVIGATION_TF_UNAVAILABLE_OR_STALE')
        health=self.rpc['localization']['result']
        record('localization',self._rpc_fresh('localization',wall) and health is not None and health.success,
               'LOCALIZATION_EVIDENCE_UNAVAILABLE')
        motion=self.rpc['motion']['result']
        record('motion_state',self._rpc_fresh('motion',wall) and motion is not None and motion.known
            and motion.safe_stance and motion.control_releasable and not motion.skill_active and not motion.error_code,
            'LOW_LEVEL_SAFE_STATE_UNCONFIRMED')
        record('nav_action',self.nav.server_is_ready(),'NAVIGATION_ACTION_UNAVAILABLE')
        for node in self.nav_nodes:
            key='nav_node:'+node;r=self.rpc[key]['result']
            record(key,self._rpc_fresh(key,wall) and r is not None and r.current_state.id==3,
                   'NAVIGATION_NODE_NOT_ACTIVE:'+node)
        skills=self.rpc['skills']['result'];registered=[];skills_ok=False
        if self._rpc_fresh('skills',wall) and skills is not None and skills.success:
            try:
                data=json.loads(skills.message);registered=data['skills']
                skills_ok=isinstance(registered,list) and all(isinstance(s,str) and s.strip() for s in registered)
            except (ValueError,TypeError,KeyError):pass
        record('skills_backend',skills_ok,'SKILLS_BACKEND_UNAVAILABLE_OR_INVALID')
        if skills_ok and self.route is not None:
            route_errors=route_requirements(self.route,registered,p['require_distinct_endpoints'],p['minimum_exit_distance'])
            record('route_skills',not route_errors,'ROUTE_SKILLS_INVALID')
            reasons.extend(route_errors)
        else:checks['route_skills']=False
        try:publishers=self.get_publishers_info_by_topic(p['output_topic'])
        except Exception:publishers=[]
        output_ok=len(publishers)==1 and publishers[0].node_name==p['expected_output_node']
        record('command_path',output_ok,'FINAL_COMMAND_PUBLISHER_NOT_UNIQUE_ARBITER')
        return dict(ready=not reasons,reasons=sorted(set(reasons)),checks=checks,
            checked_stamp_ns=now,output_publishers=[dict(name=i.node_name,namespace=i.node_namespace) for i in publishers],
            required_skills=sorted({t.skill_id for t in self.route.tasks if t.type=='obstacle'}) if self.route else [])

    def _check(self,_req,res):
        result=self.evaluate();res.success=result['ready']
        res.message=json.dumps(result,sort_keys=True);return res


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=None
    try:node=MissionPreflight();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
