"""Action tests with independent authority and measured motion-state server."""
import json
import time
import pytest
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from go2_obstacle_interfaces.action import ExecuteObstacle
from go2_obstacle_interfaces.msg import MotionAuthority, MotionCommand
from go2_obstacle_interfaces.srv import GetMotionState
from go2_mission_backends.obstacle_adapter import ObstacleAdapter


class IndependentMotionState(Node):
    def __init__(self):
        super().__init__("independent_test_motion_state")
        self.authority_pub = self.create_publisher(MotionAuthority, "/motion/authority", 10)
        self.create_subscription(MotionCommand, "/motion/skill_command", self.command, 10)
        self.create_subscription(String, "/backend_sim/skill_state", self.event, 10)
        self.motion_service = self.create_service(GetMotionState, "/motion/get_state", self.state)
        self.emergency_service = self.create_service(Trigger, "/motion/emergency_hold", self.emergency)
        self.create_timer(.02, self.tick)
        self.owner, self.generation, self.active, self.known = "SKILL", 7, False, True
        self.obstacle_id, self.mode = "previous-obstacle", "success"
        self.commands, self.events, self.queries = [], [], []
        self.last_velocity, self.stop_since = (0., 0., 0.), time.monotonic()
        self.emergency_count = 0
        self.publish_authority = True
        self.force_active = False
        self.force_unsafe = False
        self.fail_after_release = False
        self.wrong_state_id = False

    def tick(self):
        if not self.publish_authority:
            return
        m = MotionAuthority(); m.header.stamp=self.get_clock().now().to_msg()
        m.owner=self.owner; m.mission_id="test-mission"; m.generation=self.generation
        m.healthy=self.known; m.lease_remaining=1.
        self.authority_pub.publish(m)

    def command(self, m):
        self.commands.append(m)
        v = (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)
        if any(v):
            self.stop_since = None
        elif any(self.last_velocity) or self.stop_since is None:
            self.stop_since = time.monotonic()
        self.last_velocity = v

    def event(self, m):
        d=json.loads(m.data); self.events.append(d)
        self.obstacle_id, self.mode = d["obstacle_id"], d["mode"]
        if d["event"] == "running":
            self.active=True
        elif d["event"] in ("finishing", "released"):
            self.active=False
            if d["event"] == "released" and self.fail_after_release:
                self.known=False
        elif d["event"] == "fault":
            self.active=False; self.known=False

    def state(self, req, res):
        self.queries.append((req.obstacle_id, time.monotonic()))
        match = not req.obstacle_id or req.obstacle_id == self.obstacle_id
        stopped = self.stop_since is not None and time.monotonic()-self.stop_since >= .04
        res.known=self.known and match
        res.obstacle_id="wrong-obstacle" if self.wrong_state_id else self.obstacle_id
        res.skill_active=self.active or self.force_active
        res.safe_stance=res.known and not self.force_unsafe
        res.control_releasable=bool(res.known and stopped and not res.skill_active and
                                    not self.force_unsafe and self.mode != "release_failure")
        res.error_code="" if res.known else "STATE_UNKNOWN"
        return res

    def emergency(self, _req, res):
        self.emergency_count+=1
        self.owner="SAFE_HOLD"; self.generation+=1
        res.success=True; res.message="test fence only"
        return res


@pytest.fixture
def rig(tmp_path):
    rclpy.init(args=[])
    executor=SingleThreadedExecutor()
    nodes=[]
    def add(n):
        nodes.append(n); executor.add_node(n); return n
    def spin(predicate, timeout=2.):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.01)
            if predicate(): return True
        return predicate()
    plant=add(IndependentMotionState())
    user=add(Node("skill_action_test_client"))
    def server(mode="success", duration=.12, budget=.8, simulation=True, factories=None):
        p=tmp_path/("skills_"+mode+".yaml")
        p.write_text(yaml.safe_dump({"skills": {"test": dict(executor="test", mode=mode, duration=duration,
                                                             velocity=[.1, .02, .03], execution_timeout=budget)}}))
        return add(ObstacleAdapter(executor_factories=factories, parameter_overrides=[Parameter("skills_file", value=str(p)),
                   Parameter("simulation_mode", value=simulation), Parameter("state_timeout", value=.3),
                   Parameter("release_timeout", value=.25), Parameter("authority_timeout", value=.2),
                   Parameter("state_poll_period", value=.02)]))
    action=ActionClient(user, ExecuteObstacle, "/execute_obstacle")
    def goal(skill="test", obstacle="obstacle-a", feedback=None):
        assert spin(lambda: action.server_is_ready())
        fut=action.send_goal_async(ExecuteObstacle.Goal(skill_id=skill, obstacle_id=obstacle), feedback_callback=feedback)
        assert spin(fut.done, timeout=5.)
        return fut.result()
    yield plant, server, action, goal, spin, user, executor, nodes
    action.destroy()
    for n in reversed(nodes):
        executor.remove_node(n); n.destroy_node()
    executor.shutdown(); rclpy.shutdown()


def test_success_requires_independent_safe_release_and_preserves_owner(rig):
    plant, server, _, goal, spin, _, _, _ = rig
    adapter=server()
    assert spin(lambda: adapter._authorized())
    feedback=[]
    g=goal(feedback=lambda f: feedback.append(f.feedback))
    assert g.accepted
    result=g.get_result_async(); assert spin(result.done)
    r=result.result().result
    assert r.completed and r.control_released and r.safe_stance
    assert feedback and all(0. <= f.progress <= 1. for f in feedback)
    assert plant.owner == "SKILL"  # existing Mission owns subsequent SAFE_HOLD.
    assert [e["event"] for e in plant.events] == ["running", "finishing", "released"]
    assert plant.commands and any(m.twist.linear.x > 0 for m in plant.commands)
    assert all(m.owner == "SKILL" and m.generation == 7 and m.mission_id == "test-mission" for m in plant.commands)
    assert not any(plant.last_velocity)
    assert plant.queries[0][0] == ""  # allow last obstacle ID before starting.
    assert all(q[0] in ("", "obstacle-a") for q in plant.queries)
    assert all(q[0] == "obstacle-a" for q in plant.queries[-2:])


@pytest.mark.parametrize("mode,error", [("fail", "SKILL_EXECUTION_FAILED"), ("timeout", "SKILL_TIMEOUT"),
                                        ("crash", "EXECUTOR_EXCEPTION"), ("release_failure", "CONTROL_RELEASE_UNCONFIRMED")])
def test_faults_abort_and_do_not_claim_release(rig, mode, error):
    plant, server, _, goal, spin, _, _, _ = rig
    adapter=server(mode); assert spin(lambda: adapter._authorized())
    g=goal(); assert g.accepted
    result=g.get_result_async(); assert spin(result.done, timeout=2.)
    r=result.result().result
    assert not r.completed and not r.control_released and not r.safe_stance
    assert r.error_code == error
    assert spin(lambda: plant.emergency_count > 0)
    assert plant.events[-1]["event"] == "fault"
    assert plant.events[-1]["error_code"] == error
    assert not any(plant.last_velocity)


def test_confirmed_cancel_returns_canceled_only_after_safe_release(rig):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(duration=1., budget=2.); assert spin(lambda: adapter._authorized())
    g=goal(); assert g.accepted
    assert spin(lambda: plant.active and any(plant.last_velocity))
    cancel=g.cancel_goal_async(); assert spin(cancel.done)
    assert cancel.result().goals_canceling
    result=g.get_result_async(); assert spin(result.done)
    r=result.result().result
    assert not r.completed and r.control_released and r.safe_stance
    assert r.error_code == "CANCELED"
    assert plant.owner == "SKILL" and plant.emergency_count == 0


def test_unconfirmed_cancel_aborts_fences_and_never_claims_release(rig):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server("cancel_unconfirmed", duration=1., budget=2.); assert spin(lambda: adapter._authorized())
    g=goal(); assert spin(lambda: plant.active)
    cancel=g.cancel_goal_async(); assert spin(cancel.done)
    result=g.get_result_async(); assert spin(result.done)
    r=result.result().result
    assert not r.completed and not r.control_released and r.error_code == "CANCEL_UNCONFIRMED"
    assert spin(lambda: plant.emergency_count > 0)


def test_wrong_exit_action_success_contains_no_exit_arrival_claim(rig):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server("wrong_exit"); assert spin(lambda: adapter._authorized())
    g=goal(); result=g.get_result_async(); assert spin(result.done)
    assert result.result().result.completed
    assert all(not any((m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)) for m in plant.commands)


def test_conflicting_goal_is_rejected_while_executor_active(rig):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(duration=.5, budget=2.); assert spin(lambda: adapter._authorized())
    first=goal(); assert first.accepted
    assert spin(lambda: plant.active)
    second=goal(obstacle="obstacle-b"); assert not second.accepted
    result=first.get_result_async(); assert spin(result.done)
    assert result.result().result.completed
    assert all(e["obstacle_id"] == "obstacle-a" for e in plant.events)


@pytest.mark.parametrize("mutate", ["lease_stream_stopped", "generation", "owner"])
def test_authority_loss_or_new_session_aborts(rig, mutate):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(duration=1., budget=2.); assert spin(lambda: adapter._authorized())
    g=goal(); assert spin(lambda: plant.active)
    if mutate == "lease_stream_stopped": plant.publish_authority=False
    elif mutate == "generation": plant.generation+=1
    else: plant.owner="NAVIGATION"
    result=g.get_result_async(); assert spin(result.done)
    assert not result.result().result.completed
    assert result.result().result.error_code == "SKILL_AUTHORITY_LOST"


@pytest.mark.parametrize("invalid", ["unknown", "unsafe", "active"])
def test_initial_physical_state_rejects_execution(rig, invalid):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(duration=.2, budget=1.); assert spin(lambda: adapter._authorized())
    if invalid == "unknown": plant.known=False
    elif invalid == "unsafe": plant.force_unsafe=True
    else: plant.force_active=True
    g=goal()
    if not g.accepted:
        assert not plant.events
        return
    result=g.get_result_async(); assert spin(result.done)
    assert not result.result().result.completed
    assert not any(e["event"] == "running" for e in plant.events)
    assert not any(plant.last_velocity)


def test_hardware_mode_forbids_test_executor_and_reports_not_configured(rig):
    plant, server, _, goal, spin, user, _, _=rig
    adapter=server(simulation=False); assert spin(lambda: adapter._authorized())
    ready=user.create_client(Trigger, "/skills/ready"); assert spin(ready.service_is_ready)
    future=ready.call_async(Trigger.Request()); assert spin(future.done)
    assert not future.result().success
    assert json.loads(future.result().message)["reason"] == "NOT_CONFIGURED"
    assert not goal().accepted
    assert not plant.commands


def test_missing_skill_is_rejected_before_motion(rig):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(); assert spin(lambda: adapter._authorized())
    assert not goal(skill="unregistered").accepted
    assert not plant.commands


def test_motion_state_service_exit_blocks_completion(rig):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(duration=1., budget=2.); assert spin(lambda: adapter._authorized())
    g=goal(); assert spin(lambda: plant.active)
    assert plant.destroy_service(plant.motion_service)  # independent GetMotionState disappears.
    result=g.get_result_async(); assert spin(result.done)
    assert not result.result().result.completed
    assert result.result().result.error_code == "RUNNING_STATE_UNCONFIRMED"


def test_released_executor_without_new_physical_confirmation_is_rejected(rig):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(); assert spin(lambda: adapter._authorized())
    plant.fail_after_release=True
    g=goal(); result=g.get_result_async(); assert spin(result.done)
    assert not result.result().result.completed
    assert not result.result().result.control_released
    assert spin(lambda: any(e["event"] == "released" for e in plant.events))


def test_wrong_obstacle_state_does_not_confirm_active_skill(rig):
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(duration=1., budget=2.); assert spin(lambda: adapter._authorized())
    g=goal(); assert spin(lambda: plant.active)
    plant.wrong_state_id=True
    result=g.get_result_async(); assert spin(result.done)
    assert not result.result().result.completed
    assert result.result().result.error_code == "RUNNING_STATE_UNCONFIRMED"


def test_executor_close_failure_cannot_be_overridden_by_safe_service(rig):
    from go2_mission_backends.skill_executor import TestSkillExecutor
    class DoesNotRelease(TestSkillExecutor):
        def close(self, now): return False
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(factories={"test":DoesNotRelease}); assert spin(lambda: adapter._authorized())
    g=goal(); result=g.get_result_async(); assert spin(result.done)
    assert result.result().result.error_code == "EXECUTOR_RELEASE_FAILED"
    assert not result.result().result.control_released


def test_hard_server_exit_never_produces_success(rig, tmp_path):
    import subprocess
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from go2_obstacle_interfaces.msg import MotionTelemetry
    from go2_obstacle_interfaces.srv import SetControlMode
    from go2_mission_backends.motion_arbiter import MotionArbiter
    plant, _, _, goal, spin, user, executor, nodes=rig
    # Replace the test authority/state Service with the actual command gate.
    plant.publish_authority=False
    plant.destroy_service(plant.motion_service)
    plant.destroy_service(plant.emergency_service)
    telemetry_pub=plant.create_publisher(MotionTelemetry,"/motion/telemetry",10)
    odom_pub=plant.create_publisher(Odometry,"/odom",20)
    final=[]
    final_sub=plant.create_subscription(Twist,"/backend_sim/kill_cmd",final.append,10)
    sample_seq=[0]
    def low_level_sample():
        sample_seq[0]+=1
        now=plant.get_clock().now().to_msg()
        last=final[-1] if final else Twist()
        telemetry=MotionTelemetry();telemetry.header.stamp=now;telemetry.source_id="kill-test-plant"
        telemetry.sequence=sample_seq[0];telemetry.known=True;telemetry.skill_active=plant.active
        telemetry.obstacle_id=plant.obstacle_id;telemetry.safe_stance=True
        telemetry.stopped=not any((last.linear.x,last.linear.y,last.angular.z))
        telemetry.control_releasable=not plant.active
        odom=Odometry();odom.header.stamp=now;odom.header.frame_id="nav_odom"
        odom.child_frame_id="base_footprint";odom.pose.pose.orientation.w=1.;odom.twist.twist=last
        telemetry_pub.publish(telemetry);odom_pub.publish(odom)
    sample_timer=plant.create_timer(.02,low_level_sample)
    arbiter=MotionArbiter(parameter_overrides=[Parameter("simulation_mode",value=True),
        Parameter("enable_output",value=True),Parameter("output_topic",value="/backend_sim/kill_cmd"),
        Parameter("stable_duration",value=.05),Parameter("command_timeout",value=.15)])
    nodes.append(arbiter);executor.add_node(arbiter)
    switch=user.create_client(SetControlMode,"/motion/set_control_mode")
    assert spin(lambda: switch.service_is_ready() and arbiter.safe_switch())
    req=SetControlMode.Request(owner="SKILL",expected_owner="SAFE_HOLD",mission_id="test-mission",
                              request_id="kill-test-acquire",lease_duration=5.)
    switched=switch.call_async(req);assert spin(switched.done)
    assert switched.result().accepted
    path=tmp_path/"external_skills.yaml"
    path.write_text(yaml.safe_dump({"skills":{"test":dict(executor="test", mode="success", duration=10.,
                                                         velocity=[.1,0.,0.], execution_timeout=20.)}}))
    script="""import sys, rclpy
from rclpy.parameter import Parameter
from go2_mission_backends.obstacle_adapter import ObstacleAdapter
rclpy.init(args=[])
n=ObstacleAdapter(parameter_overrides=[Parameter('skills_file',value=sys.argv[1]),Parameter('simulation_mode',value=True)])
rclpy.spin(n)
"""
    log=open(tmp_path/"external_skill.log","w")
    proc=subprocess.Popen(["/usr/bin/python3","-u","-c",script,str(path)],stdout=log,stderr=subprocess.STDOUT)
    try:
        assert spin(lambda: plant.authority_pub.get_subscription_count() == 1, timeout=5.)
        start=time.monotonic(); assert spin(lambda: time.monotonic()-start>.25)
        g=goal(); assert g.accepted
        result=g.get_result_async()
        assert spin(lambda: plant.active and final and abs(final[-1].linear.x) > .05, timeout=5.)
        proc.kill(); proc.wait(timeout=3.)
        assert spin(lambda: final and not any((final[-1].linear.x,final[-1].linear.y,final[-1].angular.z)),timeout=1.)
        start=time.monotonic(); assert spin(lambda: time.monotonic()-start>.35)
        assert not result.done() or not result.result().result.completed
        assert not any((final[-1].linear.x,final[-1].linear.y,final[-1].angular.z))
        assert not any(e["event"] == "released" for e in plant.events)
    finally:
        if proc.poll() is None:
            proc.kill();proc.wait(timeout=3.)
        log.close()


def test_authority_duplicate_does_not_refresh_wall_and_new_generation_same_stamp_is_valid(rig):
    plant, server, _, _, spin, _, _, _=rig
    adapter=server(); assert spin(lambda: adapter._authorized())
    plant.publish_authority=False
    m=MotionAuthority();m.header.stamp=adapter.get_clock().now().to_msg()
    m.owner="SKILL";m.mission_id="test-mission";m.generation=7;m.healthy=True;m.lease_remaining=1.
    adapter._authority(m)
    received_at=adapter.authority_wall
    adapter._authority(m)
    assert adapter.authority is m and adapter.authority_wall == received_at
    adapter.authority_wall=time.monotonic()-.3
    adapter._authority(m)
    assert adapter.authority is m and not adapter._authorized()  # unchanged ROS tick cannot beat wall timeout.
    m2=MotionAuthority();m2.header.stamp=m.header.stamp
    m2.owner="SKILL";m2.mission_id="new-mission";m2.generation=8;m2.healthy=True;m2.lease_remaining=1.
    adapter._authority(m2)
    assert adapter._authorized()
    # Delayed old generation cannot replace newer identity, even with new stamp.
    m.header.stamp=adapter.get_clock().now().to_msg()
    adapter._authority(m)
    assert not adapter._authorized()


def test_malformed_authority_time_and_unhealthy_duplicate_are_invalid(rig):
    plant, server, _, _, spin, _, _, _=rig
    adapter=server();assert spin(lambda:adapter._authorized())
    plant.publish_authority=False
    m=MotionAuthority();m.header.stamp=adapter.get_clock().now().to_msg()
    m.owner="SKILL";m.mission_id="test-mission";m.generation=7;m.healthy=True;m.lease_remaining=1.
    adapter._authority(m)
    unhealthy=MotionAuthority();unhealthy.header.stamp=m.header.stamp
    unhealthy.owner=m.owner;unhealthy.mission_id=m.mission_id;unhealthy.generation=m.generation
    unhealthy.healthy=False;unhealthy.lease_remaining=1.
    adapter._authority(unhealthy)
    assert not adapter._authorized()
    m.header.stamp.nanosec=1000000000
    adapter._authority(m);assert not adapter._authorized()
    m.header.stamp.sec=-1;m.header.stamp.nanosec=0
    adapter._authority(m);assert not adapter._authorized()


def test_late_completed_rpc_is_not_new_measured_state(rig):
    from rclpy.task import Future
    plant, server, _, _, spin, _, _, _=rig
    adapter=server();assert spin(lambda:adapter._authorized())
    f=Future();f.set_result(GetMotionState.Response(known=True, safe_stance=True, control_releasable=True))
    now=time.monotonic()
    op=dict(query=f, query_started=now-.5, state=None, state_wall=0., query_next=now+10., phase="PRECHECK")
    adapter._query(op,now)
    assert op["state"] is None and op["state_wall"] == 0.


def test_cancel_during_precheck_never_starts_executor(rig):
    from go2_mission_backends.skill_executor import TestSkillExecutor
    class SpyExecutor(TestSkillExecutor):
        starts=0
        def start(self, now):
            type(self).starts+=1
            super().start(now)
    plant, server, _, goal, spin, _, _, _=rig
    adapter=server(duration=1.,budget=2.,factories={"test":SpyExecutor})
    assert spin(lambda:adapter._authorized())
    plant.force_unsafe=True
    g=goal();assert g.accepted
    assert adapter.op["phase"] == "PRECHECK"
    cancel=g.cancel_goal_async();assert spin(cancel.done)
    assert cancel.result().goals_canceling
    plant.force_unsafe=False
    result=g.get_result_async();assert spin(result.done)
    response=result.result().result
    assert not response.completed and response.control_released and response.safe_stance
    assert response.error_code == "CANCELED_BEFORE_START"
    assert SpyExecutor.starts == 0
    assert not plant.events
    assert plant.commands and all(not any((m.twist.linear.x,m.twist.linear.y,m.twist.angular.z)) for m in plant.commands)


def test_cancel_before_start_without_safe_confirmation_does_not_claim_release(rig):
    from go2_mission_backends.skill_executor import TestSkillExecutor
    class SpyExecutor(TestSkillExecutor):
        starts=0
        def start(self,now):
            type(self).starts+=1
            super().start(now)
    plant,server,_,goal,spin,_,_,_=rig
    adapter=server(duration=1.,budget=2.,factories={"test":SpyExecutor})
    assert spin(lambda:adapter._authorized())
    plant.force_unsafe=True
    g=goal();cancel=g.cancel_goal_async();assert spin(cancel.done)
    result=g.get_result_async();assert spin(result.done)
    response=result.result().result
    assert not response.completed and not response.control_released
    assert response.error_code == "CANCEL_SAFE_STATE_UNCONFIRMED"
    assert SpyExecutor.starts == 0
