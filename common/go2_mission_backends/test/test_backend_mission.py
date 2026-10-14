"""Mission end-to-end tests with an integrated, command-driven simulated plant.

No target is teleported, no physical stop is inferred from sent commands, and
Action SUCCESS never substitutes for exit verification. ROS discovery, TF,
services and Actions run on an isolated localhost domain. Every case records
observed final commands, state transitions, evidence and independent plant pose.
"""
import copy
import json
import math
import os
import subprocess
import threading
import time
from pathlib import Path

os.environ.setdefault('ROS_DOMAIN_ID', '128')
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import pytest
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from go2_obstacle_interfaces.msg import LocalizationEvidence, MotionAuthority
from go2_mission_backends.localization_adapter import LocalizationAdapter
from go2_mission_backends.motion_arbiter import MotionArbiter
from go2_mission_backends.obstacle_adapter import ObstacleAdapter
from go2_mission_backends.preflight import MissionPreflight
from go2_mission_backends.robot_state_adapter import RobotStateAdapter
from go2_mission_backends.simulation_backend import SimulationBackend
from robocon_mission.mission_fsm import State
from robocon_mission.mission_manager import MissionManager

ROOT = Path(__file__).resolve().parents[3]
CONFIG = Path(__file__).resolve().parents[1] / 'config'
REPORTS = Path(os.environ.get('RC2026_REPORT_DIR', str(ROOT/'docs/verification/backend_tests')))


def overrides(parameters):
    return [Parameter(name, value=value) for name, value in parameters.items()]


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def twist_values(t):
    return [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]


class MissionRig:
    """Own every ROS resource; cleanup does not leave server subprocesses."""
    def __init__(self, directory, name, mode='success', sim_time=False, native_skill=False):
        self.name = name
        self.sim_time = sim_time
        self.created = time.monotonic()
        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=10, context=self.context)
        self.nodes = []
        self.detached = []
        self.errors = []
        self.states = []
        self.statuses = []
        self.motion_status = []
        self.authorities = []
        self.sink = []
        self.odom = []
        self.evidence = []
        self.skill_events = []
        self.preflight_status = []
        self.injection = None
        self.injection_at = None
        self.started_at = None
        self.report_extra = {}
        self.native_skill_process = None
        self.native_skill_log = None
        self.native_skill_clock = None
        route = yaml.safe_load((CONFIG / 'backend_test_route.yaml').read_text())
        # Fault cases shorten only failure deadlines; all targets and tolerances
        # are copied unchanged. Stop and acceptance durations retain defaults.
        if mode == 'cancel_unconfirmed':
            route['route'][0]['skill_timeout'] = .45
        route_path = directory / (name + '_route.yaml')
        route_path.write_text(yaml.safe_dump(route, sort_keys=False))
        skills = {}
        for skill_id in ('step_x', 'step_y'):
            skills[skill_id] = dict(executor='test', mode=mode if skill_id == 'step_x' else 'success',
                duration=1., velocity=[.2, 0., .3 if mode == 'wrong_yaw' and skill_id == 'step_x' else 0.],
                execution_timeout=.8 if mode == 'timeout' and skill_id == 'step_x' else 5.)
        if mode == 'wrong_yaw':
            skills['step_x']['mode'] = 'success'
        skills_path = directory / (name + '_skills.yaml')
        skills_path.write_text(yaml.safe_dump(dict(skills=skills)))
        shared = dict(use_sim_time=sim_time)
        self.plant = self.add(SimulationBackend(context=self.context, parameter_overrides=overrides(dict(
            shared, simulation_mode=True, publish_clock=sim_time))))
        self.state_adapter = self.add(RobotStateAdapter(context=self.context, parameter_overrides=overrides(dict(
            shared, imu_in_base_xyz=[.3, .1, .2], imu_in_base_rpy=[0., 0., .25]))))
        self.localization_adapter = self.add(LocalizationAdapter(context=self.context,
            parameter_overrides=overrides(dict(shared, evidence_max_age=.8))))
        self.arbiter = self.add(MotionArbiter(context=self.context, parameter_overrides=overrides(dict(
            shared, enable_output=True, simulation_mode=True))))
        if native_skill:
            REPORTS.mkdir(parents=True, exist_ok=True)
            self.native_skill_log_path = REPORTS/(name+'_native_skill.log')
            self.native_skill_log = self.native_skill_log_path.open('w')
            self.native_skill_process = subprocess.Popen(['/usr/bin/python3', '-u', '-c',
                'from go2_mission_backends.obstacle_adapter import main; main()', '--ros-args',
                '-p', 'simulation_mode:=true', '-p', 'use_sim_time:='+str(sim_time).lower(),
                '-p', 'skills_file:='+str(skills_path), '-p', 'release_timeout:=0.8'],
                stdout=self.native_skill_log, stderr=subprocess.STDOUT)
            self.skill = None
        else:
            self.skill = self.add(ObstacleAdapter(context=self.context, parameter_overrides=overrides(dict(
                shared, simulation_mode=True, skills_file=str(skills_path), release_timeout=.8))))
        self.preflight = self.add(MissionPreflight(context=self.context, parameter_overrides=overrides(dict(
            shared, route_file=str(route_path), require_distinct_endpoints=True))))
        self.manager = self.add(MissionManager(context=self.context, parameter_overrides=overrides(dict(
            shared, route_file=str(route_path), simulation_mode=True, autostart=False,
            startup_check_service='/mission/preflight', accepted_pose_topic='/localization/evidence',
            accepted_pose_type='evidence', accepted_pose_body_frame='livox_frame',
            accepted_pose_max_age=.8, server_timeout=.7, cancel_timeout=.7,
            localization_timeout=1.3, handoff_timeout=2., skill_safe_timeout=2.,
            relocalize_timeout=.7, stop_timeout=3., mission_time_limit=60.,
            max_navigation_retries=0, max_relocalization_retries=0))))
        self.probe = self.add(Node('backend_mission_probe', context=self.context,
            parameter_overrides=overrides(shared)))
        self.probe.create_subscription(String, '/mission/status',
            lambda m: self.statuses.append(self.json_event(m)), 50)
        self.probe.create_subscription(String, '/motion/status',
            lambda m: self.motion_status.append(self.json_event(m)), 50)
        self.probe.create_subscription(String, '/skills/status',
            lambda m: self.skill_events.append(self.json_event(m)), 50)
        self.probe.create_subscription(String, '/mission/preflight_status',
            lambda m: self.preflight_status.append(self.json_event(m)), 50)
        self.probe.create_subscription(MotionAuthority, '/motion/authority', self.record_authority, 50)
        self.probe.create_subscription(Twist, '/backend_sim/cmd_vel', self.record_output, 100)
        self.probe.create_subscription(Odometry, '/odom', self.record_odom, 100)
        self.probe.create_subscription(LocalizationEvidence, '/localization/evidence',
            self.record_evidence, 100)
        self.replay_pub = self.probe.create_publisher(LocalizationEvidence, '/gicp/evidence', 10)
        self.start_client = self.probe.create_client(Trigger, '/mission/start')
        self.native_parameters = (self.probe.create_client(GetParameters, '/obstacle_action_adapter/get_parameters')
                                  if native_skill else None)
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def spin(self):
        try:
            self.executor.spin()
        except Exception as error:
            self.errors.append(repr(error))

    def json_event(self, message):
        data = json.loads(message.data)
        data['observed_wall'] = time.monotonic() - self.created
        return data

    def record_authority(self, m):
        self.authorities.append(dict(wall=time.monotonic()-self.created, owner=m.owner,
            mission_id=m.mission_id, generation=m.generation, healthy=m.healthy, reason=m.reason))

    def record_output(self, m):
        authority = self.authorities[-1] if self.authorities else {}
        self.sink.append(dict(wall=time.monotonic()-self.created, values=twist_values(m),
                             observed_owner=authority.get('owner', 'UNKNOWN'),
                             generation=authority.get('generation', 0)))

    def record_odom(self, m):
        self.odom.append(dict(wall=time.monotonic()-self.created, stamp_ns=stamp_ns(m.header.stamp),
            frame=m.header.frame_id, child=m.child_frame_id,
            xy=[m.pose.pose.position.x, m.pose.pose.position.y], velocity=twist_values(m.twist.twist)))

    def record_evidence(self, m):
        self.last_evidence = copy.deepcopy(m)
        self.evidence.append(dict(wall=time.monotonic()-self.created, stamp_ns=stamp_ns(m.header.stamp),
            sequence=m.sequence, epoch=m.epoch, accepted=m.accepted, reason=m.reason,
            source_id=m.source_id))

    def capture(self):
        state = self.manager.fsm.state.value
        task = self.manager.route.current.id if self.manager.route.current else 'return'
        token = self.manager.fsm.pending.token if self.manager.fsm.pending else self.manager.fsm.sequence
        if not self.states or (state, task, token) != tuple(self.states[-1][k] for k in ('state', 'task', 'token')):
            self.states.append(dict(wall=time.monotonic()-self.created, state=state, task=task, token=token,
                owner=self.manager.handoff.owner, accepted_stamp=self.manager.localization.accepted.stamp_ns,
                pose=list(self.plant.pose), error=self.manager.fsm.error))

    def wait(self, condition, timeout=5.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.capture()
            if self.errors:
                raise AssertionError('ROS executor failed: ' + '; '.join(self.errors))
            if condition():
                return
            time.sleep(.01)
        self.capture()
        assert condition(), dict(state=self.manager.fsm.state.value, error=self.manager.fsm.error,
            preflight=self.preflight.evaluate(), motion=self.motion_status[-3:])

    def start(self):
        self.wait(lambda: self.preflight.evaluate()['ready'] and self.manager.startup_result_positive,
                  timeout=8.)
        self.report_extra['startup_preflight'] = self.preflight.evaluate()
        if self.native_parameters is not None:
            self.wait(self.native_parameters.service_is_ready)
            future = self.native_parameters.call_async(GetParameters.Request(names=['use_sim_time']))
            self.wait(future.done)
            values = future.result().values
            assert len(values)==1 and values[0].type==1 and values[0].bool_value==self.sim_time
            self.native_skill_clock = values[0].bool_value
            self.report_extra['native_skill_pid'] = self.native_skill_process.pid
            self.report_extra['native_skill_log'] = str(self.native_skill_log_path)
            self.report_extra['native_skill_clock'] = self.native_skill_clock
        self.wait(self.start_client.service_is_ready)
        future = self.start_client.call_async(Trigger.Request())
        self.wait(future.done)
        assert future.result().success, future.result().message
        self.started_at = time.monotonic()

    def terminal(self, timeout=35.):
        self.wait(lambda: self.manager.fsm.state in (State.FINISHED, State.ABORTED), timeout)
        # Drain cleanup and observe final-command fencing, not merely FSM result.
        time.sleep(.65)
        self.capture()

    def detach(self, node):
        # Simulate abrupt process disappearance: stop all timers/callbacks first.
        # Shutdown/destruction happens only during fixture cleanup afterwards.
        for timer in list(node.timers):
            timer.cancel()
        self.executor.remove_node(node)
        self.detached.append(node)
        self.nodes.remove(node)

    def mark_injection(self, label):
        self.injection = label
        self.injection_at = time.monotonic()-self.created
        self.report_extra['pose_at_injection'] = list(self.plant.pose)
        self.report_extra['nav_requests_at_injection'] = len(self.plant.nav_requests)

    def report(self):
        self.capture()
        return dict(case=self.name, elapsed=time.monotonic()-self.created,
            state=self.manager.fsm.state.value, reason=self.manager.fsm.error,
            completed=list(self.manager.route.completed), independent_plant_pose=list(self.plant.pose),
            waypoint_errors=self.manager.waypoint_errors, end_errors=self.manager.verifier.last_errors,
            transitions=self.states, mission_status=self.statuses, motion_status=self.motion_status,
            skill_events=self.skill_events, authority=self.authorities, final_commands=self.sink,
            accepted_evidence=self.evidence, measured_odometry=self.odom,
            nav_request_count=len(self.plant.nav_requests), injection=self.injection,
            injection_at=self.injection_at, extra=self.report_extra, executor_errors=self.errors,
            acceptance_limits=dict(xy=.10, yaw=.10, measured_stop_duration=1., exit_stable_duration=.5),
            use_sim_time=self.sim_time, node_clocks={**{n.get_name(): n.get_parameter('use_sim_time').value for n in self.nodes+self.detached},
                **({'obstacle_action_adapter': self.native_skill_clock} if self.native_skill_clock is not None else {})},
            rmw_implementation=os.environ.get('RMW_IMPLEMENTATION','default'),
            simulation_only=True, real_hardware_verified=False)

    def close(self):
        self.manager.closing = True
        self.manager._cleanup()
        self.arbiter.core.hold('TEST_TEARDOWN')
        time.sleep(.1)
        # Resolve the coroutine while the executor can still deliver its result.
        if self.skill is not None and self.skill.op:
            with self.skill.lock:
                self.skill._fault(self.skill.op, 'TEST_TEARDOWN')
        for node in self.nodes:
            for timer in list(node.timers):
                timer.cancel()
        self.executor.shutdown(timeout_sec=3.)
        self.thread.join(timeout=3.)
        for node in reversed(self.nodes+self.detached):
            try:
                node.destroy_node()
            except Exception as error:
                self.errors.append('teardown: '+repr(error))
        self.context.shutdown()
        if self.native_skill_process is not None:
            if self.native_skill_process.poll() is None:
                self.native_skill_process.terminate()
                try:
                    self.native_skill_process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    self.native_skill_process.kill()
                    self.native_skill_process.wait(timeout=3.)
            self.native_skill_log.close()
        assert not self.thread.is_alive(), 'ROS executor did not terminate'


@pytest.fixture
def rig_factory(tmp_path):
    instances = []
    def make(name, mode='success', sim_time=True, native_skill=False):
        rig = MissionRig(tmp_path, name, mode, sim_time, native_skill)
        instances.append(rig)
        return rig
    yield make
    REPORTS.mkdir(parents=True, exist_ok=True)
    for rig in reversed(instances):
        report = rig.report()
        try:
            rig.close()
        finally:
            report['teardown_errors'] = list(rig.errors)
            (REPORTS/(rig.name+'.json')).write_text(json.dumps(report, indent=2))


def assert_aborted_and_fenced(rig, expected_reason=None):
    assert rig.manager.fsm.state == State.ABORTED
    assert not rig.manager.route.completed
    assert rig.manager.fsm.error
    if expected_reason:
        assert expected_reason in rig.manager.fsm.error, rig.manager.fsm.error
    assert len(rig.plant.nav_requests) == 2, 'navigation continued after first obstacle fault'
    skill_started = next((t['wall'] for t in rig.states if t['state']=='EXECUTING_SKILL'), float('inf'))
    assert not any(t['state']=='RESTORING_NAV' and t['task']=='obstacle_x' and t['wall']>skill_started
                   for t in rig.states), 'navigation reacquired before verified exit'
    # Plant has an independent final-output watchdog as well as the live gate.
    assert max(abs(v) for v in rig.plant.measured_velocity) < 1e-9
    if rig.arbiter not in rig.detached:
        assert rig.sink and all(max(abs(v) for v in event['values']) < 1e-9 for event in rig.sink[-5:])


@pytest.mark.parametrize('sim_time', [False, True], ids=['wall_clock','sim_clock'])
def test_mission_two_distinct_obstacles_and_return(rig_factory, sim_time):
    rig = rig_factory('e2e_success_'+('sim_clock' if sim_time else 'wall_clock'), sim_time=sim_time)
    rig.start()
    rig.terminal()
    assert rig.manager.fsm.state == State.FINISHED, rig.manager.fsm.error
    assert rig.manager.route.completed == ['obstacle_x', 'obstacle_y']
    assert len(rig.plant.nav_requests) == 6
    assert math.hypot(*rig.plant.pose[:2]) <= .10
    assert abs(math.atan2(math.sin(rig.plant.pose[2]), math.cos(rig.plant.pose[2]))) <= .10
    assert rig.manager.waypoint_errors['distance'] <= .10
    assert rig.manager.waypoint_errors['yaw_error'] <= .10
    for task in ('obstacle_x', 'obstacle_y'):
        observed = [t['state'] for t in rig.states if t['task']==task]
        required = ['NAVIGATING','APPROACHING','WAIT_STOP','ACQUIRING_CONTROL','EXECUTING_SKILL',
                    'WAIT_SKILL_SAFE','RELEASING_CONTROL','WAIT_LOCALIZATION','VERIFYING_END','RESTORING_NAV']
        index = 0
        for state in observed:
            if index < len(required) and state == required[index]:
                index += 1
        assert index == len(required), (task, observed)
        waiting = next(t for t in rig.states if t['task']==task and t['state']=='WAIT_STOP')
        acquired = next(t for t in rig.states if t['task']==task and t['state']=='ACQUIRING_CONTROL')
        assert acquired['wall']-waiting['wall'] >= 1., 'continuous measured stop was bypassed'
        assert acquired['accepted_stamp'] > waiting['accepted_stamp']
        running = next(t for t in rig.states if t['task']==task and t['state']=='EXECUTING_SKILL')
        verified = next(t for t in rig.states if t['task']==task and t['state']=='RESTORING_NAV' and t['wall']>running['wall'])
        assert math.hypot(verified['pose'][0]-running['pose'][0], verified['pose'][1]-running['pose'][1]) > .15
        assert verified['accepted_stamp'] > running['accepted_stamp']
        assert verified['wall']-running['wall'] >= 2., 'skill/stop/exit dwell was bypassed'
    owners = {a['owner'] for a in rig.authorities}
    assert {'NAVIGATION','SKILL','SAFE_HOLD'} <= owners
    moving = [e for e in rig.sink if max(abs(v) for v in e['values']) > 1e-6]
    assert moving and {e['observed_owner'] for e in moving} == {'NAVIGATION','SKILL'}
    assert all(o['frame']=='nav_odom' and o['child']=='base_footprint' for o in rig.odom)
    assert sum(e['accepted'] for e in rig.evidence)>20
    assert rig.probe.count_publishers('/backend_sim/cmd_vel') == 1
    assert rig.probe.count_publishers('/cmd_vel') == 0
    assert any(e['event']=='released' for e in rig.skill_events)


@pytest.mark.parametrize('mode', ['fail','timeout','cancel_unconfirmed','release_failure','crash','wrong_exit','wrong_yaw'])
def test_skill_fault_or_success_outside_exit_never_restores_navigation(rig_factory, mode):
    rig = rig_factory('e2e_skill_'+mode, mode)
    rig.start()
    rig.terminal(timeout=20.)
    assert_aborted_and_fenced(rig)
    if mode in ('wrong_exit','wrong_yaw'):
        assert any(t['state']=='VERIFYING_END' for t in rig.states)
        assert 'end pose verification' in rig.manager.fsm.error
        assert any(e['event']=='released' for e in rig.skill_events), 'Action did not reach completed-release'
    else:
        assert not any(t['state']=='VERIFYING_END' for t in rig.states)
        expected = dict(fail='SKILL_EXECUTION_FAILED', timeout='SKILL_TIMEOUT',
                        cancel_unconfirmed='CANCEL_UNCONFIRMED', release_failure='CONTROL_RELEASE_UNCONFIRMED',
                        crash='EXECUTOR_EXCEPTION')[mode]
        assert any(e['event']=='fault' and e.get('error_code')==expected for e in rig.skill_events), rig.skill_events


@pytest.mark.parametrize('fault', ['rejected','stale','old_phase'])
def test_exit_requires_new_valid_localization_evidence(rig_factory, fault):
    rig = rig_factory('e2e_localization_'+fault)
    rig.start()
    rig.wait(lambda: rig.manager.fsm.state==State.WAIT_SKILL_SAFE, timeout=12.)
    rig.mark_injection(fault)
    if fault=='rejected':
        rig.plant.evidence_rejected = True
    else:
        rig.plant.evidence_enabled = False
        if fault=='old_phase':
            replay = copy.deepcopy(rig.last_evidence)
            # Keep the producer identity, sequence and measurement stamp from
            # the phase before exit acceptance; repeated packets cannot age it.
            rig.probe.create_timer(.03, lambda: rig.replay_pub.publish(replay))
    rig.terminal(timeout=12.)
    assert_aborted_and_fenced(rig)
    assert not any(t['state']=='VERIFYING_END' for t in rig.states), 'old/invalid evidence passed phase gate'
    assert any(not e['accepted'] for e in rig.evidence if e['wall'] >= rig.injection_at)


@pytest.mark.parametrize('fault', ['arbiter_exit','backend_exit','action_server_exit','unknown_owner','output_bypass','clock_pause'])
def test_required_backend_disappearance_or_unknown_control_blocks_progress(rig_factory, fault):
    rig = rig_factory('e2e_backend_'+fault, sim_time=True, native_skill=fault=='action_server_exit')
    rig.start()
    if fault=='backend_exit':
        rig.wait(lambda: rig.manager.fsm.state==State.WAIT_SKILL_SAFE, timeout=12.)
    else:
        rig.wait(lambda: rig.manager.fsm.state==State.EXECUTING_SKILL and rig.plant.skill_active, timeout=12.)
    if fault=='action_server_exit':
        # First prove the native server drove the actual final-output plant;
        # killing before any motion would not exercise output watchdog safety.
        rig.wait(lambda: any(e['observed_owner']=='SKILL' and max(abs(v) for v in e['values'])>1e-6
                             for e in rig.sink), timeout=1.)
        rig.report_extra['native_skill_motion_observed'] = True
    rig.mark_injection(fault)
    if fault=='arbiter_exit':
        rig.detach(rig.arbiter)
    elif fault=='backend_exit':
        rig.detach(rig.plant)
    elif fault=='action_server_exit':
        assert rig.native_skill_process is not None and rig.native_skill_process.poll() is None
        assert rig.native_skill_clock is True
        # SIGKILL cannot publish a fabricated terminal SUCCESS or safe release.
        rig.native_skill_process.kill()
        rig.report_extra['native_skill_exit_code'] = rig.native_skill_process.wait(timeout=3.)
        assert rig.report_extra['native_skill_exit_code'] == -9
    elif fault=='output_bypass':
        # Presence of a second final-command writer must fence the supported
        # software chain. DDS ACL enforcement remains a deployment boundary.
        rig.bypass_pub = rig.probe.create_publisher(Twist, '/backend_sim/cmd_vel', 1)
    elif fault=='unknown_owner':
        # Corrupt authoritative backend ownership, not a local Mission mock.
        rig.arbiter.core.owner='UNKNOWN'
    else:
        rig.plant.freeze_clock=True
    rig.terminal(timeout=13.)
    assert_aborted_and_fenced(rig)
    if fault=='arbiter_exit':
        assert not [e for e in rig.sink if e['wall']>rig.injection_at+.2], 'dead arbiter kept sending output'
    if fault=='output_bypass':
        assert any(e.get('bypass_detected') for e in rig.motion_status)
    if fault=='action_server_exit':
        assert not any(e.get('event')=='released' for e in rig.skill_events)
        assert rig.manager.obstacle.unknown_terminal or 'unknown' in rig.manager.fsm.error or 'ownership' in rig.manager.fsm.error
