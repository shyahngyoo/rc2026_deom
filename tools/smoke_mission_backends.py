#!/usr/bin/env python3
"""Independent-process smoke for the command-driven Mission backend launch.

Run after sourcing ROS Humble and the workspace overlay. This launches only a
simulation kinematic plant and simulated NavigateToPose controller. It does not
start Gazebo, deploy Nav2, or connect a robot driver. The only final output is
/backend_sim/cmd_vel on an isolated localhost ROS domain. Cleanup targets only
the process group created by this tool.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=151)
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parents[1]/
                        'docs/verification/launch_smoke')
    args = parser.parse_args()
    if not 1 <= args.domain <= 200 or args.timeout <= 0:
        parser.error('domain must be 1..200 and timeout must be positive')
    return args


def run(args):
    # Configure before importing/initializing RMW in the independent probe.
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    from ament_index_python.packages import get_package_share_directory
    cyclone_path = Path(get_package_share_directory('go2_mission_backends'))/'config/cyclonedds_sim.xml'
    if not cyclone_path.is_file():
        raise RuntimeError('Installed CycloneDDS simulation XML missing; rebuild go2_mission_backends: '+str(cyclone_path))
    # Match the independent probe and every launch child before RMW initializes.
    cyclone_uri = cyclone_path.as_uri()
    os.environ['CYCLONEDDS_URI'] = cyclone_uri
    import rclpy
    from geometry_msgs.msg import Twist
    from go2_obstacle_interfaces.msg import LocalizationEvidence, MotionAuthority
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String

    args.output_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.output_dir/'launch.log'
    report_path = args.output_dir/'report.json'
    start = time.monotonic()
    state = {'statuses': [], 'commands': [], 'authority': [], 'plant': [], 'evidence': [], 'odom': []}
    report = dict(simulation_only=True, real_hardware_verified=False,
        physics_model_verified=False, navigation_backend='simulated NavigateToPose controller; no Nav2/Gazebo',
        domain=args.domain, rmw_implementation=os.environ['RMW_IMPLEMENTATION'],
        cyclonedds_uri=os.environ['CYCLONEDDS_URI'], cyclonedds_config=str(cyclone_path), use_sim_time=True,
        launch_log=str(log_path), report_file=str(report_path), success=False)
    context = Context()
    executor = None
    probe = None
    process = None
    log = None
    try:
        rclpy.init(context=context)
        probe = Node('independent_launch_smoke_probe', context=context,
                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(probe)
        durable = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        def parse(kind, message):
            data = json.loads(message.data)
            data['observed_wall'] = time.monotonic()-start
            state[kind].append(data)
        probe.create_subscription(String, '/mission/status', lambda m: parse('statuses', m), durable)
        probe.create_subscription(String, '/backend_sim/plant_status', lambda m: parse('plant', m), 50)
        def command(m):
            t=m
            state['commands'].append(dict(wall=time.monotonic()-start,
                values=[t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z]))
        probe.create_subscription(Twist, '/backend_sim/cmd_vel', command, 100)
        def authority(m):
            state['authority'].append(dict(wall=time.monotonic()-start, owner=m.owner,
                healthy=m.healthy, generation=m.generation, mission_id=m.mission_id))
        probe.create_subscription(MotionAuthority, '/motion/authority', authority, 100)
        def evidence(m):
            state['evidence'].append(dict(wall=time.monotonic()-start, stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                sequence=m.sequence, epoch=m.epoch, source_id=m.source_id, accepted=m.accepted, reason=m.reason))
        probe.create_subscription(LocalizationEvidence, '/localization/evidence', evidence, 100)
        def odom(m):
            state['odom'].append(dict(wall=time.monotonic()-start, frame=m.header.frame_id,
                child=m.child_frame_id, stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                xy=[m.pose.pose.position.x,m.pose.pose.position.y],
                velocity=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]))
        probe.create_subscription(Odometry, '/odom', odom, 100)
        command_line = ['ros2', 'launch', 'go2_mission_backends', 'backend_simulation.launch.py',
            'ros_domain_id:='+str(args.domain), 'rmw_implementation:=rmw_cyclonedds_cpp',
            'use_sim_time:=true', 'autostart:=true', 'cyclonedds_uri:='+cyclone_uri]
        report['command'] = command_line
        log = log_path.open('w')
        process = subprocess.Popen(command_line, stdout=log, stderr=subprocess.STDOUT,
                                   env=dict(os.environ), start_new_session=True)
        report['launch_pid'] = process.pid
        report['owned_process_group'] = process.pid
        report['probe_pid'] = os.getpid()
        deadline = start+args.timeout
        finished_at = None
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
            if process.poll() is not None:
                raise AssertionError('launch exited before acceptance: '+str(process.returncode))
            # ExecuteProcess failures do not necessarily terminate ros2 launch.
            # A dead critical child means this complete-flow smoke cannot pass.
            current_log=log_path.read_text()
            if 'process has died' in current_log or 'startup_timeout:' in current_log:
                raise AssertionError('required launch child failed or startup timed out; inspect launch.log')
            if state['statuses']:
                status = state['statuses'][-1]
                if status.get('state')=='ABORTED':
                    raise AssertionError('Mission aborted: '+status.get('error',''))
                if status.get('state')=='FINISHED':
                    finished_at = finished_at or time.monotonic()
                    if time.monotonic()-finished_at>=1.:
                        break
        assert finished_at is not None, 'Mission did not finish within '+str(args.timeout)+' seconds'
        final = state['statuses'][-1]
        assert final['state']=='FINISHED' and final['completed_task_ids']==['obstacle_x','obstacle_y'], final
        assert state['commands'] and state['authority'] and state['plant'] and state['odom'] and state['evidence']
        assert all(max(abs(v) for v in m['values'])<1e-9 for m in state['commands'][-10:]), 'final command not continuously zero'
        last_authority=state['authority'][-1]
        assert last_authority['owner']=='SAFE_HOLD' and last_authority['healthy'], last_authority
        assert time.monotonic()-start-last_authority['wall']<.5, 'final authority heartbeat stale'
        pose=state['plant'][-1]['pose']
        assert math.hypot(pose[0],pose[1])<=.10 and abs(math.atan2(math.sin(pose[2]),math.cos(pose[2])))<=.10, pose
        assert {a['owner'] for a in state['authority']} >= {'SAFE_HOLD','NAVIGATION','SKILL'}
        assert any(max(abs(v) for v in m['values'])>1e-6 for m in state['commands']), 'plant never received actual motion'
        assert all(o['frame']=='nav_odom' and o['child']=='base_footprint' for o in state['odom'])
        assert sum(m['accepted'] for m in state['evidence'])>20
        assert probe.count_publishers('/backend_sim/cmd_vel')==1, 'final command writer not unique'
        assert probe.count_publishers('/cmd_vel')==0, 'hardware output unexpectedly advertised'
        for task in ('obstacle_x','obstacle_y'):
            states=[s['state'] for s in state['statuses'] if s['task_id']==task]
            expected=['NAVIGATING','APPROACHING','WAIT_STOP','ACQUIRING_CONTROL','EXECUTING_SKILL',
                'WAIT_SKILL_SAFE','RELEASING_CONTROL','WAIT_LOCALIZATION','VERIFYING_END','RESTORING_NAV']
            index=0
            for observed in states:
                if index<len(expected) and observed==expected[index]:index+=1
            assert index==len(expected), (task, states)
        content=log_path.read_text()
        children=re.findall(r'\[([^\]]+)\]: process started with pid \[(\d+)\]',content)
        report['child_processes']=[dict(name=name,pid=int(pid)) for name,pid in children]
        assert len({int(pid) for _,pid in children})==7, children
        assert process.pid not in {int(pid) for _,pid in children}
        report.update(success=True, final_status=final, final_authority=last_authority,
                      independent_return_pose=pose, elapsed=time.monotonic()-start)
    except Exception as error:
        report['failure']=repr(error)
    finally:
        if process is not None:
            # This PID was created with start_new_session; only its group is owned.
            report['cleanup_signals']=[]
            for sig, timeout in ((signal.SIGINT,8.),(signal.SIGTERM,3.),(signal.SIGKILL,3.)):
                try:
                    os.killpg(process.pid,sig)
                    report['cleanup_signals'].append(int(sig))
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    continue
            report['launch_return_code']=process.poll()
            report['launch_reaped']=process.poll() is not None
            if not report['launch_reaped']:
                report['success']=False
                report['failure']='owned launch could not be reaped'
        if log is not None:
            log.close()
            if 'child_processes' not in report:
                children=re.findall(r'\[([^\]]+)\]: process started with pid \[(\d+)\]',log_path.read_text())
                report['child_processes']=[dict(name=name,pid=int(pid)) for name,pid in children]
        alive=[c for c in report.get('child_processes',[]) if Path('/proc/'+str(c['pid'])).exists()]
        report['children_alive_after_cleanup']=alive
        if alive:
            report['success']=False
            report['failure']='owned launch children remained after process-group cleanup'
        if executor is not None:
            executor.shutdown()
        if probe is not None:probe.destroy_node()
        if context.ok():context.shutdown()
        report['observations']=state
        report['elapsed']=time.monotonic()-start
        report_path.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:report.get(k) for k in ['success','failure','elapsed','launch_pid','launch_reaped','report_file']}))
    return 0 if report['success'] else 1


if __name__=='__main__':
    sys.exit(run(arguments()))
