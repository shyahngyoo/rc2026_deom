"""Real ROS topics/services on a hardware-free isolated ROS domain."""
import time
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from go2_obstacle_interfaces.msg import LocalizationEvidence
from go2_mission_backends.localization_adapter import LocalizationAdapter
from robocon_mission.localization_client import LocalizationClient


@pytest.fixture
def rig():
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    nodes = []
    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node
    def spin(predicate, timeout=2.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return True
        return predicate()
    yield add, spin, executor
    for node in reversed(nodes):
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


def message(node, seq=1, epoch=1, accepted=True, stamp_ns=None, source="test-source"):
    m = LocalizationEvidence()
    ns = node.get_clock().now().nanoseconds if stamp_ns is None else stamp_ns
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1000000000)
    m.header.frame_id = "map"
    m.source_id, m.epoch, m.sequence = source, epoch, seq
    m.accepted, m.covariance_known = accepted, False
    m.pose.pose.orientation.w = 1.
    m.body_frame = "livox_frame"
    m.reason = "ACCEPTED" if accepted else "TEST_REJECTED"
    return m


def test_atomic_adapter_rejection_fresh_recovery_and_unsynced_legacy(rig):
    add, spin, _ = rig
    adapter = add(LocalizationAdapter(parameter_overrides=[Parameter("evidence_max_age", value=.3)]))
    sender = add(Node("evidence_sender"))
    publisher = sender.create_publisher(LocalizationEvidence, "/gicp/evidence", 10)
    legacy_pose = sender.create_publisher(PoseStamped, "/gicp/pose", 10)
    legacy_valid = sender.create_publisher(Bool, "/gicp/valid", 10)
    received = []
    receiver = sender.create_subscription(LocalizationEvidence, "/localization/evidence", received.append, 10)
    service = sender.create_client(Trigger, "/mission/localization_ok")
    assert spin(lambda: publisher.get_subscription_count() == 1 and service.service_is_ready())
    legacy_valid.publish(Bool(data=True))
    old = PoseStamped(); old.header.frame_id="map"; old.pose.orientation.w=1.
    legacy_pose.publish(old)
    future = service.call_async(Trigger.Request())
    assert spin(future.done)
    assert not future.result().success  # Bool+Pose cannot create accepted evidence.
    first = message(sender)
    publisher.publish(first)
    assert spin(lambda: len(received) >= 1)
    assert received[-1].accepted
    assert received[-1].header.stamp == first.header.stamp
    assert not received[-1].covariance_known
    future = service.call_async(Trigger.Request()); assert spin(future.done)
    assert future.result().success
    publisher.publish(message(sender, 2, accepted=False))
    assert spin(lambda: len(received) >= 2)
    assert not received[-1].accepted
    future = service.call_async(Trigger.Request()); assert spin(future.done)
    assert not future.result().success
    publisher.publish(message(sender, 3, stamp_ns=first.header.stamp.sec*1000000000+first.header.stamp.nanosec))
    assert spin(lambda: len(received) >= 3)
    assert not received[-1].accepted  # cached pre-fault measurement is insufficient.
    publisher.publish(message(sender, 4))
    assert spin(lambda: len(received) >= 4)
    assert received[-1].accepted
    assert spin(lambda: len(received) >= 5, timeout=.8)
    assert not received[-1].accepted  # node exit/input interruption expires on wall time.
    future = service.call_async(Trigger.Request()); assert spin(future.done)
    assert not future.result().success


def test_mission_client_phase_and_source_evidence_on_ros_topics(rig):
    add, spin, _ = rig
    sender = add(Node("mission_evidence_sender"))
    consumer = add(Node("mission_evidence_consumer"))
    client = LocalizationClient(consumer, "/test_unused_health", "map", "base_footprint",
                                .5, 1., False, accepted_pose_topic="/test/evidence",
                                accepted_pose_type="evidence")
    pub = sender.create_publisher(LocalizationEvidence, "/test/evidence", 10)
    assert spin(lambda: pub.get_subscription_count() == 1)
    first = message(sender)
    pub.publish(first)
    assert spin(lambda: client.accepted.sequence == 1)
    assert client.accepted_fresh()
    client.require_new_accepted_pose()
    assert not client.accepted_fresh()
    pub.publish(message(sender, 2, stamp_ns=first.header.stamp.sec*1000000000+first.header.stamp.nanosec))
    assert spin(lambda: client.accepted.sequence == 2)
    assert not client.accepted_fresh()
    pub.publish(message(sender, 3))
    assert spin(lambda: client.accepted.sequence == 3)
    assert client.accepted_fresh()
    pub.publish(message(sender, 4, source="previous-process"))
    assert spin(lambda: client.accepted.reason == "SOURCE_CONFLICT")
    assert not client.accepted_fresh()
    pub.publish(message(sender, 5))
    assert spin(lambda: client.accepted.sequence == 5)
    assert client.accepted_fresh()


def test_paused_clock_invalidates_adapter_despite_repeated_clock_messages(rig):
    add, spin, _ = rig
    adapter = add(LocalizationAdapter(parameter_overrides=[Parameter("use_sim_time", value=True),
                                                           Parameter("clock_timeout", value=.15)]))
    sender = add(Node("sim_clock_evidence_sender"))
    clock_pub = sender.create_publisher(Clock, "/clock", 10)
    pub = sender.create_publisher(LocalizationEvidence, "/gicp/evidence", 10)
    c = Clock(); c.clock.sec = 10
    assert spin(lambda: pub.get_subscription_count() == 1)
    clock_pub.publish(c)
    assert spin(lambda: adapter.get_clock().now().nanoseconds == int(10e9))
    pub.publish(message(sender, stamp_ns=int(10e9)))
    assert spin(lambda: adapter.health.stamp_ns is not None)
    # /clock packets with unchanged simulated time must not refresh progress.
    timer = sender.create_timer(.02, lambda: clock_pub.publish(c))
    assert spin(lambda: adapter.health.reason == "CLOCK_STALLED", timeout=.7)
    assert adapter.health.stamp_ns is None
    c.clock.sec = 11
    assert spin(lambda: adapter.get_clock().now().nanoseconds == int(11e9))
    pub.publish(message(sender, 2, stamp_ns=int(10e9)))
    assert spin(lambda: adapter.health.sequence == 2)
    assert adapter.health.stamp_ns is None
    pub.publish(message(sender, 3, stamp_ns=int(11e9)))
    assert spin(lambda: adapter.health.sequence == 3)
    assert adapter.health.stamp_ns == int(11e9)
    c.clock.sec = 2
    assert spin(lambda: adapter.health.reason == "CLOCK_ROLLBACK")
    pub.publish(message(sender, 4, stamp_ns=int(2e9), epoch=1))
    assert spin(lambda: adapter.health.reason == "OLD_EPOCH")
    pub.publish(message(sender, 5, stamp_ns=int(2e9), epoch=2))
    assert spin(lambda: adapter.health.epoch == 2)
    assert adapter.health.stamp_ns == int(2e9)
    sender.destroy_timer(timer)


def test_real_gicp_publisher_atomic_pose_stamp_epoch_with_synthetic_cloud(rig, tmp_path):
    """Native FAST-LIO-like input process -> real GICP -> Python evidence observer.

    Uses the same PCD/pose relationship and default GICP quality checks; no
    accepted result is synthesized. The clock is controlled to avoid unrelated
    WSL host wall-clock adjustments during the cross-process regression.
    """
    import json
    import random
    import subprocess
    from pathlib import Path
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.clock import Clock as SteadyClock, ClockType

    from ament_index_python.packages import get_package_prefix
    binary_dir = Path(get_package_prefix("go2_real_localization"))/"lib"/"go2_real_localization"
    executable = binary_dir / "gicp_localizer"
    producer_executable = binary_dir/"synthetic_gicp_inputs"
    assert executable.exists() and producer_executable.exists(), "build go2_real_localization with BUILD_TESTING=ON"
    rng = random.Random(71)
    points = [(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-1, 3)) for _ in range(1800)]
    pcd = tmp_path/"synthetic.pcd"
    pcd.write_text("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1800\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1800\nDATA ascii\n" +
                   "".join(f"{x:.7f} {y:.7f} {z:.7f}\n" for x,y,z in points))
    add,spin,_ = rig
    sender=add(Node("real_gicp_test_observer",parameter_overrides=[Parameter("use_sim_time",value=True)]))
    clock_pub=sender.create_publisher(Clock,"/clock",10)
    steady=SteadyClock(clock_type=ClockType.STEADY_TIME)
    start=time.monotonic()
    def advance_clock():
        c=Clock();c.clock.sec,c.clock.nanosec=divmod(int((100.+time.monotonic()-start)*1e9),1000000000)
        clock_pub.publish(c)
    clock_timer=sender.create_timer(.02,advance_clock,clock=steady)
    sent_stamps=set()
    def observe_input(m):
        assert m.header.frame_id == "odom" and m.child_frame_id == "livox_frame"
        sent_stamps.add(m.header.stamp.sec*1000000000+m.header.stamp.nanosec)
    input_sub=sender.create_subscription(Odometry,"/test_real_gicp/odom",observe_input,10)
    initial_pub=sender.create_publisher(PoseWithCovarianceStamped,"/test_real_gicp/initialpose",10)
    events=[]
    evidence_sub=sender.create_subscription(LocalizationEvidence,"/test_real_gicp/gicp/evidence",events.append,10)
    input_status=[]
    def observe_status(m):
        input_status.append(json.loads(m.data))
    status_sub=sender.create_subscription(String,"/test_real_gicp/gicp/input_status",observe_status,10)
    logs=[open(tmp_path/"gicp.log","w"),open(tmp_path/"native_inputs.log","w")]
    processes=[]
    try:
        processes.append(subprocess.Popen([str(executable),"--ros-args","-r","__ns:=/test_real_gicp",
            "-r","/initialpose:=/test_real_gicp/initialpose","-p","map_path:="+str(pcd),
            "-p","use_sim_time:=true","-p","initial_pose_enabled:=true",
            "-p","initial_pose:=[1.0,0.0,0.0,0.0,0.0,0.0]",
            "-p","cloud_topic:=/test_real_gicp/cloud","-p","odom_topic:=/test_real_gicp/odom",
            "-p","registration_period:=0.1","-p","num_threads:=1"],stdout=logs[0],stderr=subprocess.STDOUT))
        processes.append(subprocess.Popen([str(producer_executable),"--ros-args","-p","map_path:="+str(pcd),
                                           "-p","use_sim_time:=true"],stdout=logs[1],stderr=subprocess.STDOUT))
        def measurement_ns(e):
            return e.header.stamp.sec*1000000000+e.header.stamp.nanosec
        # Several distinct solver measurements prove a continuing input stream,
        # rather than one cached cloud re-registered after an initialpose reset.
        assert spin(lambda:len({measurement_ns(e) for e in events if e.accepted})>=3,timeout=10.), {
            "evidence": [(e.accepted,e.reason,measurement_ns(e)) for e in events[-10:]],
            "input_status": input_status[-3:],
            "observed_odom_stamps": len(sent_stamps),
        }
        before_reset=[e for e in events if e.accepted]
        assert all(measurement_ns(a)<measurement_ns(b) for a,b in zip(before_reset,before_reset[1:]))
        assert spin(lambda:any(s["cloud_count"]>=3 and s["odom_count"]>=3 and s["pair_count"]>=3 for s in input_status))
        first=before_reset[0]
        stamp_ns=first.header.stamp.sec*1000000000+first.header.stamp.nanosec
        assert spin(lambda:stamp_ns in sent_stamps)
        assert abs(first.pose.pose.position.x-1.) < .02
        assert abs(first.pose.pose.position.y) < .02
        assert first.source_id and not first.covariance_known
        initial=PoseWithCovarianceStamped();initial.header.frame_id="map"
        initial.header.stamp=sender.get_clock().now().to_msg()
        initial.pose.pose.position.x=1.;initial.pose.pose.orientation.w=1.
        reset_ns=initial.header.stamp.sec*1000000000+initial.header.stamp.nanosec
        initial_pub.publish(initial)
        assert spin(lambda:any(e.epoch>first.epoch and not e.accepted and e.reason=="INITIAL_POSE_RECEIVED" for e in events))
        assert spin(lambda:any(e.accepted and e.epoch>first.epoch and measurement_ns(e)>reset_ns for e in events),timeout=5.), {
            "reset_stamp_ns": reset_ns,
            "evidence": [(e.accepted,e.epoch,measurement_ns(e)) for e in events[-10:]],
            "input_status": input_status[-3:],
        }
        assert all(a.sequence < b.sequence for a,b in zip(events,events[1:]))
    finally:
        sender.destroy_timer(clock_timer)
        for process in reversed(processes):
            process.terminate()
            try: process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                process.kill();process.wait(timeout=3.)
        for log in logs:log.close()
        (tmp_path/"input_status.json").write_text(json.dumps(input_status,indent=2))
