"""Real ROS topics/services + PTY firmware fixture; no physical robot is used.

All measured state and odometry below are explicit test fixtures. They prove
software wiring/fencing only, never firmware compatibility or motor safety.
"""
import errno
import json
import multiprocessing
import os
import select
import signal
import subprocess
import threading
import time
import traceback
import uuid
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import Twist
from go2_obstacle_interfaces.msg import MotionAuthority, MotionCommand, MotionTelemetry
from go2_obstacle_interfaces.srv import SetControlMode, GetMotionState
from go2_obstacle_interfaces.action import ExecuteObstacle
from rcl_interfaces.srv import GetParameters
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.context import Context
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_msgs.msg import String

from go2_mission_backends.motion_arbiter import MotionArbiter
from quadruped_stm32_bridge.gateway_core import BASE_FEATURES
from quadruped_stm32_bridge.nav_velocity_adapter import NavVelocityAdapter
from quadruped_stm32_bridge.protocol import FrameParser, MessageType as T, encode_frame
from quadruped_stm32_bridge.serial_gateway import SerialGateway
from quadruped_stm32_bridge.telemetry_adapter import TelemetryAdapter


class PtyController:
    """A protocol-v1 draft peer, intentionally not part of any installed node."""
    def __init__(self):
        import pty
        self.master, slave = pty.openpty()
        self.port = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(self.master, False)
        self.parser = FrameParser()
        self.stop = threading.Event()
        self.lock = threading.RLock()
        self.session = None
        self.sequence = 0
        self.frames = []
        self.pause_status = False
        self.error = None
        self.state = dict(known=True, mode="HOLD", stopped=True, safe_stance=True,
            control_releasable=True, nav_active=False, skill_active=False, obstacle_id="",
            skill_state="idle", request_id="", runner_session="", step_index=-1,
            estop=False, human_enable=True, fault_code="", capabilities=[6, 7, 9])
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _send(self, kind, payload):
        self.sequence += 1
        os.write(self.master, encode_frame(kind, self.sequence, self.session, payload))

    def _receive(self, frame):
        self.frames.append((time.monotonic(), frame))
        if frame.kind == T.HELLO:
            self.session = frame.session
            self.sequence = 0
        if self.session != frame.session:
            return
        ack = dict(acked_sequence=frame.sequence, acked_type=int(frame.kind), accepted=True)
        if frame.kind == T.HELLO:
            ack.update(protocol_version=1,
                features=sorted(BASE_FEATURES | {"SKILL_START", "SKILL_CANCEL", "STOP"}))
        self._send(T.ACK, ack)
        if frame.kind == T.SET_MODE:
            mode = frame.payload["mode"]
            self.state.update(mode=mode, nav_active=mode == "NAVIGATION",
                              skill_active=mode == "SKILL", control_releasable=False,
                              skill_state="idle")
        elif frame.kind == T.WALK:
            moving = bool(frame.payload["forward"] or frame.payload["turn"])
            self.state.update(stopped=not moving, control_releasable=False)
        elif frame.kind == T.SKILL_START:
            self.state.update(mode="SKILL", skill_active=True, nav_active=False,
                              skill_state="running", control_releasable=False)
            self.state.update({k: frame.payload[k] for k in
                               ("request_id", "runner_session", "step_index", "obstacle_id")})
        elif frame.kind in (T.HOLD, T.SKILL_CANCEL):
            self.state.update(mode="HOLD", stopped=True, safe_stance=True,
                control_releasable=True, nav_active=False, skill_active=False,
                skill_state="succeeded" if frame.payload.get("request_id") else "idle")
            self.state.update({k: frame.payload[k] for k in
                ("request_id", "runner_session", "step_index", "obstacle_id") if k in frame.payload})
        elif frame.kind == T.STOP:
            self.state.update(mode="DISABLED", stopped=True, safe_stance=False,
                              control_releasable=False, nav_active=False, skill_active=False)

    def _run(self):
        next_status = 0.
        try:
            while not self.stop.is_set():
                readable, _, _ = select.select([self.master], [], [], .005)
                if readable:
                    try:
                        data = os.read(self.master, 4096)
                    except OSError as error:
                        if error.errno in (errno.EIO, errno.EAGAIN, errno.EWOULDBLOCK):
                            # Readiness is a hint: a nonblocking PTY may have no
                            # bytes by the time read runs. This is not disconnect.
                            self.stop.wait(.005)
                            continue
                        raise
                    with self.lock:
                        for frame in self.parser.feed(data):
                            self._receive(frame)
                with self.lock:
                    now = time.monotonic()
                    if self.session is not None and not self.pause_status and now >= next_status:
                        self._send(T.STATUS, dict(self.state))
                        next_status = now + .025
        except BaseException:
            self.error = traceback.format_exc()

    def recorded(self, kind=None, after=0.):
        with self.lock:
            return [frame for at, frame in self.frames if at >= after
                    and (kind is None or frame.kind == kind)]

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)
        os.close(self.master)
        assert not self.thread.is_alive()
        assert self.error is None, self.error


class TestGraph(Node):
    """Only this fixture supplies synthetic odometry and a Nav action server."""
    __test__ = False
    def __init__(self, context):
        super().__init__("gateway_ros_test_fixture", context=context)
        self.authority = None
        self.telemetry = None
        self.events = []
        self.snapshots = []
        self.typed_nav = []
        self.nav_speed = self.skill_speed = 0.
        self.skill_heartbeat = False
        self.stale_generation = None
        self.nav_handle = None
        self.nav_result = None
        self.odom = self.create_publisher(Odometry, "/odom", 10)
        self.raw_nav = self.create_publisher(Twist, "/onboard/nav_cmd_raw", 10)
        self.skill = self.create_publisher(MotionCommand, "/motion/skill_command", 10)
        self.request = self.create_publisher(String, "/stm32/skill_request", 10)
        self.create_subscription(MotionAuthority, "/motion/authority", self._authority, 10)
        self.create_subscription(MotionTelemetry, "/motion/telemetry", self._telemetry, 10)
        self.create_subscription(MotionCommand, "/motion/nav_command", self.typed_nav.append, 10)
        self.create_subscription(String, "/stm32/status", self._snapshot, 10)
        self.create_subscription(String, "/stm32/event", self._event, 10)
        self.switch = self.create_client(SetControlMode, "/motion/set_control_mode")
        self.nav = ActionServer(self, NavigateToPose, "/navigate_to_pose", self._execute,
                                cancel_callback=lambda _: CancelResponse.ACCEPT,
                                callback_group=ReentrantCallbackGroup())
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.timer = self.create_timer(.025, self._tick)

    def _authority(self, message): self.authority = message
    def _telemetry(self, message): self.telemetry = message
    def _snapshot(self, message): self.snapshots.append(json.loads(message.data))
    def _event(self, message): self.events.append(json.loads(message.data))

    async def _execute(self, handle):
        self.nav_handle = handle
        self.nav_result = Future()
        return await self.nav_result

    def finish_navigation(self):
        if self.nav_result is not None and not self.nav_result.done():
            if self.nav_handle.is_cancel_requested:
                self.nav_handle.canceled()
            else:
                self.nav_handle.succeed()
            self.nav_result.set_result(NavigateToPose.Result())

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = "nav_odom", "base_footprint"
        odom.pose.pose.orientation.w = 1.
        self.odom.publish(odom)
        twist = Twist()
        twist.linear.x = self.nav_speed
        self.raw_nav.publish(twist)
        authority = self.authority
        if self.skill_heartbeat and authority is not None:
            command = MotionCommand()
            command.header.stamp, command.header.frame_id = stamp, "base_footprint"
            command.owner, command.mission_id = "SKILL", "ros_test_mission"
            command.generation = authority.generation if self.stale_generation is None else self.stale_generation
            command.twist.linear.x = self.skill_speed
            self.skill.publish(command)

    def change_owner(self, owner, expected):
        request = SetControlMode.Request(owner=owner, expected_owner=expected,
            mission_id="ros_test_mission", request_id=uuid.uuid4().hex, lease_duration=5.)
        future = self.switch.call_async(request)
        wait(lambda: future.done(), diagnostic=lambda: self.summary())
        return future.result()

    def start_navigation(self):
        assert self.nav_client.wait_for_server(timeout_sec=3.)
        goal = self.nav_client.send_goal_async(NavigateToPose.Goal())
        wait(lambda: goal.done(), diagnostic=lambda: self.summary())
        handle = goal.result()
        assert handle.accepted
        wait(lambda: self.nav_handle is not None, diagnostic=lambda: self.summary())
        return handle

    def summary(self):
        return dict(authority=str(self.authority), telemetry=str(self.telemetry),
                    snapshot=self.snapshots[-1:] , events=self.events[-5:])


def wait(predicate, timeout=5., diagnostic=lambda: {}):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("condition timed out: " + repr(diagnostic()))


def parameters(**values):
    return [Parameter(key, value=value) for key, value in values.items()]


def scenario(enabled):
    # Separate process/domain avoids external ROS graphs affecting publisher-
    # uniqueness checks, while production checks remain completely unchanged.
    os.environ["ROS_DOMAIN_ID"] = "183"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    dds = Path(__file__).resolve().parents[3] / "common/go2_mission_backends/config/cyclonedds_sim.xml"
    os.environ["CYCLONEDDS_URI"] = "file://" + str(dds)
    context = Context()
    rclpy.init(context=context)
    peer = PtyController()
    nodes = []
    gateway = None
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    thread = None
    try:
        graph = TestGraph(context)
        nodes.append(graph)
        gateway = SerialGateway(context=context, parameter_overrides=parameters(
            port=peer.port, enable_motion=enabled, test_mode=not enabled,
            calibration_confirmed=enabled))
        nodes.append(gateway)
        telemetry = TelemetryAdapter(context=context)
        nodes.append(telemetry)
        arbiter = MotionArbiter(context=context, parameter_overrides=parameters(
            enable_output=enabled, simulation_mode=False, telemetry_source_id="stm32",
            output_topic="/stm32/arbiter_cmd_vel", authorized_output_topic="/stm32/arbiter_command"))
        nodes.append(arbiter)
        nav = NavVelocityAdapter(context=context)
        nodes.append(nav)
        for node in nodes:
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        wait(lambda: gateway.core.handshake and graph.telemetry is not None,
             diagnostic=lambda: graph.summary())
        assert peer.recorded(T.HELLO)
        assert graph.switch.wait_for_service(timeout_sec=3.)
        if not enabled:
            graph.nav_speed = .2
            graph.start_navigation()
            response = graph.change_owner("NAVIGATION", "SAFE_HOLD")
            assert not response.accepted and response.message == "OUTPUT_DISABLED"
            time.sleep(.4)
            assert {frame.kind for frame in peer.recorded()} == {T.HELLO}
            assert gateway.core.motion_enabled is False
            assert graph.authority.owner == "SAFE_HOLD"
            assert not graph.typed_nav
            return dict(disabled=True, frames=[int(frame.kind) for frame in peer.recorded()])
        wait(lambda: arbiter.safe_switch() and gateway.core.session_authority_seen,
             diagnostic=lambda: graph.summary())
        assert gateway.core.fenced_generation >= 1
        assert graph.change_owner("NAVIGATION", "SAFE_HOLD").accepted
        wait(lambda: graph.authority.owner == "NAVIGATION", diagnostic=lambda: graph.summary())
        old_generation = graph.authority.generation
        graph.start_navigation()
        graph.nav_speed = .1
        wait(lambda: any(frame.payload.get("forward", 0) > 0 for frame in peer.recorded(T.WALK)),
             diagnostic=lambda: graph.summary())
        walk = next(frame for frame in peer.recorded(T.WALK) if frame.payload.get("forward", 0) > 0)
        assert walk.payload["owner"] == "NAVIGATION"
        assert walk.payload["generation"] == old_generation
        assert walk.payload["forward"] == pytest.approx(.2)
        assert any(frame.payload["mode"] == "NAVIGATION" for frame in peer.recorded(T.SET_MODE))
        assert graph.typed_nav and all(message.twist.linear.y == 0. for message in graph.typed_nav)
        # Action terminal status requests measured HOLD before ownership handoff.
        graph.nav_speed = 0.
        graph.finish_navigation()
        wait(lambda: arbiter.safe_switch(), diagnostic=lambda: graph.summary())
        assert graph.change_owner("SKILL", "NAVIGATION").accepted
        wait(lambda: graph.authority.owner == "SKILL", diagnostic=lambda: graph.summary())
        generation = graph.authority.generation
        assert generation > old_generation
        graph.skill_heartbeat = True
        graph.request.publish(String(data=json.dumps(dict(op="command", command_id=2,
            request_id=uuid.uuid4().hex, runner_session="test_runner", obstacle_id="test_obstacle",
            step_index=0, owner="SKILL", mission_id="ros_test_mission", generation=generation))))
        wait(lambda: gateway.core.status and gateway.core.status["mode"] == "SKILL",
             diagnostic=lambda: graph.summary())
        handoff_at = time.monotonic()
        graph.skill_speed = .06
        graph.nav_speed = .35  # Deliberate raw Nav2 interference, without authority.
        wait(lambda: any(frame.payload.get("owner") == "SKILL" and
                         frame.payload.get("forward", 0) > 0
                         for frame in peer.recorded(T.WALK, handoff_at)),
             diagnostic=lambda: graph.summary())
        time.sleep(.15)
        skill_frames = peer.recorded(T.WALK, handoff_at)
        assert skill_frames and all(frame.payload["owner"] == "SKILL" for frame in skill_frames)
        assert all(frame.payload["forward"] in (0., .12) for frame in skill_frames)
        # Replayed old typed input cannot be rebound to the new authority.
        replay_at = time.monotonic()
        graph.stale_generation = old_generation
        graph.skill_speed = .4
        time.sleep(.15)
        assert all(frame.payload["forward"] != pytest.approx(.8)
                   for frame in peer.recorded(T.WALK, replay_at))
        # A status-silent peer still ACKs, proving ACK is not fresh telemetry.
        peer.pause_status = True
        wait(lambda: graph.telemetry is not None and not graph.telemetry.known
             and graph.authority.owner == "SAFE_HOLD", diagnostic=lambda: graph.summary())
        wait(lambda: bool(peer.recorded(T.HOLD, replay_at)), diagnostic=lambda: graph.summary())
        fenced_at = time.monotonic()
        time.sleep(.15)
        assert not peer.recorded(T.WALK, fenced_at)
        assert not graph.telemetry.safe_stance and not graph.telemetry.control_releasable
        return dict(disabled=False, navigation_generation=old_generation,
                    skill_generation=generation, frames=len(peer.recorded()),
                    final_owner=graph.authority.owner, telemetry_known=graph.telemetry.known)
    finally:
        if nodes and isinstance(nodes[0], TestGraph):
            nodes[0].finish_navigation()
        if gateway is not None:
            gateway.close()
        executor.shutdown(timeout_sec=2.)
        if thread is not None:
            thread.join(timeout=2.)
        for node in reversed(nodes):
            if isinstance(node, TestGraph):
                node.nav.destroy()
            node.destroy_node()
        peer.close()
        rclpy.shutdown(context=context)


def child(sender, enabled):
    try:
        sender.send((True, scenario(enabled)))
    except BaseException:
        sender.send((False, traceback.format_exc()))
    finally:
        sender.close()


def run_scenario(enabled):
    process_context = multiprocessing.get_context("spawn")
    receiver, sender = process_context.Pipe(duplex=False)
    process = process_context.Process(target=child, args=(sender, enabled))
    process.start()
    sender.close()
    try:
        assert receiver.poll(25.), "isolated serial ROS scenario timed out"
        okay, result = receiver.recv()
        assert okay, result
        return result
    finally:
        receiver.close()
        process.join(timeout=3.)
        if process.is_alive():
            process.terminate()
            process.join(timeout=3.)


def test_default_disabled_real_ros_graph_only_performs_hello():
    assert run_scenario(False)["disabled"]


def test_real_ros_navigation_skill_handoff_status_timeout_and_old_generation_fencing():
    result = run_scenario(True)
    assert result["final_owner"] == "SAFE_HOLD" and not result["telemetry_known"], result


def default_launch_scenario(directory):
    """Start the installed Launch unchanged and inspect its real ROS graph."""
    os.environ["ROS_DOMAIN_ID"] = "184"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    dds = Path(__file__).resolve().parents[3] / "common/go2_mission_backends/config/cyclonedds_sim.xml"
    os.environ["CYCLONEDDS_URI"] = "file://" + str(dds)
    context = Context()
    rclpy.init(context=context)
    observer = Node("default_stm32_launch_observer", context=context)
    messages, commands = [], []
    observer.create_subscription(String, "/stm32/status", lambda m: messages.append(json.loads(m.data)), 10)
    observer.create_subscription(MotionCommand, "/stm32/arbiter_command", commands.append, 10)
    state = observer.create_client(GetMotionState, "/motion/get_state")
    gateway_parameters = observer.create_client(GetParameters, "/serial_gateway/get_parameters")
    arbiter_parameters = observer.create_client(GetParameters, "/motion_arbiter/get_parameters")
    action = ActionClient(observer, ExecuteObstacle, "/execute_obstacle")
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(observer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    log_path = Path(directory) / "stm32_default_launch.log"
    logfile = log_path.open("w")
    process = subprocess.Popen(["ros2", "launch", "quadruped_stm32_bridge", "stm32_bringup.launch.py"],
        stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ))
    try:
        expected = {"serial_gateway", "nav_velocity_adapter", "stm32_telemetry_adapter", "motion_arbiter"}
        wait(lambda: expected <= set(observer.get_node_names()) and state.service_is_ready()
             and gateway_parameters.service_is_ready() and arbiter_parameters.service_is_ready()
             and bool(messages), timeout=8.,
             diagnostic=lambda: dict(nodes=observer.get_node_names(), status=messages[-1:],
                                     launch_log=log_path.read_text()))
        assert process.poll() is None
        names = set(observer.get_node_names()) - {observer.get_name()}
        assert names == expected, names
        assert all("sim" not in name and "mock" not in name for name in names)
        state_future = state.call_async(GetMotionState.Request())
        params_future = gateway_parameters.call_async(GetParameters.Request(
            names=["port", "enable_motion", "test_mode", "calibration_confirmed", "use_sim_time"]))
        arbiter_future = arbiter_parameters.call_async(GetParameters.Request(
            names=["enable_output", "simulation_mode", "use_sim_time"]))
        wait(lambda: all(future.done() for future in (state_future, params_future, arbiter_future)))
        measured = state_future.result()
        assert not measured.known and not measured.safe_stance and not measured.control_releasable
        params = params_future.result().values
        assert params[0].string_value == ""
        assert [value.bool_value for value in params[1:]] == [False, True, False, False]
        assert [value.bool_value for value in arbiter_future.result().values] == [False, False, False]
        assert not action.server_is_ready()
        time.sleep(.2)
        assert all(m["connected"] is False and m["handshake"] is False and m["session"] == 0
                   for m in messages)
        assert commands and all(not any((m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
            m.twist.angular.x, m.twist.angular.y, m.twist.angular.z)) for m in commands)
        assert "/cmd_vel" not in {name for name, _ in observer.get_topic_names_and_types()}
        return dict(nodes=sorted(names), unknown=not measured.known,
                    uart_connected=messages[-1]["connected"], action_present=action.server_is_ready(),
                    log=str(log_path))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=4.)
        logfile.close()
        executor.shutdown(timeout_sec=2.)
        thread.join(timeout=2.)
        observer.destroy_node()
        rclpy.shutdown(context=context)


def launch_child(sender, directory):
    try:
        sender.send((True, default_launch_scenario(directory)))
    except BaseException:
        sender.send((False, traceback.format_exc()))
    finally:
        sender.close()


def test_installed_default_launch_isolates_hardware_and_keeps_motion_disabled(tmp_path):
    process_context = multiprocessing.get_context("spawn")
    receiver, sender = process_context.Pipe(duplex=False)
    process = process_context.Process(target=launch_child, args=(sender, str(tmp_path)))
    process.start()
    sender.close()
    try:
        assert receiver.poll(25.), "default installed launch test timed out"
        okay, result = receiver.recv()
        assert okay, result
        assert result["unknown"] and not result["uart_connected"] and not result["action_present"]
    finally:
        receiver.close()
        process.join(timeout=3.)
        if process.is_alive():
            process.terminate()
            process.join(timeout=3.)
