"""Run the formal Mission launch with missing real preflight; never fake hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger
from go2_obstacle_interfaces.srv import GetMotionState, SetControlMode


def test_installed_real_mission_launch_stays_idle_without_hardware_and_has_no_motor_output(tmp_path):
    log_dir = Path(os.environ.get("RC2026_REPORT_DIR", str(tmp_path)))
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "real_mission_launch.log"
    log = log_path.open("w")
    process = subprocess.Popen([
        "ros2", "launch", "robocon_mission", "mission.launch.py",
        "startup_check_service:=/cleanup_test/missing_real_preflight",
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    context = Context(); rclpy.init(context=context)
    node = rclpy.create_node("real_launch_isolation_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    statuses = []
    node.create_subscription(String, "/mission/status", lambda m: statuses.append(json.loads(m.data)), 10)

    def wait(predicate, timeout=12.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            assert process.poll() is None, log_path.read_text()
            executor.spin_once(timeout_sec=.03)
        assert predicate(), log_path.read_text()

    try:
        parameters = node.create_client(GetParameters, "/mission_manager/get_parameters")
        wait(parameters.service_is_ready)
        future = parameters.call_async(GetParameters.Request(names=["use_sim_time", "autostart", "simulation_mode", "startup_check_service"]))
        wait(future.done)
        values = future.result().values
        assert [v.bool_value for v in values[:3]] == [False, False, False]
        assert values[3].string_value == "/cleanup_test/missing_real_preflight"
        wait(lambda: bool(statuses))
        assert statuses[-1]["state"] == "IDLE"
        start = node.create_client(Trigger, "/mission/start")
        wait(start.service_is_ready)
        future = start.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success is False
        assert future.result().message
        assert statuses[-1]["state"] == "IDLE"
        names = set(node.get_node_names())
        assert not any(name.startswith("mock_") or name == "simulation_backend" for name in names)
        assert node.count_publishers("/cmd_vel") == 0
        assert node.count_publishers("/backend_sim/cmd_vel") == 0
        assert node.count_publishers("/motion/telemetry") == 0
        assert node.count_publishers("/motion/control_state") == 0
        # Service names can be advertised by clients too. Check for actual
        # responders, preserving the no-fake-hardware-service requirement.
        hardware_servers = {
            name: node.create_client(kind, name).service_is_ready()
            for name, kind in (("/motion/set_control_mode", SetControlMode),
                               ("/motion/get_state", GetMotionState),
                               ("/mission/localization_ok", Trigger))
        }
        assert not any(hardware_servers.values()), hardware_servers
        (log_dir / "real_mission_isolation.json").write_text(json.dumps(dict(
            nodes=sorted(names), defaults=dict(use_sim_time=False, autostart=False, simulation_mode=False),
            missing_preflight_start_rejected=True, state=statuses[-1]["state"],
            hardware_servers=hardware_servers, motor_publishers=node.count_publishers("/cmd_vel")), indent=2))
    finally:
        executor.shutdown()
        node.destroy_node(); context.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try: process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=3.)
        log.close()
