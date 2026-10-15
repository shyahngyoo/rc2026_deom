"""Source/install contracts for the real-only cleanup; no ROS or hardware starts."""
import ast
from pathlib import Path
import re
import pytest

ROOT = Path(__file__).resolve().parents[2]
REMOVED_CONSOLES = {"mock_localization_ok", "mock_motion_services", "mock_mission_demo", "mock_obstacle_skill"}
REMOVED_LAUNCHES = (
    "common/robocon_mission/launch/simulation.launch.py",
    "common/go2_nav_bringup/launch/localization.launch.py",
    "common/go2_nav_bringup/launch/mapping.launch.py",
    "common/go2_nav_bringup/launch/navigation.launch.py",
    "common/go2_fastlio_localization/launch/localization.launch.py",
    "common/go2_fastlio_localization/launch/mapping.launch.py",
    "common/go2_fastlio_localization/launch/odometry.launch.py",
    "common/go2_fastlio_localization/launch/real_fastlio.launch.py",
    "common/go2_fastlio_localization/launch/real_lidar.launch.py",
    "common/go2_fastlio_localization/launch/real_mapping.launch.py",
    "real/go2_real_localization/launch/bringup.launch.py",
)


def console_entries():
    for setup in ROOT.rglob("setup.py"):
        for node in ast.walk(ast.parse(setup.read_text())):
            if isinstance(node, ast.keyword) and node.arg == "entry_points":
                for entry in ast.literal_eval(node.value).get("console_scripts", []):
                    name, target = (v.strip() for v in entry.split("=", 1))
                    yield setup.parent, name, target


def test_removed_mock_programs_are_not_registered_or_runtime_modules():
    entries = list(console_entries())
    assert not REMOVED_CONSOLES.intersection(name for _, name, _ in entries)
    for name in ("mock_localization_ok", "mock_motion_services", "mock_mission_demo"):
        assert not (ROOT / "common/robocon_mission/robocon_mission" / (name + ".py")).exists()
    assert not (ROOT / "common/go2_nav_bringup/scripts/mock_obstacle_skill.py").exists()
    assert not (ROOT / "common/robocon_mission/config/mock_closed_loop_route.yaml").exists()
    assert "scripts/mock_obstacle_skill.py" not in (ROOT / "common/go2_nav_bringup/CMakeLists.txt").read_text()


def test_every_registered_module_and_cmake_program_exists():
    names = set()
    for package, name, target in console_entries():
        module, function = target.split(":", 1)
        path = package.joinpath(*module.split(".")).with_suffix(".py")
        assert path.is_file(), (name, path)
        tree = ast.parse(path.read_text())
        assert any(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == function
                   for n in tree.body), (name, function)
        names.add(name)
    assert {"mission_manager", "motion_arbiter", "preflight", "robot_state_adapter", "localization_adapter"} <= names
    for cmake in ROOT.rglob("CMakeLists.txt"):
        for path in re.findall(r"install\(PROGRAMS\s+([^\s)]+)", cmake.read_text()):
            assert (cmake.parent / path).is_file(), (cmake, path)


@pytest.mark.parametrize("relative", REMOVED_LAUNCHES)
def test_unused_compatibility_launch_is_removed(relative):
    assert not (ROOT / relative).exists()


def test_real_sensor_localization_and_nav2_launches_are_preserved():
    for relative in (
        "real/go2_onboard_bringup/launch/real.launch.py",
        "real/go2_real_bringup/launch/lidar.launch.py",
        "real/go2_real_bringup/launch/odometry.launch.py",
        "real/go2_real_bringup/launch/mapping.launch.py",
        "real/go2_real_bringup/launch/localization.launch.py",
        "real/go2_real_localization/launch/localization.launch.py",
        "common/go2_fastlio_localization/launch/fastlio.launch.py",
        "common/go2_fastlio_localization/launch/icp.launch.py",
        "common/go2_nav_bringup/launch/nav2.launch.py",
        "common/robocon_mission/launch/hurricane.launch.py",
    ):
        assert (ROOT / relative).is_file(), relative


def test_mission_launch_defaults_and_node_list_are_real_only():
    source = (ROOT / "common/robocon_mission/launch/mission.launch.py").read_text()
    tree = ast.parse(source)
    arguments, nodes = {}, []
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)):
        values = {k.arg: k.value for k in call.keywords}
        if call.func.id == "DeclareLaunchArgument":
            default = values["default_value"]
            arguments[ast.literal_eval(call.args[0])] = default.value if isinstance(default, ast.Constant) else ast.unparse(default)
        elif call.func.id == "Node":
            nodes.append((ast.literal_eval(values["package"]), ast.literal_eval(values["executable"])))
    assert nodes == [("robocon_mission", "mission_manager")]
    assert arguments["use_sim_time"] == arguments["autostart"] == arguments["simulation_mode"] == "false"
    assert "start_mock_servers" not in arguments
    assert {"route_file", "params_file", "accepted_pose_topic", "accepted_pose_type",
            "accepted_pose_body_frame", "accepted_pose_source_id", "startup_check_service", "mission_time_limit"} <= arguments.keys()
    assert "IfCondition" not in source and "PythonExpression" not in source


def test_no_real_launch_references_mock_nodes_or_missing_simulator_package():
    for path in ROOT.rglob("*.launch.py"):
        if path.relative_to(ROOT).as_posix() == "common/go2_mission_backends/launch/backend_simulation.launch.py":
            continue  # Explicit, guarded software safety-test entry, not an onboard include.
        source = path.read_text()
        assert "go2_sim_bringup" not in source, path
        assert "simulation_backend" not in source, path
        for name in REMOVED_CONSOLES:
            assert name not in source, (path, name)
    onboard = (ROOT / "real/go2_onboard_bringup/go2_onboard_bringup/launching.py").read_text()
    assert "backend_simulation.launch.py" not in onboard and "simulation_backend" not in onboard
    assert "__MOTOR_OUTPUT_DISABLED__" in onboard
    assert "/onboard/nav_cmd_internal" in onboard and "/onboard/nav_cmd_raw" in onboard


def test_mock_safety_fixtures_remain_source_only_and_all_launch_contracts_run():
    directory = ROOT / "common/robocon_mission/test/fixtures"
    for name in ("mock_motion_services.py", "mock_obstacle_skill.py"):
        assert (directory / name).is_file()
    setup = (ROOT / "common/robocon_mission/setup.py").read_text()
    assert 'find_packages(exclude=["test"])' in setup
    assert "test/fixtures" not in setup
    verify = (ROOT / "tools/verify_real.bash").read_text()
    assert "--ignore=common/robocon_mission/test/test_simulation_launch.py" not in verify
    assert "common/robocon_mission/test" in verify
    assert (ROOT / "common/go2_mission_backends/go2_mission_backends/simulation_backend.py").is_file()
    assert (ROOT / "common/go2_mission_backends/config/cyclonedds_sim.xml").is_file()
