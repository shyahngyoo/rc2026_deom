#!/usr/bin/env bash
# Software-only tests; never starts a radar or actuator driver.
set -eo pipefail
ulimit -c 0
verify_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
verify_ws="${RC2026_WS:-}"
verify_report="${RC2026_REPORT_DIR:-}"
verify_domain="${RC2026_TEST_DOMAIN:-154}"
verify_build=1
verify_smoke=1
verify_wall=0
usage() {
  cat <<'USAGE'
Usage: tools/verify_real.bash [--workspace PATH] [--report-dir PATH]
       [--domain 1..198] [--skip-build] [--skip-smoke] [--include-wall-clock]
Run after install_real.bash. WORKSPACE/src/rc2026_deom determines WORKSPACE;
otherwise pass --workspace or set RC2026_WS. Uses only the new workspace.
Builds test runtimes, tests synthetic native GICP and backend Mission, and
runs independent /backend_sim software launch. No lidar or motors start.
Default complete-flow test uses controlled /clock. Host realtime jumps may
invalidate the additional wall-clock positive case; opt in with its flag.
USAGE
}
while (($#)); do
  case "$1" in
    --workspace|--report-dir|--domain)
      (($# >= 2)) || { echo "Missing value for $1" >&2; exit 2; }
      case "$1" in
        --workspace) verify_ws="$2";;
        --report-dir) verify_report="$2";;
        --domain) verify_domain="$2";;
      esac
      shift 2;;
    --skip-build) verify_build=0; shift;;
    --skip-smoke) verify_smoke=0; shift;;
    --include-wall-clock) verify_wall=1; shift;;
    --help|-h) usage; exit 0;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done
[[ "$verify_domain" =~ ^[0-9]+$ ]] &&
  ((10#$verify_domain >= 1 && 10#$verify_domain <= 198)) ||
  { echo "domain must be 1..198" >&2; exit 2; }
verify_domain=$((10#$verify_domain))
if [[ -z "$verify_ws" ]]; then
  verify_parent=$(dirname -- "$verify_repo")
  if [[ "$(basename -- "$verify_parent")" == src ]]; then
    verify_ws=$(dirname -- "$verify_parent")
  else
    echo "Checkout outside WORKSPACE/src; pass --workspace after installing dependencies." >&2
    exit 2
  fi
fi
verify_ws=$(realpath -m -- "$verify_ws")
[[ -d "$verify_ws/src" ]] || { echo "Workspace src missing: $verify_ws/src" >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS 2 Humble is required." >&2; exit 2; }
verify_report="${verify_report:-$verify_ws/log/verification/$(date -u +%Y%m%dT%H%M%SZ)}"
verify_report=$(realpath -m -- "$verify_report")
mkdir -p -- "$verify_report"
export PYTHONNOUSERSITE=1
export MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
if [[ -f "$verify_ws/install/local_setup.bash" ]]; then
  source "$verify_ws/install/local_setup.bash"
fi
export ROS_DOMAIN_ID="$verify_domain" ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RC2026_CLI_DOMAIN_ID="$((verify_domain+1))"
export RC2026_REPORT_DIR="$verify_report/backend_cases" ROBOCON_REQUIRE_ROS_IPC=1
export CYCLONEDDS_URI="file://$verify_repo/common/go2_mission_backends/config/cyclonedds_sim.xml"
verify_packages=(go2_obstacle_interfaces go2_nav_bringup robocon_mission go2_mission_backends go2_real_localization go2_onboard_bringup)
if ((verify_build)); then
  cd -- "$verify_ws"
  colcon build --executor sequential --base-paths "$verify_repo" --packages-select "${verify_packages[@]}" \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    2>&1 | tee "$verify_report/build.log"
fi
[[ -f "$verify_ws/install/local_setup.bash" ]] ||
  { echo "Workspace install missing; run install_real.bash first." >&2; exit 2; }
source "$verify_ws/install/local_setup.bash"
export RC2026_VERIFY_INSTALL="$verify_ws/install"
/usr/bin/python3 - <<'PY' 2>&1 | tee "$verify_report/environment.log"
import json, os
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import rclpy, numpy, scipy, yaml
root=Path(os.environ["RC2026_VERIFY_INSTALL"]).resolve()
names=("go2_obstacle_interfaces","go2_nav_bringup","robocon_mission",
       "go2_mission_backends","go2_real_localization","go2_onboard_bringup","fast_gicp")
prefixes={name:Path(get_package_prefix(name)).resolve() for name in names}
for name,prefix in prefixes.items():
    assert prefix==root or root in prefix.parents, \
        f"{name} resolves outside this workspace: {prefix}; expected {root}"
native=prefixes["go2_real_localization"]/"lib"/"go2_real_localization"
assert (native/"gicp_localizer").is_file()
assert (native/"synthetic_gicp_inputs").is_file(), "Rebuild with BUILD_TESTING=ON."
assert Path(os.environ["CYCLONEDDS_URI"].removeprefix("file://")).is_file()
print(json.dumps(dict(workspace_install=str(root),packages={k:str(v) for k,v in prefixes.items()},
                     domain=os.environ["ROS_DOMAIN_ID"],simulation_only=True,
                     numpy=dict(version=numpy.__version__,file=numpy.__file__),
                     scipy=dict(version=scipy.__version__,file=scipy.__file__),
                     python_no_user_site=os.environ.get("PYTHONNOUSERSITE")),indent=2))
PY
cd -- "$verify_repo"
verify_extra=()
if ((!verify_wall)); then
  verify_extra+=(--deselect='common/go2_mission_backends/test/test_backend_mission.py::test_mission_two_distinct_obstacles_and_return[wall_clock]')
fi
/usr/bin/python3 -m pytest -q common/go2_mission_backends/test common/robocon_mission/test \
  real/go2_onboard_bringup/test/test_profile.py tools/tests \
  "${verify_extra[@]}" --junitxml="$verify_report/tests.xml" 2>&1 | tee "$verify_report/tests.log"
verify_nav_build="$verify_ws/build/go2_nav_bringup"
[[ -f "$verify_nav_build/CTestTestfile.cmake" ]] ||
  { echo "Nav2 contract tests missing; rebuild go2_nav_bringup with BUILD_TESTING=ON." >&2; exit 2; }
ctest --test-dir "$verify_nav_build" --show-only=json-v1 > "$verify_report/nav_contract_inventory.json"
/usr/bin/python3 - "$verify_report/nav_contract_inventory.json" <<'PY'
import json,sys
expected={"precision_pose_contract","recoverable_rotate_to_goal_contract",
          "pause_replanning_contract","navigate_to_pose_client","map_activation","bt_plugin_environment"}
actual={t["name"] for t in json.load(open(sys.argv[1]))["tests"]}
assert expected<=actual, f"Nav2 contract tests missing: {sorted(expected-actual)}"
PY
ctest --test-dir "$verify_nav_build" --output-on-failure 2>&1 | tee "$verify_report/nav_contracts.log"
if ((verify_smoke)); then
  /usr/bin/python3 tools/smoke_mission_backends.py --domain "$((verify_domain+2))" \
    --output-dir "$verify_report/launch_smoke" 2>&1 | tee "$verify_report/launch_smoke.log"
fi
printf 'Software verification passed. Reports: %s\n' "$verify_report"

