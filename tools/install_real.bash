#!/usr/bin/env bash
# Ubuntu 22.04/Humble onboard runtime only. No motor commands or simulation.
set -Eeuo pipefail
usage() {
  cat <<'HELP'
Usage: bash tools/install_real.bash [--workspace PATH] [--jobs 1..4] [--install-ros] [--dry-run]
  --workspace PATH  Workspace (default: ~/rc2026_ws); preserve unrelated existing directories.
  --jobs N          Per-package compilation concurrency (default: 2); package executor sequential.
  --install-ros     If Humble is missing, install ROS base from the official ROS apt source.
  --dry-run         Validate local inputs and print commands; no sudo/network/filesystem mutation.
  --help            Show this help.
Existing dependencies must be clean pinned clones or unchanged installer-managed patched clones.
An unknown modification, revision or workspace link is rejected, never reset/cleaned/deleted.
HELP
}
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo_dir=$(cd -- "$script_dir/.." && pwd -P)
workspace_arg="${HOME}/rc2026_ws"; jobs=2; install_ros=false; dry_run=false
while (($#)); do
  case "$1" in
    --workspace) (($#>=2)) || fail '--workspace needs a path'; workspace_arg=$2; shift 2 ;;
    --jobs) (($#>=2)) || fail '--jobs needs a value'; jobs=$2; shift 2 ;;
    --install-ros) install_ros=true; shift ;;
    --dry-run) dry_run=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) fail "unknown option: $1" ;;
  esac
done
[[ "$jobs" =~ ^[1-4]$ ]] || fail '--jobs must be an integer from 1 through 4'
[[ -n "$workspace_arg" ]] || fail 'workspace path is empty'
workspace=$(realpath -m -- "$workspace_arg")
[[ "$workspace" != / && "$workspace" != /opt && "$workspace" != /opt/ros ]] || fail 'choose a dedicated workspace directory'
[[ -f /etc/os-release ]] || fail 'Ubuntu /etc/os-release is required'
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || fail 'only Ubuntu 22.04 is supported'
architecture=$(dpkg --print-architecture)
[[ "$architecture" == amd64 || "$architecture" == arm64 ]] || fail "unsupported architecture: $architecture"
ros_setup=/opt/ros/humble/setup.bash
[[ -f "$ros_setup" || "$install_ros" == true ]] || fail 'ROS Humble is absent; rerun with --install-ros to install official ROS base'
[[ -f "$repo_dir/dependencies.repos" && -f "$repo_dir/dependencies.lock.yaml" ]] || fail 'source dependency manifests are missing'
runtime_rel=(common/go2_obstacle_interfaces common/go2_fastlio_localization common/go2_nav_bringup
  common/robocon_mission common/go2_mission_backends real/go2_real_localization
  real/go2_real_bringup real/go2_onboard_bringup)
runtime_paths=()
for relative in "${runtime_rel[@]}"; do
  [[ -f "$repo_dir/$relative/package.xml" ]] || fail "runtime package missing: $relative/package.xml"
  runtime_paths+=("$repo_dir/$relative")
done
/usr/bin/python3 - "$repo_dir" <<'PY'
import hashlib,pathlib,re,sys
root=pathlib.Path(sys.argv[1]);lock=(root/'dependencies.lock.yaml').read_text()
expected={'livox_driver2_monotonic_clock.patch','fast_lio_handheld.patch','fast_lio_timestamp_guard.patch'}
hashes=dict(re.findall(r'^  ([a-z0-9_]+\.patch): ([0-9a-f]{64})$',lock,re.M))
if set(hashes)!=expected:raise SystemExit('missing patch locks')
for name,digest in hashes.items():
    p=root/'patches'/name
    if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise SystemExit('patch checksum mismatch: '+name)
PY
# Parse this small vcs subset without depending on PyYAML before apt.
dep_text=$(/usr/bin/python3 - "$repo_dir/dependencies.repos" <<'PY'
import re,sys
expected={'Livox-SDK2','livox_ros_driver2','FAST_LIO','fast_gicp'}
records={};current=None
for raw in open(sys.argv[1],encoding='utf-8'):
    line=raw.rstrip('\n')
    if not line.strip() or line.lstrip().startswith('#') or line=='repositories:':continue
    match=re.fullmatch(r'  ([A-Za-z0-9_-]+):',line)
    if match:
        current=match[1]
        if current in records:raise SystemExit('duplicate repository')
        records[current]={};continue
    match=re.fullmatch(r'    (type|url|version): (.+)',line)
    if not match or current is None or match[1] in records[current]:raise SystemExit('invalid dependency manifest')
    records[current][match[1]]=match[2]
if set(records)!=expected:raise SystemExit('unexpected runtime dependency set')
for name,data in records.items():
    if set(data)!={'type','url','version'} or data['type']!='git' or not data['url'].startswith('https://github.com/') or not re.fullmatch('[0-9a-f]{40}',data['version']):raise SystemExit('dependency must have HTTPS URL and exact SHA')
    print(name+'\t'+data['url']+'\t'+data['version'])
PY
) || fail 'invalid pinned source manifest'
mapfile -t dep_specs <<<"$dep_text"
recipe_hash=$(cat -- "$script_dir/install_real.bash" "$repo_dir/dependencies.repos" \
  "$repo_dir/patches/livox_driver2_monotonic_clock.patch" "$repo_dir/patches/fast_lio_handheld.patch" \
  "$repo_dir/patches/fast_lio_timestamp_guard.patch" | sha256sum | cut -d' ' -f1)
link="$workspace/src/rc2026_deom"
if [[ -e "$link" || -L "$link" ]]; then
  [[ "$(realpath -m -- "$link")" == "$repo_dir" ]] || fail "workspace path belongs to another checkout: $link"
fi
if ((EUID==0)); then root_cmd=(); else root_cmd=(sudo); fi
run() { printf '+ '; printf '%q ' "$@"; printf '\n'; if ! "$dry_run"; then "$@"; fi; }
trap 'printf "ERROR: install failed at line %s; existing user files were not reset.\n" "$LINENO" >&2' ERR
validate_checkout() {
  local name=$1 revision=$2 target="$workspace/src/$1" stamp="$workspace/.rc2026_install/$1.json"
  [[ ! -e "$target" && ! -L "$target" ]] && return
  [[ -d "$target" && -e "$target/.git" ]] || fail "existing dependency is not a Git checkout: $target"
  [[ "$(git -C "$target" rev-parse --show-toplevel)" == "$(realpath -- "$target")" ]] || fail "dependency is not its own checkout: $target"
  [[ "$(git -C "$target" rev-parse HEAD)" == "$revision" ]] || fail "dependency revision mismatch; preserve and resolve manually: $target"
  /usr/bin/python3 - "$target" "$stamp" "$revision" "$recipe_hash" <<'PY'
import hashlib,json,pathlib,subprocess,sys
root,stamp,revision,recipe=sys.argv[1:]
def git(*args):return subprocess.check_output(['git','-C',root,*args])
diff=hashlib.sha256(git('diff','--binary','HEAD')).hexdigest();untracked={}
for raw in git('ls-files','--others','--exclude-standard','-z').split(b'\0'):
    if not raw:continue
    name=raw.decode();p=pathlib.Path(root)/name
    if not p.is_file() or p.is_symlink():raise SystemExit('unknown untracked path: '+name)
    untracked[name]=hashlib.sha256(p.read_bytes()).hexdigest()
if pathlib.Path(stamp).exists():
    data=json.loads(pathlib.Path(stamp).read_text())
    if data!={'revision':revision,'recipe':recipe,'diff':diff,'untracked':untracked}:
        raise SystemExit('dependency modified or install recipe changed; preserve and resolve manually: '+root)
elif git('status','--porcelain','--untracked-files=all').strip():
    raise SystemExit('unknown dependency changes; preserve and resolve manually: '+root)
PY
  if [[ "$name" == FAST_LIO ]]; then
    local substatus
    substatus=$(git -C "$target" submodule status --recursive)
    [[ ! "$substatus" =~ (^|$'\n')[+U] ]] || fail 'FAST_LIO submodule revision differs; preserve and resolve manually'
    git -C "$target" submodule foreach --recursive 'test -z "$(git status --porcelain --untracked-files=all)"' || fail 'FAST_LIO submodule has unknown modifications'
  fi
}
for spec in "${dep_specs[@]}"; do IFS=$'\t' read -r name url revision <<<"$spec"; validate_checkout "$name" "$revision"; done
printf 'Runtime-only installer: Ubuntu %s, %s; repo=%s; workspace=%s; jobs=%s\n' "$VERSION_ID" "$architecture" "$repo_dir" "$workspace" "$jobs"
if [[ ! -f "$ros_setup" ]]; then
  run "${root_cmd[@]}" apt-get update
  run "${root_cmd[@]}" apt-get install -y curl ca-certificates software-properties-common locales
  run "${root_cmd[@]}" add-apt-repository -y universe
  run "${root_cmd[@]}" locale-gen en_US.UTF-8
  run "${root_cmd[@]}" apt-get install -y --only-upgrade systemd udev
  deb="$workspace/.rc2026_install/ros2-apt-source_1.3.0.jammy_all.deb"
  run mkdir -p -- "$workspace/.rc2026_install"
  run curl --fail --location --retry 3 --connect-timeout 15 --max-time 180 \
    -o "$deb" https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb
  run "${root_cmd[@]}" dpkg -i "$deb"
  run "${root_cmd[@]}" apt-get update
  run "${root_cmd[@]}" apt-get install -y ros-humble-ros-base
fi
run "${root_cmd[@]}" apt-get update
run "${root_cmd[@]}" apt-get install -y git build-essential cmake pkg-config python3-colcon-common-extensions \
  python3-rosdep python3-numpy python3-scipy python3-yaml ros-humble-rmw-cyclonedds-cpp
run mkdir -p -- "$workspace/src" "$workspace/.rc2026_install"
if [[ ! -e "$link" && ! -L "$link" ]]; then run ln -s -- "$repo_dir" "$link"; fi
patch_once() {
  local target=$1 patch_file=$2
  if "$dry_run"; then run git -C "$target" apply "$patch_file"; return; fi
  if git -C "$target" apply --check "$patch_file" 2>/dev/null; then git -C "$target" apply "$patch_file"
  elif git -C "$target" apply --reverse --check "$patch_file" 2>/dev/null; then printf 'Patch already applied: %s\n' "$patch_file"
  else fail "patch does not match pinned source: $patch_file"; fi
}
patch_driver_headers() {
  local target=$1
  if "$dry_run"; then printf '+ patch_driver_headers %q (Humble ROSIDL headers; package_ROS2.xml -> package.xml)\n' "$target"; return; fi
  /usr/bin/python3 - "$target" <<'PY'
import pathlib,sys
root=pathlib.Path(sys.argv[1]);p=root/'CMakeLists.txt';data=p.read_text()
data='\n'.join(line for line in data.split('\n') if '${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}   # for custom msgs' not in line and '${LIVOX_INTERFACE_TARGET}   # for custom msgs' not in line)
needle='target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")'
if needle not in data:raise SystemExit('unexpected pinned Livox CMake ROSIDL block')
fix='\n    add_dependencies(${PROJECT_NAME} ${LIVOX_INTERFACES})\n    target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")'
if 'CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp' not in data:data=data.replace(needle,needle+fix,1)
manifest=root/'package_ROS2.xml'
if not manifest.is_file():raise SystemExit('Livox package_ROS2.xml missing')
content=manifest.read_bytes();p.write_text(data);(root/'package.xml').write_bytes(content)
PY
}
record_checkout() {
  local name=$1 revision=$2 target="$workspace/src/$1"
  if "$dry_run"; then printf '+ record pinned dependency fingerprint %q\n' "$target"; return; fi
  /usr/bin/python3 - "$target" "$workspace/.rc2026_install/$name.json" "$revision" "$recipe_hash" "$name" <<'PY'
import hashlib,json,pathlib,subprocess,sys
root,stamp,revision,recipe,name=sys.argv[1:]
def git(*args):return subprocess.check_output(['git','-C',root,*args])
untracked={}
for raw in git('ls-files','--others','--exclude-standard','-z').split(b'\0'):
    if not raw:continue
    entry=raw.decode();p=pathlib.Path(root)/entry
    if name!='livox_ros_driver2' or entry!='package.xml' or not p.is_file() or p.is_symlink():raise SystemExit('unexpected generated source file: '+entry)
    untracked[entry]=hashlib.sha256(p.read_bytes()).hexdigest()
data={'revision':revision,'recipe':recipe,'diff':hashlib.sha256(git('diff','--binary','HEAD')).hexdigest(),'untracked':untracked}
p=pathlib.Path(stamp);tmp=p.with_suffix('.json.tmp');tmp.write_text(json.dumps(data,sort_keys=True)+'\n');tmp.replace(p)
PY
}
for spec in "${dep_specs[@]}"; do
  IFS=$'\t' read -r name url revision <<<"$spec"; target="$workspace/src/$name"
  if [[ ! -e "$target" ]]; then
    # Stage new clones; never delete/reset an existing source directory.
    stage="$workspace/.rc2026_install/${name}.clone.$$"
    [[ ! -e "$stage" ]] || fail "staging path already exists: $stage"
    run git -c http.lowSpeedLimit=1000 -c http.lowSpeedTime=30 clone --no-checkout "$url" "$stage"
    run git -C "$stage" checkout --detach "$revision"
    run mv -- "$stage" "$target"
  fi
  record_checkout "$name" "$revision"
  if [[ "$name" == FAST_LIO ]]; then run git -C "$target" submodule update --init --recursive; fi
  case "$name" in
    livox_ros_driver2)
      patch_once "$target" "$repo_dir/patches/livox_driver2_monotonic_clock.patch"; record_checkout "$name" "$revision"
      patch_driver_headers "$target"; record_checkout "$name" "$revision" ;;
    FAST_LIO)
      patch_once "$target" "$repo_dir/patches/fast_lio_handheld.patch"; record_checkout "$name" "$revision"
      patch_once "$target" "$repo_dir/patches/fast_lio_timestamp_guard.patch"; record_checkout "$name" "$revision" ;;
  esac
done
# Clear inherited workspace overlays in this subprocess, never in the caller.
if "$dry_run"; then printf '+ source %q (without inherited workspace overlays)\n' "$ros_setup"
else
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
  set +u
  # shellcheck disable=SC1090
  source "$ros_setup"
  set -u
fi
export LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONNOUSERSITE=1
base_paths=("${runtime_paths[@]}" "$workspace/src/livox_ros_driver2" "$workspace/src/FAST_LIO" "$workspace/src/fast_gicp")
if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then run "${root_cmd[@]}" rosdep init; fi
run rosdep update --rosdistro humble
run rosdep install --from-paths "${base_paths[@]}" --ignore-src --rosdistro humble \
  -t build -t buildtool -t build_export -t buildtool_export -t exec -r -y
sdk_prefix="$workspace/install/livox-sdk2"
run cmake -S "$workspace/src/Livox-SDK2" -B "$workspace/build/livox_sdk2_bootstrap" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$sdk_prefix"
run cmake --build "$workspace/build/livox_sdk2_bootstrap" --parallel "$jobs"
run cmake --install "$workspace/build/livox_sdk2_bootstrap"
if ! "$dry_run"; then
  [[ -f "$sdk_prefix/lib/liblivox_lidar_sdk_shared.so" && -f "$sdk_prefix/include/livox_lidar_api.h" ]] || fail 'Livox SDK did not install expected library/header'
fi
export MAKEFLAGS="-j$jobs" CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
run colcon --log-base "$workspace/log" build --base-paths "${base_paths[@]}" \
  --build-base "$workspace/build" --install-base "$workspace/install" --symlink-install --executor sequential \
  --packages-up-to go2_onboard_bringup go2_nav_bringup go2_mission_backends \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_VGICP_CUDA=OFF \
  -DBUILD_apps=OFF -DBUILD_test=OFF -DBUILD_PYTHON_BINDINGS=OFF -DDISTRO_ROS=humble \
  "-DLIVOX_LIDAR_SDK_LIBRARY=$sdk_prefix/lib/liblivox_lidar_sdk_shared.so" \
  "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$sdk_prefix/include" "-DCMAKE_INSTALL_RPATH=$sdk_prefix/lib"
printf '\nInstalled runtime only. Next steps (no movement is enabled):\n'
printf '  source /opt/ros/humble/setup.bash\n  source %q\n' "$workspace/install/setup.bash"
printf '  ros2 run go2_onboard_bringup configure_robot --help\n'
printf 'Configure real MID-360 IP, measured IMU installation, map and fixed start pose.\n'
printf 'No Gazebo/CHAMP/CUDA or motor driver is installed; no example map is configured and no fake state/test skill is started.\n'
