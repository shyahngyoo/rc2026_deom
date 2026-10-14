#!/usr/bin/env bash
# Run local ROS tests/Mock programs without WSL mirrored localhost forwarding.
# The temporary route affects UDP to 127.0.0.1, domain 97 ports only.
set -euo pipefail
if (( $# == 0 )); then
  echo "Usage: $0 COMMAND [ARG ...] (local tests/Mock only)" >&2
  exit 2
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
profile="${script_dir}/../config/fastdds_local_test.xml"
if [[ ! -f "$profile" ]]; then
  echo "Missing local DDS profile: $profile" >&2
  exit 2
fi
# Serialize users of this entry point so cleanup cannot interrupt another run.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/robocon_local_dds_${UID}.lock"
flock 9
root_ip() {
  if (( EUID == 0 )); then
    ip "$@"
  elif [[ -n "${WSL_DISTRO_NAME:-}" ]] && command -v wsl.exe >/dev/null; then
    wsl.exe --distribution "$WSL_DISTRO_NAME" --user root --exec ip "$@"
  else
    sudo -n ip "$@"
  fi
}
route_added=false
cleanup() {
  if [[ "$route_added" == true ]]; then
    root_ip rule del pref 0 to 127.0.0.1/32 ipproto udp \
      dport 31650-31899 lookup local || {
        echo "Failed to remove temporary local DDS route; remove the rule manually." >&2
        return 1
      }
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Domain 97: 7400 + 250 * 97 = 31650. Keep other localhost traffic unchanged.
if ! ip route get 127.0.0.1 ipproto udp dport 31660 | grep -q 'dev lo '; then
  root_ip rule add pref 0 to 127.0.0.1/32 ipproto udp \
    dport 31650-31899 lookup local
  route_added=true
fi
if ! ip route get 127.0.0.1 ipproto udp dport 31660 | grep -q 'dev lo '; then
  echo "Local DDS route is still not using lo; refusing to run." >&2
  exit 1
fi
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"
# Configured acceptance runs must fail if IPC is broken, never silently skip.
export ROBOCON_REQUIRE_ROS_IPC=1
echo "Local DDS: domain=97, UDP/unicast=127.0.0.1, route=lo" >&2
"$@"
