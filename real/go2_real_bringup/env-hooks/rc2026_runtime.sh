# Defaults for the onboard runtime; explicit non-empty values take precedence.
# Humble itself exports ROS_LOCALHOST_ONLY=0, so use a project-specific override.
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${RC2026_ROS_LOCALHOST_ONLY:-1}"

# The declared dependency is sourced first. Resolve its installed config without
# launching Python/ros2, and support both isolated and merged install layouts.
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  _rc2026_prefixes="${AMENT_PREFIX_PATH:-}"
  while [ -n "$_rc2026_prefixes" ]; do
    _rc2026_prefix="${_rc2026_prefixes%%:*}"
    _rc2026_config="$_rc2026_prefix/share/go2_mission_backends/config/cyclonedds_sim.xml"
    if [ -f "$_rc2026_config" ]; then
      export CYCLONEDDS_URI="file://$_rc2026_config"
      break
    fi
    case "$_rc2026_prefixes" in
      *:*) _rc2026_prefixes="${_rc2026_prefixes#*:}" ;;
      *) break ;;
    esac
  done
  if [ -z "${CYCLONEDDS_URI:-}" ]; then
    printf '%s\n' 'RC2026: CycloneDDS config missing; build and source go2_mission_backends.' >&2
  fi
  unset _rc2026_prefixes _rc2026_prefix _rc2026_config
fi
