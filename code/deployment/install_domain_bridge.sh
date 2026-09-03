#!/usr/bin/env bash
# Install the ROS 2 domain_bridge package used by the cross-domain demo.
# Works on Ubuntu amd64 (development host) and arm64 (Raspberry Pi running
# Ubuntu + ROS 2). The package is selected from ROS_DISTRO.

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: install_domain_bridge.sh [OPTIONS]

Install the ros-<distro>-domain-bridge package with apt.

Options:
  --ros-distro DISTRO  ROS 2 distribution (default: ROS_DISTRO or humble)
  --check              Only verify that domain_bridge is already installed
  -h, --help           Show this help
EOF
}

ros_distro="${ROS_DISTRO:-}"
check_only=0

while (($# > 0)); do
  case "$1" in
    --ros-distro)
      (($# >= 2)) || { echo "--ros-distro requires a value" >&2; exit 2; }
      ros_distro="$2"
      shift 2
      ;;
    --check)
      check_only=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$ros_distro" ]]; then
  # Prefer an already-installed ROS setup when ROS_DISTRO was not exported.
  for setup_file in /opt/ros/*/setup.bash; do
    if [[ -f "$setup_file" ]]; then
      ros_distro="${setup_file#/opt/ros/}"
      ros_distro="${ros_distro%/setup.bash}"
      break
    fi
  done
fi
ros_distro="${ros_distro:-humble}"

if [[ ! "$ros_distro" =~ ^[a-z0-9][a-z0-9_-]*$ ]]; then
  echo "Invalid ROS distribution name: $ros_distro" >&2
  exit 2
fi

package="ros-${ros_distro}-domain-bridge"
ros_prefix="/opt/ros/${ros_distro}"

is_installed() {
  [[ -x "${ros_prefix}/lib/domain_bridge/domain_bridge" ]] && return 0
  command -v ros2 >/dev/null 2>&1 || return 1
  [[ -f "${ros_prefix}/setup.bash" ]] || return 1
  source "${ros_prefix}/setup.bash"
  local detected_prefix
  detected_prefix="$(ros2 pkg prefix domain_bridge 2>/dev/null || true)"
  [[ "$detected_prefix" == "$ros_prefix" ]]
}

if is_installed; then
  echo "domain_bridge is installed for ROS ${ros_distro} (${ros_prefix})."
  exit 0
fi

if ((check_only)); then
  echo "domain_bridge is not installed for ROS ${ros_distro}." >&2
  exit 1
fi

command -v apt-get >/dev/null 2>&1 || {
  echo "apt-get is required (use Ubuntu 22.04 on the host or Raspberry Pi)." >&2
  exit 1
}

if (( $(id -u) == 0 )); then
  apt=(apt-get)
else
  command -v sudo >/dev/null 2>&1 || {
    echo "Run as root or install sudo before running this script." >&2
    exit 1
  }
  apt=(sudo apt-get)
fi

echo "Installing ${package} for $(dpkg --print-architecture 2>/dev/null || echo unknown)..."
"${apt[@]}" update
if ! apt-cache show "$package" >/dev/null 2>&1; then
  cat >&2 <<EOF
Package ${package} is not available from the configured apt repositories.
For a Raspberry Pi, use 64-bit Ubuntu 22.04 with the ROS 2 apt repository,
then run this script again (or set --ros-distro to the installed ROS version).
EOF
  exit 1
fi
DEBIAN_FRONTEND=noninteractive "${apt[@]}" install -y "$package"

if ! is_installed; then
  echo "Installation finished but domain_bridge could not be verified." >&2
  echo "Source /opt/ros/${ros_distro}/setup.bash and run: ros2 pkg prefix domain_bridge" >&2
  exit 1
fi

echo "Installed ${package}; cross-domain forwarding can now be enabled."
