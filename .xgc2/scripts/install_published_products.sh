#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DEBIAN_FRONTEND=noninteractive
"${SCRIPT_DIR}/setup_xgc2_apt_source.sh"
apt-get install -y --no-install-recommends \
  libxgc2-math-dev \
  libxgc2-state-machine-dev \
  xgc2-acados \
  ros-noetic-xgc2-estimator-hover-thrust-msgs \
  ros-noetic-xgc2-estimator-rigid-state-msgs \
  ros-noetic-xgc2-multirotor-reference-trajectory-msgs \
  ros-noetic-xgc2-px4-multirotor-controller-msgs \
  ros-noetic-xgc2-ros1-utils

require_version() {
  local package="$1" minimum="$2" installed
  installed="$(dpkg-query -W -f='${Version}' "${package}")"
  echo "Build dependency: ${package} ${installed} (minimum ${minimum})"
  dpkg --compare-versions "${installed}" ge "${minimum}" || {
    echo "Build dependency ${package} requires >= ${minimum}; installed ${installed}" >&2
    exit 1
  }
}
require_version libxgc2-math-dev '0.5.12-1~'
require_version "ros-${ROS_DISTRO:-noetic}-xgc2-multirotor-reference-trajectory-msgs" 1.4.0-1

# The generated header fixes the ROS1 wire MD5 in the compiled consumer.
message_header="/opt/ros/${ROS_DISTRO:-noetic}/include/multirotor_reference_trajectory_msgs/ReferenceStatus.h"
dpkg-query -S "${message_header}"
python3 - "${message_header}" '9e2c74fe168950b0e94149350644003c' <<'PY_MD5'
import pathlib
import re
import sys

header, expected = sys.argv[1:]
text = pathlib.Path(header).read_text()
trait = text.split("struct MD5Sum<", 1)[1].split("struct DataType<", 1)[0]
match = re.search(r'return "([0-9a-f]{32})";', trait)
actual = match.group(1) if match else None
print(f"Build message header: {header} MD5={actual} (required {expected})")
if actual != expected:
    raise SystemExit("Installed message header does not match the required ROS1 contract")
PY_MD5
