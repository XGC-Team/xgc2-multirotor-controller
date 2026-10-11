#!/usr/bin/env bash
set -euo pipefail

# The ROS-free boundary of the product: the two cores, the NMPC runtime and the two modules are
# built without ROS include directories and must not link a ROS library either. The ROS nodes
# (input producers, output consumers, node mains) are the only places that may depend on ROS.
ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      PREFIX="$2/opt/ros/${ROS_DISTRO}"
      shift 2
      ;;
    --prefix)
      PREFIX="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

check_ros_free() {
  local lib_path="$1"
  test -f "${lib_path}"
  if readelf -d "${lib_path}" | grep -E 'NEEDED.*(libros|libroscpp|librosconsole|librostime|libcpp_common|libxmlrpcpp|libtf|libmavros|libgeometry_msgs|libsensor_msgs|libstd_msgs)'; then
    echo "must not link against ROS: ${lib_path}" >&2
    exit 1
  fi
}

check_ros_free "${PREFIX}/lib/libmultirotor_reference_trajectory_core.so"
check_ros_free "${PREFIX}/lib/libpx4_multirotor_controller_core.so"
check_ros_free "${PREFIX}/lib/libpx4_multirotor_controller_uav_nmpc_runtime.so"
check_ros_free "${PREFIX}/lib/libpx4_multirotor_reference_module.so"
check_ros_free "${PREFIX}/lib/libpx4_multirotor_controller_module.so"

echo "Core library ROS-boundary check passed"
