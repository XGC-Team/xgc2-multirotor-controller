#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
work_dir="${RUNNER_TEMP:-/tmp}/xgc2-multirotor-controller-compliance"
install_root="${RUNNER_TEMP:-/tmp}/xgc2-multirotor-controller-install-root"

rm -rf "$work_dir" "$install_root"
mkdir -p "$work_dir/src/xgc2-multirotor-controller"
rsync -a --delete "$REPO_ROOT/" "$work_dir/src/xgc2-multirotor-controller/"

cd "$work_dir"
set +u
source /opt/ros/noetic/setup.bash
set -u
parallel_jobs="$(nproc)"
catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" \
  run_tests_multirotor_reference_trajectory \
  run_tests_px4_multirotor_controller
catkin_test_results
DESTDIR="$install_root" catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
  -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
  -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
test "$(rospack find multirotor_reference_trajectory)" = "$work_dir/src/xgc2-multirotor-controller/multirotor_reference_trajectory"
test "$(rospack find px4_multirotor_controller)" = "$work_dir/src/xgc2-multirotor-controller/px4_multirotor_controller"
roslaunch --files multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch >/tmp/xgc2-multirotor-reference-files.txt
roslaunch --files px4_multirotor_controller uav_nmpc_controller.launch >/tmp/xgc2-px4-controller-files.txt
