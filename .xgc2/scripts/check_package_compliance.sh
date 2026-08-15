#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

bash -n .xgc2/scripts/*.sh

nested_git="$(
  find . \
    -path ./.git -prune -o \
    -path './*/build' -prune -o \
    -path './*/devel' -prune -o \
    -path './*/install' -prune -o \
    -path ./.work -prune -o \
    -path ./debs -prune -o \
    -name .git -print
)"
if [[ -n "${nested_git}" ]]; then
  echo "Nested .git directory found." >&2
  echo "${nested_git}" >&2
  exit 1
fi

if git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.work|debs)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.work|debs)(/|$)' >&2
  exit 1
fi

required_files=(
  .clang-format
  .clang-tidy
  .github/workflows/ci.yml
  .github/workflows/release.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/check_core_libraries.sh
  .xgc2/scripts/check_cpp_quality.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/check_version_bump.sh
  .xgc2/scripts/install_published_products.sh
  .xgc2/scripts/package_debs.sh
  .xgc2/scripts/run_source_tests.sh
  .xgc2/scripts/setup_xgc2_apt_source.sh
  px4_multirotor_controller/CMakeLists.txt
  px4_multirotor_controller/package.xml
  px4_multirotor_controller/launch/uav_nmpc_controller.launch
  px4_multirotor_controller/config/uav_nmpc.yaml
  multirotor_reference_trajectory/CMakeLists.txt
  multirotor_reference_trajectory/package.xml
  multirotor_reference_trajectory/include/multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h
  multirotor_reference_trajectory/launch/uav_multirotor_reference_trajectory.launch
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

grep -q "id: xgc2-multirotor-controller" .xgc2/product.yml
grep -Eq '^version: [0-9]+\.[0-9]+\.[0-9]+-[0-9]+$' .xgc2/product.yml
grep -q "<name>px4_multirotor_controller</name>" px4_multirotor_controller/package.xml
grep -q "<name>multirotor_reference_trajectory</name>" multirotor_reference_trajectory/package.xml
grep -q "run_tests_multirotor_reference_trajectory" .xgc2/scripts/run_source_tests.sh
grep -q "run_tests_px4_multirotor_controller" .xgc2/scripts/run_source_tests.sh
grep -q "install_published_products.sh" .github/workflows/ci.yml
grep -q "run_source_tests.sh" .github/workflows/ci.yml
grep -q "expected_version" .github/workflows/release.yml
grep -q "expected_source_sha" .github/workflows/release.yml
grep -q "PACKAGE=\"ros-\${ROS_DISTRO}-xgc2-multirotor-controller\"" .xgc2/scripts/package_debs.sh
grep -q "px4_multirotor_controller" .xgc2/scripts/package_debs.sh
grep -q "multirotor_reference_trajectory" .xgc2/scripts/package_debs.sh
grep -q "libxgc2-state-machine-dev (>= 0.1.3-4~focal)" .xgc2/scripts/package_debs.sh
grep -q "libxgc2-math-dev (>= 0.5.6-6~focal)" .xgc2/scripts/package_debs.sh
grep -q "xgc2-acados (>= 0.1.0-10~focal)" .xgc2/product.yml
grep -q "xgc2-acados (>= 0.1.0-10~focal)" .xgc2/scripts/package_debs.sh
grep -q "ros-\${ROS_DISTRO}-xgc2-estimator-hover-thrust-msgs (>= 1.2.0-3)" .xgc2/scripts/package_debs.sh
grep -q "ros-\${ROS_DISTRO}-xgc2-estimator-rigid-state-msgs (>= 1.2.0-3)" .xgc2/scripts/package_debs.sh
grep -q "ros-\${ROS_DISTRO}-xgc2-multirotor-reference-trajectory-msgs (>= 1.2.0-3)" .xgc2/scripts/package_debs.sh
grep -q "ros-\${ROS_DISTRO}-xgc2-px4-multirotor-controller-msgs (>= 1.2.0-3)" .xgc2/scripts/package_debs.sh

if grep -R --exclude='check_package_compliance.sh' \
  -E "ros-noetic-xgc2-estimator-hover-thrust([[:space:]]|$|[),])" \
  .github .xgc2 README.md px4_multirotor_controller multirotor_reference_trajectory >/dev/null; then
  echo "Estimator implementation dependency found in multirotor controller package." >&2
  exit 1
fi

if grep -R --exclude='check_package_compliance.sh' "ros-noetic-xgc2-reference" \
  .github .xgc2 README.md px4_multirotor_controller multirotor_reference_trajectory >/dev/null; then
  echo "Deprecated ros-noetic-xgc2-reference dependency found." >&2
  exit 1
fi

if grep -R --exclude='check_package_compliance.sh' \
  -E "UavFlatTrajectory|UavBsplineTrajectory|nmpc_reference_trajectory|uav_reference_circle_entry|publish_uav_reference_trajectory|alg/reference_trajectory/(flat|bspline|activate)|core/trajectory_core|core/nmpc_reference" \
  .github .xgc2 README.md px4_multirotor_controller multirotor_reference_trajectory >/dev/null; then
  echo "Deprecated reference trajectory interface found." >&2
  exit 1
fi

echo "Package compliance checks passed."
