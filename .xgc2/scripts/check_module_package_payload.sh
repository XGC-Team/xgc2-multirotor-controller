#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || { echo 'usage: check_module_package_payload.sh INSTALL_ROOT DEB_DIR' >&2; exit 2; }
install_root="$1"
deb_dir="$2"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
prefix="/opt/ros/${ROS_DISTRO:-noetic}"
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
shopt -s nullglob
debs=("$deb_dir"/ros-"${ROS_DISTRO:-noetic}"-xgc2-multirotor-controller_*.deb)
[[ ${#debs[@]} == 1 ]] || { echo 'expected exactly one controller Deb' >&2; exit 1; }
dpkg-deb --extract "${debs[0]}" "$work/payload"

modules=(libpx4_multirotor_reference_module.so libpx4_multirotor_controller_module.so)
for library in "${modules[@]}"; do
  path="$work/payload$prefix/lib/$library"
  test -f "$path"
  file -b "$path" | grep -q '^ELF'
  # A module exports its entry point and nothing else, and links no ROS library.
  exported="$(nm -D --defined-only "$path" | awk '{print $3}')"
  [[ "$exported" == "xgc2_module_entry" ]] || { echo "$library exports: $exported" >&2; exit 1; }
  if readelf -d "$path" | grep -E 'NEEDED.*(libros|libroscpp|librosconsole|librostime|libcpp_common|libxmlrpcpp|libtf|libmavros)'; then
    echo "$library links ROS" >&2
    exit 1
  fi
  cmp "$install_root$prefix/lib/$library" "$path"
done
for missing in "${modules[@]}"; do
  root="$work/missing-$missing"
  mkdir -p "$root$prefix/lib"
  cp -a "$install_root/." "$root/"
  rm -- "$root$prefix/lib/$missing"
  if "$script_dir/package_debs.sh" --install-root "$root" --output-dir "$work/out-$missing" >"$work/$missing.log" 2>&1; then
    echo "packager accepted missing $missing" >&2; exit 1
  fi
  grep -Fq "missing required installed module library: $root$prefix/lib/$missing" "$work/$missing.log"
  [[ ! -d "$work/out-$missing" ]] || [[ -z "$(find "$work/out-$missing" -name '*.deb' -print -quit)" ]]
  echo "PASS: missing $missing refused"
done
echo 'PASS: the controller Deb contains both modules, exporting only their entry point and linking no ROS'

# The payload headers are the public vocabulary of the modules; the retired wire packages are gone.
payload_headers=(
  "${prefix}/include/multirotor_reference_trajectory/payloads.h"
  "${prefix}/include/px4_multirotor_controller/payloads.h"
)
for path in "${payload_headers[@]}"; do
  cmp "$install_root$path" "$work/payload$path"
done
for missing in "${payload_headers[@]}"; do
  root="$work/missing-header"
  rm -rf -- "$root" "$work/header-out"
  mkdir -p "$root"
  cp -a "$install_root/." "$root/"
  rm -- "$root$missing"
  if "$script_dir/package_debs.sh" --install-root "$root" --output-dir "$work/header-out" >"$work/header-negative.log" 2>&1; then
    echo "packager accepted missing payload header: $missing" >&2; exit 1
  fi
  grep -Fq "missing required installed payload header: $missing" "$work/header-negative.log"
  [[ ! -d "$work/header-out" ]] || [[ -z "$(find "$work/header-out" -name '*.deb' -print -quit)" ]]
done
retired=(
  "$work/payload$prefix/include/multirotor_reference_trajectory/reference_wire.hpp"
  "$work/payload$prefix/include/multirotor_reference_trajectory/reference_wire_v1.h"
  "$work/payload$prefix/share/cmake/ReferenceTrajectoryNative"
  "$work/payload$prefix/lib/libctl_px4.so"
  "$work/payload$prefix/lib/libref_trajectory.so"
)
for path in "${retired[@]}"; do
  [[ ! -e "$path" ]] || { echo "retired file is packaged: ${path#"$work/payload"}" >&2; exit 1; }
done
echo 'PASS: payload headers match the install root and are required; retired wire files are absent'
