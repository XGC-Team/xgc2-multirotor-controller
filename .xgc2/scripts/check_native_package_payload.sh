#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || { echo 'usage: check_native_package_payload.sh INSTALL_ROOT DEB_DIR' >&2; exit 2; }
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
for library in libctl_px4.so libref_trajectory.so; do
  path="$work/payload$prefix/lib/$library"
  test -f "$path"
  file -b "$path" | grep -q '^ELF'
  nm -D --defined-only "$path" | awk '$3 == "xgc_rt_plugin_v1" {found=1} END {exit !found}'
  cmp "$install_root$prefix/lib/$library" "$path"
done
for missing in libctl_px4.so libref_trajectory.so; do
  root="$work/missing-$missing"
  mkdir -p "$root$prefix/lib"
  for library in libctl_px4.so libref_trajectory.so; do
    [[ "$library" == "$missing" ]] || cp -a "$install_root$prefix/lib/$library" "$root$prefix/lib/"
  done
  if "$script_dir/package_debs.sh" --install-root "$root" --output-dir "$work/out-$missing" >"$work/$missing.log" 2>&1; then
    echo "packager accepted missing $missing" >&2; exit 1
  fi
  grep -Fq "missing required installed native library: $root$prefix/lib/$missing" "$work/$missing.log"
  [[ ! -d "$work/out-$missing" ]] || [[ -z "$(find "$work/out-$missing" -name '*.deb' -print -quit)" ]]
  echo "PASS: missing $missing refused"
done
echo 'PASS: real controller Deb contains both owning native facades'

WIRE_PATHS=(
  "${prefix}/include/multirotor_reference_trajectory/reference_wire.hpp"
  "${prefix}/include/multirotor_reference_trajectory/reference_wire_v1.h"
  "${prefix}/share/cmake/ReferenceTrajectoryNative/ReferenceTrajectoryNativeConfig.cmake"
  "${prefix}/share/cmake/ReferenceTrajectoryNative/ReferenceTrajectoryNativeTargets.cmake"
)
for path in "${WIRE_PATHS[@]}"; do
  cmp "$install_root$path" "$work/payload$path"
done
for missing in "${WIRE_PATHS[@]}"; do
  root="$work/missing-wire"
  rm -rf -- "$root" "$work/wire-out"
  mkdir -p "$root"
  cp -al "$install_root/." "$root/"
  rm -- "$root$missing"
  if "$script_dir/package_debs.sh" --install-root "$root" --output-dir "$work/wire-out" >"$work/wire-negative.log" 2>&1; then
    echo "packager accepted missing owning DTO export: $missing" >&2; exit 1
  fi
  grep -Fq "missing required installed owning DTO export: $missing" "$work/wire-negative.log"
  [[ ! -d "$work/wire-out" ]] || [[ -z "$(find "$work/wire-out" -name '*.deb' -print -quit)" ]]
done
echo 'PASS: reference owning DTO bytes and missing-header/export refusals'
