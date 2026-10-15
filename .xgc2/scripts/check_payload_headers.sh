#!/usr/bin/env bash
set -euo pipefail

# Include search must come from the selected install, not a caller's source overlay.
unset CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
prefix="/opt/ros/${ROS_DISTRO:-noetic}"
if [[ $# == 2 && "$1" == --prefix ]]; then
  prefix="$2"
elif [[ $# != 0 ]]; then
  echo "usage: $0 [--prefix PREFIX]" >&2
  exit 2
fi

# Compile the public vocabulary from the installed tree, without source include paths. An
# installed legacy package must not conceal a dependency in either product-owned header.
work="$(mktemp -d -t xgc2-controller-payload-headers-XXXXXX)"
trap 'rm -rf -- "$work"' EXIT
for package in px4_multirotor_controller multirotor_reference_trajectory; do
  test -f "$prefix/include/$package/payloads.h"
  for language in c cpp; do
    compiler="${CC:-cc}"
    standard=c11
    source=payloads_layout.c
    if [[ "$language" == cpp ]]; then
      compiler="${CXX:-c++}"
      standard=c++11
      source=payloads_api.cpp
    fi
    output="$work/$package-$language"
    "$compiler" "-std=$standard" -Wall -Wextra -Werror -pedantic-errors \
      -I "$prefix/include" -MD -MF "$output.d" \
      "$repo_root/$package/test/$source" -o "$output"
    if grep -Eq 'xgc-robotics-(interfaces|wire)/|XgcRobotics(Interfaces|Wire)/' "$output.d"; then
      echo "installed $package payload header depends on retired robotics-interfaces" >&2
      exit 1
    fi
    "$output"
  done
done
echo 'Installed payload headers compile independently in C11 and C++11'
