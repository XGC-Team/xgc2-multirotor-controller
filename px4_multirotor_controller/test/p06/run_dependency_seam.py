#!/usr/bin/env python3
"""Explicit offline dependency seam, NOT a ROS/native build or flight test.

Compile the owning lifter/buffer headers unchanged. Only ros::Time and the
small types/Eigen surface they consume are doubled below. Never silently
fall back to these doubles in the native CMake target or production build.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import shlex
import subprocess
import sys
import time

ROS = r'''#pragma once
#include <cmath>
#include <cstdint>
namespace ros {
class Duration {
 public:
  explicit Duration(int64_t n) : ns_(n) {}
  double toSec() const { return ns_ / 1000000000.0; }
 private: int64_t ns_;
};
class Time {
 public:
  explicit Time(double s = 0) : ns_(std::llround(s * 1000000000.0)) {}
  bool isZero() const { return ns_ == 0; }
  static void init() { valid_ = true; }
  static bool isValid() { return valid_; }
  friend Duration operator-(const Time& a, const Time& b) { return Duration(a.ns_ - b.ns_); }
 private:
  int64_t ns_;
  inline static bool valid_ = false;
};
}
'''
TYPES = r'''#pragma once
#include <ros/ros.h>
#include <cmath>
#include <cstdint>
// Test-only dependency surface. This is not Eigen, the ROS clock or the product ABI.
namespace Eigen {
class Vector3d {
 public:
  Vector3d(double x = 0, double y = 0, double z = 0) : d_{x,y,z} {}
  static Vector3d Zero() { return {}; }
  double& x() { return d_[0]; } double x() const { return d_[0]; }
  double& y() { return d_[1]; } double y() const { return d_[1]; }
  double& z() { return d_[2]; } double z() const { return d_[2]; }
  bool allFinite() const { return std::isfinite(d_[0]) && std::isfinite(d_[1]) && std::isfinite(d_[2]); }
 private: double d_[3];
};
}
namespace px4_multirotor_controller {
constexpr uint16_t kIgnorePxBit=1u<<0, kIgnorePyBit=1u<<1, kIgnorePzBit=1u<<2;
constexpr uint16_t kIgnoreVxBit=1u<<3, kIgnoreVyBit=1u<<4, kIgnoreVzBit=1u<<5;
constexpr uint16_t kIgnoreAfxBit=1u<<6, kIgnoreAfyBit=1u<<7, kIgnoreAfzBit=1u<<8;
constexpr uint16_t kIgnoreYawBit=1u<<10, kIgnoreYawRateBit=1u<<11;
constexpr uint16_t kDefaultPvaLocalTypeMask=kIgnoreYawBit|kIgnoreYawRateBit;
struct Setpoint {
 double x{0},y{0},z{0},vx{0},vy{0},vz{0},ax{0},ay{0},az{0};
 double qx{0},qy{0},qz{0},qw{1},yaw_rate{0};
 uint16_t type_mask{0}; uint8_t coordinate_frame{1};
};
struct MpcTrajectoryState {
 Eigen::Vector3d position_k,velocity_k,acceleration_k;
 ros::Time planning_time;
 double qx{0},qy{0},qz{0},qw{1},yaw_rate{0};
 uint16_t type_mask{0}; uint8_t coordinate_frame{1};
 bool is_valid{false},new_data_received{false};
 MpcTrajectoryState() : position_k(Eigen::Vector3d::Zero()),
 velocity_k(Eigen::Vector3d::Zero()),acceleration_k(Eigen::Vector3d::Zero()) {}
};
}
'''

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (1024**3, 1024**3))
    resource.setrlimit(resource.RLIMIT_CPU, (45, 45))
    resource.setrlimit(resource.RLIMIT_FSIZE, (32*1024**2, 32*1024**2))

def digest(path):
    data = path.read_bytes()
    return {"sha256": hashlib.sha256(data).hexdigest(),
            "git_blob_sha": hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    root = args.package_root.resolve()
    out = args.output_dir.resolve()
    if out == root or root in out.parents:
        parser.error("output must be outside the source package")
    probe = Path(__file__).resolve().parents[1] / "p06_trajectory_lifter_probe.cpp"
    inputs = [root/"include/px4_multirotor_controller/control/trajectory_lifter.h",
              root/"include/px4_multirotor_controller/uav/mpc_trajectory_buffer.h", probe,
              Path(__file__).resolve()]
    for path in inputs:
        if not path.is_file(): parser.error(f"missing input: {path}")
    out.mkdir(parents=True, exist_ok=False)  # Preserve all previous runs.
    seam = out / "dependency-seam"
    for name, text in [("ros/ros.h", ROS),
                       ("px4_multirotor_controller/common/types.h", TYPES)]:
        path = seam/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        inputs.append(path)
    manifest = {"kind": "DEPENDENCY_SEAM_NOT_NATIVE_ROS", "seed": 157,
                "platform": platform.platform(), "python": sys.version,
                "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "limits": {"address_space_bytes": 1024**3, "cpu_seconds_per_process": 45,
                           "file_size_bytes": 32*1024**2, "parallel_compilers": 1},
                "inputs": {str(p): digest(p) for p in inputs}, "steps": []}
    compiler = shlex.split(os.environ.get("CXX", "g++"))
    steps = [("compiler-version", compiler + ["--version"]),
             ("compile", compiler + ["-std=c++17", "-O1", "-g0", "-Wall", "-Wextra",
               "-Werror", "-pedantic", "-fsanitize=undefined", "-fno-sanitize-recover=all",
               "-I"+str(seam), "-I"+str(root/"include"), str(probe), "-o", str(out/"probe")]),
             ("probe", [str(out/"probe")])]
    status = 2
    try:
        for name, command in steps:
            start = time.monotonic()
            with (out/(name+".stdout")).open("w") as stdout, (out/(name+".stderr")).open("w") as stderr:
                result = subprocess.run(command, stdout=stdout, stderr=stderr,
                                        timeout=50, preexec_fn=limits, check=False)
            status = result.returncode
            manifest["steps"].append({"name": name, "command": command,
                                      "returncode": status, "seconds": time.monotonic()-start})
            print(f"{name}: exit={status}")
            if status: break
    except (OSError, subprocess.TimeoutExpired) as error:
        manifest["error"] = str(error)
        status = 2
    finally:
        manifest["returncode"] = status
        (out/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n", encoding="utf-8")
    return status

if __name__ == "__main__":
    sys.exit(main())
