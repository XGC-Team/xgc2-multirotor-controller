#!/usr/bin/env python3
"""Compile the production HealthMonitor against deterministic sensor boundaries.

Only ROS/controller/FSM boundaries are doubled. This does not run the full FSM,
ROS transport, MAVLink services, SITL, or a vehicle. The catkin runtime test covers
actual DroneController/FSM transitions separately.
"""
from pathlib import Path
import os
import re
import shlex
import shutil
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
SUPPORT = r'''
#pragma once
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include "px4_multirotor_controller/common/world_boundary.h"
namespace px4_multirotor_controller {
enum class FlightSignal : uint32_t;
enum class FlightState : uint32_t { SelfCheck = 1, Ready = 2, Landing = 9 };
enum class TrackingBackend { PX4_LOCAL, NMPC, DFBC };
enum class FeedbackSource { RIGID_ESTIMATE, MAVROS_LOCAL };
inline bool trackingUsesFusedEstimate(TrackingBackend backend) {
    return backend != TrackingBackend::PX4_LOCAL;
}
struct SensorData {
    FeedbackSource feedback_source{FeedbackSource::RIGID_ESTIMATE};
    struct TopicStats { bool is_active{true}; bool is_new{false}; double time_since_last_msg{0}; };
    TopicStats vrpn_pose_stats, local_pos_stats, local_velocity_stats, uav_state_estimate_stats, state_stats, imu_stats;
    double vrpn_x{0}, vrpn_y{0}, vrpn_z{0}, vrpn_qx{0}, vrpn_qy{0}, vrpn_qz{0}, vrpn_qw{1};
    double local_x{0}, local_y{0}, local_z{0}, local_qx{0}, local_qy{0}, local_qz{0}, local_qw{1};
    double local_vx{0}, local_vy{0}, local_vz{0}, x{0}, y{0}, z{0}, vx{0}, vy{0}, vz{0};
    bool usable{true};
    bool local_pose_invalid{false}, vrpn_pose_invalid{false};
    uint64_t local_pose_invalid_count{0}, vrpn_pose_invalid_count{0};
    std::optional<double> last_valid_local_rx_s, last_valid_vrpn_rx_s;
    std::optional<uint64_t> last_valid_local_stamp_ns, last_valid_vrpn_stamp_ns;
    struct PoseSample {
        uint64_t seq{0}, stamp_ns{0};
        double stamp_s{0}, rx_s{0};
        bool has_header_stamp{false};
        double x{0}, y{0}, z{0}, qx{0}, qy{0}, qz{0}, qw{1};
    };
    struct PoseRing {
        static constexpr size_t kCapacity = 64;
        std::array<PoseSample, kCapacity> samples{};
        size_t head{0}, count{0};
    };
    uint64_t local_pose_seq{0}, vrpn_pose_seq{0};
    PoseRing local_pose_ring, vrpn_pose_ring;
};
struct ControllerConfig {
    TrackingBackend tracking_backend{TrackingBackend::PX4_LOCAL};
    struct Safety {
        std::optional<WorldBoundary> world_boundary;
        bool require_control_velocity_bounds{false};
        double pose_stale_timeout_s{2.0};
        WorldControlBounds mocap_gross_bounds{-100.0, 100.0, -50.0, 50.0, -5.0, 10.0};
        double localization_delay_s{0.0};
        double max_aligned_position_error_m{0.50};
        double max_pairing_gap_s{0.100};
        double max_velocity_xy{5}, max_velocity_z{2};
        double acc_saturation_xy{3}, acc_saturation_z{3}, state_estimate_unusable_trip_delay{.15};
    } safety;
};
struct Setpoint { double ax{0}, ay{0}, az{0}; };
struct DroneController {
    SensorData sensor;
    ControllerConfig config;
    Setpoint setpoint;
    FlightState current_state{FlightState::Ready};
    double now{100.0};
    std::vector<FlightSignal> signals;
    void emitFlightSignal(FlightSignal signal) { signals.push_back(signal); }
    FlightState flightState() const { return current_state; }
    const SensorData& getSensorData() const { return sensor; }
    const ControllerConfig& getConfig() const { return config; }
    const Setpoint& getSetpoint() const { return setpoint; }
    double getCurrentTime() const { return now; }
    template<class... Args> void logError(const char*, Args...) const {}
};
namespace event_type {
constexpr uint32_t ALTCTL_READY=16, OFFBOARD_READY=17, ARM_READY=18, ALTITUDE_REACHED=10;
constexpr uint32_t TAKEOFF_TIMEOUT=15, REFERENCE_TRAJECTORY_FINISHED=19, TOUCHDOWN=13;
constexpr uint32_t SAFE_TIMEOUT_UAV_STATE_ESTIMATE = 27, SAFE_TIMEOUT_STATE = 23;
constexpr uint32_t SAFE_TIMEOUT_LOCAL_POS=20, SAFE_TIMEOUT_LOCAL_VELOCITY=21, SAFE_TIMEOUT_IMU=22;
constexpr uint32_t SAFE_UAV_STATE_ESTIMATE_UNUSABLE = 28, SAFE_GEOFENCE_VIOLATION = 40;
constexpr uint32_t SAFE_VELOCITY_XY_EXCEEDED = 41, SAFE_VELOCITY_Z_EXCEEDED = 42;
constexpr uint32_t SAFE_CONTROL_SATURATION_XY = 43, SAFE_CONTROL_SATURATION_Z = 44;
constexpr uint32_t SAFE_POSITION_JUMP = 47, SAFE_VRPN_POSITION_JUMP = 49;
constexpr uint32_t SAFE_TIMEOUT_VRPN_POSE = 50, SAFE_VRPN_POSE_INVALID = 51;
constexpr uint32_t SAFE_LOCAL_POSE_INVALID = 52, SAFE_MOCAP_GROSS_BOUNDS_VIOLATION = 53;
constexpr uint32_t SAFE_ALIGNED_POSE_ERROR_EXCEEDED = 54, SAFE_LOCAL_GROSS_BOUNDS_VIOLATION = 55;
}
namespace sensor_checks {
inline bool isControlStateActive(const SensorData& s) { return s.uav_state_estimate_stats.is_active; }
inline bool isControlStateUsableForControl(const SensorData& s) { return s.usable; }
inline bool isWorldPoseNew(const SensorData& s, TrackingBackend b) {
    return trackingUsesFusedEstimate(b) ? s.uav_state_estimate_stats.is_new : s.local_pos_stats.is_new;
}
inline double worldX(const SensorData& s, TrackingBackend b) {
    return trackingUsesFusedEstimate(b) ? s.x : s.local_x;
}
inline double worldY(const SensorData& s, TrackingBackend b) {
    return trackingUsesFusedEstimate(b) ? s.y : s.local_y;
}
inline double worldZ(const SensorData& s, TrackingBackend b) {
    return trackingUsesFusedEstimate(b) ? s.z : s.local_z;
}
inline double worldVx(const SensorData& s, TrackingBackend) { return s.local_vx; }
inline double worldVy(const SensorData& s, TrackingBackend) { return s.local_vy; }
inline double worldVz(const SensorData& s, TrackingBackend) { return s.local_vz; }
}
}
'''
TEST = r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "px4_multirotor_controller/driver/pose_ingress.h"
#include "px4_multirotor_controller/uav/state_machine/health_monitor_state.h"
using namespace px4_multirotor_controller;
struct Fixture {
    DroneController controller;
    HealthMonitorState health{controller};
    const auto& getPositionDistance() const { return health.positionDistance(); }
    void pushLocal(double stamp_s, double x, double y, double z,
                   double qx = 0, double qy = 0, double qz = 0, double qw = 1) {
        const uint64_t ns = static_cast<uint64_t>(std::llround(stamp_s * 1e9));
        ingestLocalPoseSample(controller.sensor, static_cast<uint32_t>(ns / 1000000000ULL),
                              static_cast<uint32_t>(ns % 1000000000ULL), stamp_s,
                              x, y, z, qx, qy, qz, qw);
        controller.sensor.local_pos_stats.is_new = true;
    }
    void pushVrpn(double stamp_s, double x, double y, double z,
                  double qx = 0, double qy = 0, double qz = 0, double qw = 1) {
        const uint64_t ns = static_cast<uint64_t>(std::llround(stamp_s * 1e9));
        ingestVrpnPoseSample(controller.sensor, static_cast<uint32_t>(ns / 1000000000ULL),
                             static_cast<uint32_t>(ns % 1000000000ULL), stamp_s,
                             x, y, z, qx, qy, qz, qw);
        controller.sensor.vrpn_pose_stats.is_new = true;
    }
    void tick(double now_s) {
        controller.now = now_s;
        health.onTick();
        controller.sensor.vrpn_pose_stats.is_new = false;
        controller.sensor.local_pos_stats.is_new = false;
        controller.sensor.uav_state_estimate_stats.is_new = false;
        controller.sensor.imu_stats.is_new = false;
    }
};
void testAligned50MsAnd0MsPairingAndClosedThreshold() {
    Fixture f;
    f.controller.config.safety.localization_delay_s = 0.050;
    f.pushLocal(100.00, 0.0, 0.0, 1.0);
    f.pushLocal(100.04, 0.48, 0.0, 1.0);
    f.pushLocal(100.08, 0.96, 0.0, 1.0);
    f.pushVrpn(100.09, 0.48, 0.0, 1.0);
    f.tick(100.09);
    assert(f.getPositionDistance().available);
    assert(std::abs(f.getPositionDistance().metres - 0.0) < 1e-9);
    assert(!f.getPositionDistance().exceeded);
    assert(f.controller.signals.empty());

    // Gap > 100 ms in local history marks available=false without emitting a fault.
    f.pushLocal(100.25, 0.96, 0.0, 1.0);
    f.pushVrpn(100.20, 2.0, 0.0, 1.0);
    f.tick(100.26);
    assert(!f.getPositionDistance().available);
    assert(!f.getPositionDistance().exceeded);
    assert(f.controller.signals.empty());

    // Direct 0 ms pose threshold 0.50 m exact endpoint vs breach.
    Fixture f0;
    f0.controller.config.safety.localization_delay_s = 0.0;
    f0.pushLocal(100.00, 0.0, 0.0, 1.0);
    f0.pushVrpn(100.00, 0.50, 0.0, 1.0);
    f0.tick(100.00);
    assert(f0.getPositionDistance().available && !f0.getPositionDistance().exceeded);
    f0.pushLocal(100.05, 0.0, 0.0, 1.0);
    f0.pushVrpn(100.05, 0.501, 0.0, 1.0);
    f0.tick(100.05);
    assert(f0.getPositionDistance().exceeded);
    assert(f0.controller.signals.size() == 1 && f0.controller.signals[0] == FlightSignal::AlignedPoseError);
}
void testPerAxisJumpDiagonalAndNormalizedQuatAndLocalJump() {
    Fixture f;
    f.controller.config.safety.max_aligned_position_error_m = 5.0;
    f.pushLocal(100.00, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    f.pushVrpn(100.00, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    f.tick(100.00);
    // Repeated valid non-unit quaternion (0,0,0,0.96) at 4 ms -> 0 rad angular jump.
    f.pushLocal(100.004, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    f.pushVrpn(100.004, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    f.tick(100.004);
    assert(f.controller.signals.empty());

    // Diagonal motion (0.70, 0.70, 0.70) at 0.033 s: hypot=1.212 > 1.198, max_axis=0.70 <= 1.198 -> OK.
    f.pushLocal(100.037, 0.70, 0.70, 1.70, 0.0, 0.0, 0.0, 0.96);
    f.pushVrpn(100.037, 0.70, 0.70, 1.70, 0.0, 0.0, 0.0, 0.96);
    f.tick(100.037);
    assert(f.controller.signals.empty());

    // Single-axis local jump dx=1.25 > 1.198 -> LocalJump and local position faulted!
    f.pushLocal(100.070, 1.95, 0.70, 1.70, 0.0, 0.0, 0.0, 0.96);
    f.tick(100.070);
    assert(f.controller.signals.size() == 1 && f.controller.signals[0] == FlightSignal::LocalJump);
    assert(f.health.isLocalPositionFaulted());
}
void testInvalidBurstAndBackendDoesNotReplaceCanonical() {
    Fixture f;
    f.controller.config.tracking_backend = TrackingBackend::DFBC;
    f.controller.sensor.x = -50.0;
    f.pushLocal(100.00, 10.5, 0.0, 1.0);
    f.pushVrpn(100.00, 10.0, 0.0, 1.0);
    f.tick(100.00);
    assert(std::abs(f.getPositionDistance().metres - 0.5) < 1e-9);
    assert(!f.getPositionDistance().exceeded);
    f.controller.sensor.x = 50.0;
    f.controller.sensor.uav_state_estimate_stats.is_new = true;
    f.tick(100.01);
    assert(std::abs(f.getPositionDistance().metres - 0.5) < 1e-9);

    // Invalid sample followed by valid sample in the same burst STILL fires VrpnPoseInvalid!
    const double qnan = std::numeric_limits<double>::quiet_NaN();
    assert(ingestVrpnPoseSample(f.controller.sensor, 100, 15000000, 100.015,
                                qnan, qnan, qnan, qnan, qnan, qnan, qnan) ==
           PoseIngressResult::RejectedInvalid);
    f.pushVrpn(100.018, 10.0, 0.0, 1.0);
    f.tick(100.02);
    assert(f.controller.signals.size() == 1 &&
           f.controller.signals[0] == FlightSignal::VrpnPoseInvalid);
}
int main() {
    testAligned50MsAnd0MsPairingAndClosedThreshold();
    testPerAxisJumpDiagonalAndNormalizedQuatAndLocalJump();
    testInvalidBurstAndBackendDoesNotReplaceCanonical();
    std::cout << "3 production HealthMonitor aligned/jump/invalid cases passed\n";
}
'''



class PositionDistanceTest(unittest.TestCase):
    def test_production_monitor_distance_facts(self):
        compiler = shutil.which(os.environ.get("CXX", "g++"))
        self.assertIsNotNone(compiler, "A C++17 compiler is required; do not silently skip")
        with tempfile.TemporaryDirectory(prefix="position-distance-") as directory:
            root = Path(directory)
            (root / "support.h").write_text(SUPPORT)
            for name in [
                "px4_multirotor_controller/common/types.h",
                "px4_multirotor_controller/common/sensor_checks.h",
                "px4_multirotor_controller/drone_controller.h",
            ]:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('#pragma once\n#include "support.h"\n')
            (root / "test.cpp").write_text(TEST)
            binary = root / "position-distance-test"
            subprocess.run([
                compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                *shlex.split(os.environ.get("CXXFLAGS", "")),
                "-I" + str(root), "-I" + str(PACKAGE / "include"),
                str(PACKAGE / "src/uav/state_machine/health_monitor_state.cpp"),
                str(root / "test.cpp"), "-o", str(binary),
            ], check=True, timeout=60)
            subprocess.run([str(binary)], check=True, timeout=30)

    def test_single_calculation_owner_and_no_legacy_predicate(self):
        for directory in [PACKAGE / "src", PACKAGE / "include"]:
            for path in directory.rglob("*"):
                if path.suffix not in {".cpp", ".h"}:
                    continue
                source = path.read_text()
                self.assertNotIn("vrpnLocalPositionDiff", source, str(path))
                self.assertNotIn("isVrpnPoseConsistent", source, str(path))
        self_check = (PACKAGE / "src/uav/state_machine/self_check_state.cpp").read_text()
        self.assertIn("controller_.getPositionDistance()", self_check)
        self.assertIn("distance.metres", self_check)

    def test_profiles_use_only_namespaced_canonical_pose(self):
        import yaml
        for name in ["px4_local_1m.yaml", "dfbc_local_1m.yaml", "uav_nmpc.yaml"]:
            text = (PACKAGE / "config" / name).read_text()
            data = yaml.safe_load(text)
            self.assertEqual(data.get("pose_source"), "simulation", name)
            self.assertNotIn("vrpn_pose_topic", data, name)
            self.assertNotIn("/vrpn_client_node/", text, name)


if __name__ == "__main__":
    unittest.main()
