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
#include <cstdint>
#include <string>
#include <utility>
#include <vector>
#include "px4_multirotor_controller/common/world_boundary.h"
namespace px4_multirotor_controller {
enum class FlightSignal : uint32_t;
enum class TrackingBackend { PX4_LOCAL, NMPC, DFBC };
enum class FeedbackSource { RIGID_ESTIMATE, MAVROS_LOCAL };
inline bool trackingUsesFusedEstimate(TrackingBackend backend) {
    return backend != TrackingBackend::PX4_LOCAL;
}
struct SensorData {
    FeedbackSource feedback_source{FeedbackSource::RIGID_ESTIMATE};
    struct TopicStats { bool is_active{true}; bool is_new{false}; };
    TopicStats vrpn_pose_stats, local_pos_stats, local_velocity_stats, uav_state_estimate_stats, state_stats, imu_stats;
    double vrpn_x{0}, vrpn_y{0}, vrpn_z{0}, local_x{0}, local_y{0}, local_z{0};
    double local_vx{0}, local_vy{0}, local_vz{0}, x{0}, y{0}, z{0};
    bool usable{true};
};
struct ControllerConfig {
    TrackingBackend tracking_backend{TrackingBackend::PX4_LOCAL};
    struct Safety {
        std::optional<WorldBoundary> world_boundary;
        double max_velocity_xy{5}, max_velocity_z{2};
        double acc_saturation_xy{3}, acc_saturation_z{3}, state_estimate_unusable_trip_delay{.15};
    } safety;
};
struct Setpoint { double ax{0}, ay{0}, az{0}; };
struct DroneController {
    SensorData sensor;
    ControllerConfig config;
    Setpoint setpoint;
    std::vector<FlightSignal> signals;
    void emitFlightSignal(FlightSignal signal) { signals.push_back(signal); }
    const SensorData& getSensorData() const { return sensor; }
    const ControllerConfig& getConfig() const { return config; }
    const Setpoint& getSetpoint() const { return setpoint; }
    double getCurrentTime() const { return 100; }
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
#include "px4_multirotor_controller/uav/state_machine/health_monitor_state.h"
using namespace px4_multirotor_controller;
struct Fixture {
    DroneController controller;
    HealthMonitorState health{controller};
    const auto& getPositionDistance() const { return health.positionDistance(); }
    void tick() {
        health.onTick();
        controller.sensor.vrpn_pose_stats.is_new = false;
        controller.sensor.local_pos_stats.is_new = false;
        controller.sensor.uav_state_estimate_stats.is_new = false;
        controller.sensor.imu_stats.is_new = false;
    }
    void pair() {
        controller.sensor.vrpn_pose_stats.is_new = true;
        controller.sensor.local_pos_stats.is_new = true;
        tick();
    }
};
void testPairRequiredAndClosedBoundary() {
    Fixture f;
    assert(!f.getPositionDistance().available);
    assert(f.getPositionDistance().exceeded == (false));
    f.controller.sensor.vrpn_x = .6;
    f.controller.sensor.vrpn_y = .8;
    f.controller.sensor.vrpn_pose_stats.is_new = true;
    f.tick();
    assert(!f.getPositionDistance().available);
    assert(!f.getPositionDistance().available);
    f.controller.sensor.local_pos_stats.is_new = true;
    f.tick();
    assert(f.getPositionDistance().available);
    assert(std::abs(f.getPositionDistance().metres - 1.0) < 1e-12);
    assert(f.getPositionDistance().available && !f.getPositionDistance().exceeded);
    assert(f.getPositionDistance().exceeded == (false));
}
void testEitherDirtyInputAndRecovery() {
    Fixture f;
    f.pair();
    for (double delta : {.05, .999, 1.0, 1.001}) {
        f.controller.sensor.vrpn_z = delta;
        f.controller.sensor.vrpn_pose_stats.is_new = true;
        f.tick();
        assert(f.getPositionDistance().available && f.getPositionDistance().exceeded == (delta > 1.0));
        assert(f.getPositionDistance().exceeded == (delta > 1.0));
    }
    f.controller.sensor.local_z = 1.001;
    f.controller.sensor.local_pos_stats.is_new = true;
    f.tick();
    assert(f.getPositionDistance().metres == 0.0);
    assert(f.getPositionDistance().available && !f.getPositionDistance().exceeded);
    assert(f.getPositionDistance().exceeded == (false));
    // 3-D Euclidean distance, not a per-axis 1 m check.
    f.controller.sensor.vrpn_x = .8;
    f.controller.sensor.vrpn_y = .8;
    f.controller.sensor.vrpn_pose_stats.is_new = true;
    f.tick();
    assert(f.getPositionDistance().exceeded);
    assert(f.getPositionDistance().exceeded == (true));
}
void testNoDirtyInputKeepsCacheAndEmitsNothing() {
    Fixture f;
    f.controller.sensor.vrpn_x = 1.25;
    f.pair();
    f.controller.sensor.vrpn_x = 0;
    for (int i = 0; i < 1000; ++i) {
        f.controller.sensor.imu_stats.is_new = true;
        f.tick();
        assert(f.getPositionDistance().metres == 1.25);
        assert(f.getPositionDistance().exceeded == (true));
    }
    assert(f.controller.signals.empty());
    f.controller.sensor.vrpn_pose_stats.is_new = true;
    f.tick();
    assert(f.getPositionDistance().metres == 0.0);
    assert(f.controller.signals.empty());
}
void testBackendDoesNotReplaceCanonicalInput() {
    Fixture f;
    f.controller.config.tracking_backend = TrackingBackend::DFBC;
    f.controller.sensor.x = -50;
    f.controller.sensor.vrpn_x = 10;
    f.controller.sensor.local_x = 10.5;
    f.pair();
    assert(f.getPositionDistance().metres == .5);
    assert(f.getPositionDistance().available && !f.getPositionDistance().exceeded);
    f.controller.sensor.x = 50;
    f.controller.sensor.uav_state_estimate_stats.is_new = true;
    f.tick();
    assert(f.getPositionDistance().metres == .5);
}
void testUnavailableIsNotAnAdditionalLandingDetector() {
    Fixture f;
    f.controller.sensor.vrpn_x = std::numeric_limits<double>::quiet_NaN();
    f.pair();
    assert(!f.getPositionDistance().available);
    assert(!f.getPositionDistance().available);
    assert(f.getPositionDistance().exceeded == (false));
    assert(f.controller.signals.empty());
}
int main() {
    testPairRequiredAndClosedBoundary();
    testEitherDirtyInputAndRecovery();
    testNoDirtyInputKeepsCacheAndEmitsNothing();
    testBackendDoesNotReplaceCanonicalInput();
    testUnavailableIsNotAnAdditionalLandingDetector();
    std::cout << "5 production HealthMonitor distance cases passed\n";
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
        for name in ["px4_local_1m.yaml", "uav_nmpc.yaml"]:
            text = (PACKAGE / "config" / name).read_text()
            values = re.findall(r"^vrpn_pose_topic:\s*(\S+)\s*$", text, re.MULTILINE)
            self.assertEqual(values, ["pose"], name)
            self.assertNotIn("/vrpn_client_node/", text, name)


if __name__ == "__main__":
    unittest.main()
