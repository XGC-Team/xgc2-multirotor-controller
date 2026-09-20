#!/usr/bin/env python3
"""Compile the production HealthMonitor and production guard lambdas.

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
namespace state_machine {
using EventId = uint32_t;
struct ActionResult {};
struct GuardContext {};
struct Status {
    std::string message;
    bool ok() const { return true; }
};
struct EventTimestamp { double seconds; };
struct Event {
    EventId id;
    EventTimestamp timestamp;
    std::string source;
    Event(EventId value, EventTimestamp stamp) : id(value), timestamp(stamp) {}
};
struct StateContext {
    std::vector<Event> internals;
    Status postInternalEvent(Event event) { internals.push_back(std::move(event)); return {}; }
};
struct State {
    virtual ~State() = default;
    virtual std::string name() const = 0;
    void tick(StateContext& ctx) { (void)onTick(ctx); }
protected:
    virtual ActionResult onTick(StateContext&) { return {}; }
};
}
namespace px4_multirotor_controller {
enum class TrackingBackend { PX4_LOCAL, NMPC, DFBC };
inline bool trackingUsesFusedEstimate(TrackingBackend backend) {
    return backend != TrackingBackend::PX4_LOCAL;
}
struct SensorData {
    struct TopicStats { bool is_active{true}; bool is_new{false}; };
    TopicStats vrpn_pose_stats, local_pos_stats, uav_state_estimate_stats, state_stats, imu_stats;
    double vrpn_x{0}, vrpn_y{0}, vrpn_z{0}, local_x{0}, local_y{0}, local_z{0};
    double local_vx{0}, local_vy{0}, local_vz{0}, x{0}, y{0}, z{0};
    bool usable{true};
};
struct ControllerConfig {
    TrackingBackend tracking_backend{TrackingBackend::PX4_LOCAL};
    struct Safety {
        double fence_x_min{-100}, fence_x_max{100}, fence_y_min{-100}, fence_y_max{100};
        double fence_z_min{-100}, fence_z_max{100}, max_velocity_xy{5}, max_velocity_z{2};
        double acc_saturation_xy{3}, acc_saturation_z{3}, state_estimate_unusable_trip_delay{.15};
    } safety;
};
struct Setpoint { double ax{0}, ay{0}, az{0}; };
struct DroneController {
    SensorData sensor;
    ControllerConfig config;
    Setpoint setpoint;
    const SensorData& getSensorData() const { return sensor; }
    const ControllerConfig& getConfig() const { return config; }
    const Setpoint& getSetpoint() const { return setpoint; }
    double getCurrentTime() const { return 100; }
    template<class... Args> void logError(const char*, Args...) const {}
};
namespace event_type {
constexpr uint32_t SAFE_TIMEOUT_UAV_STATE_ESTIMATE = 27, SAFE_TIMEOUT_STATE = 23;
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
    state_machine::StateContext context;
    const auto& getPositionDistance() const { return health.positionDistance(); }
    #include "production_guards.inc"
    void tick() {
        health.tick(context);
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
void checkLandingGuards(const Fixture& f, bool expected) {
    assert(f.landFromTakeoff() == expected);
    assert(f.landFromHover() == expected);
    assert(f.landFromCustom1() == expected);
}
void testPairRequiredAndClosedBoundary() {
    Fixture f;
    assert(!f.takeoffAllowed());
    checkLandingGuards(f, false);
    f.controller.sensor.vrpn_x = .6;
    f.controller.sensor.vrpn_y = .8;
    f.controller.sensor.vrpn_pose_stats.is_new = true;
    f.tick();
    assert(!f.getPositionDistance().available);
    assert(!f.takeoffAllowed());
    f.controller.sensor.local_pos_stats.is_new = true;
    f.tick();
    assert(f.getPositionDistance().available);
    assert(std::abs(f.getPositionDistance().metres - 1.0) < 1e-12);
    assert(f.takeoffAllowed());
    checkLandingGuards(f, false);
}
void testEitherDirtyInputAndRecovery() {
    Fixture f;
    f.pair();
    for (double delta : {.05, .999, 1.0, 1.001}) {
        f.controller.sensor.vrpn_z = delta;
        f.controller.sensor.vrpn_pose_stats.is_new = true;
        f.tick();
        assert(f.takeoffAllowed() == (delta <= 1.0));
        checkLandingGuards(f, delta > 1.0);
    }
    f.controller.sensor.local_z = 1.001;
    f.controller.sensor.local_pos_stats.is_new = true;
    f.tick();
    assert(f.getPositionDistance().metres == 0.0);
    assert(f.takeoffAllowed());
    checkLandingGuards(f, false);
    // 3-D Euclidean distance, not a per-axis 1 m check.
    f.controller.sensor.vrpn_x = .8;
    f.controller.sensor.vrpn_y = .8;
    f.controller.sensor.vrpn_pose_stats.is_new = true;
    f.tick();
    assert(!f.takeoffAllowed());
    checkLandingGuards(f, true);
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
        checkLandingGuards(f, true);
    }
    assert(f.context.internals.empty());
    f.controller.sensor.vrpn_pose_stats.is_new = true;
    f.tick();
    assert(f.getPositionDistance().metres == 0.0);
    assert(f.context.internals.empty());
}
void testBackendDoesNotReplaceCanonicalInput() {
    Fixture f;
    f.controller.config.tracking_backend = TrackingBackend::DFBC;
    f.controller.sensor.x = -50;
    f.controller.sensor.vrpn_x = 10;
    f.controller.sensor.local_x = 10.5;
    f.pair();
    assert(f.getPositionDistance().metres == .5);
    assert(f.takeoffAllowed());
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
    assert(!f.takeoffAllowed());
    checkLandingGuards(f, false);
    assert(f.context.internals.empty());
}
int main() {
    testPairRequiredAndClosedBoundary();
    testEitherDirtyInputAndRecovery();
    testNoDirtyInputKeepsCacheAndEmitsNothing();
    testBackendDoesNotReplaceCanonicalInput();
    testUnavailableIsNotAnAdditionalLandingDetector();
    std::cout << "5 production HealthMonitor/guard cases passed\n";
}
'''


def production_guards():
    """Execute the actual four lambdas, not a second Python/C++ policy."""
    source = (PACKAGE / "src/drone_controller.cpp").read_text()
    guards = {}
    for block in source.split(".transition()")[1:]:
        if "getPositionDistance()" not in block:
            continue
        source_state = re.search(r"\.from\(state_type::(\w+)\)", block).group(1)
        target = re.search(r"\.to\(state_type::(\w+)\)", block).group(1)
        guard = re.search(
            r"\.when\((\[this\]\(const ::state_machine::GuardContext&\)\s*\{.*?\})\)",
            block, re.DOTALL,
        )
        if guard is None:
            raise AssertionError("Cannot bind production position guard")
        if source_state == "Ready":
            assert target == "Takeoff" and ".on(TAKEOFF_REQUESTED)" in block
            name = "takeoffAllowed"
        else:
            assert source_state in {"Takeoff", "Hover", "Custom1"}
            assert target == "Landing" and ".on(" not in block
            assert ".priority(transition_priority::EMERGENCY)" in block
            name = "landFrom" + source_state
        assert name not in guards
        guards[name] = (
            "bool " + name + "() const { auto guard = " + guard.group(1)
            + "; return guard(state_machine::GuardContext{}); }\n"
        )
    assert set(guards) == {"takeoffAllowed", "landFromTakeoff", "landFromHover", "landFromCustom1"}
    return "".join(guards.values())


class PositionDistanceTest(unittest.TestCase):
    def test_production_monitor_and_guards(self):
        compiler = shutil.which(os.environ.get("CXX", "g++"))
        self.assertIsNotNone(compiler, "A C++17 compiler is required; do not silently skip")
        with tempfile.TemporaryDirectory(prefix="position-distance-") as directory:
            root = Path(directory)
            (root / "support.h").write_text(SUPPORT)
            for name in [
                "state_machine/state_machine.hpp",
                "px4_multirotor_controller/common/types.h",
                "px4_multirotor_controller/common/sensor_checks.h",
                "px4_multirotor_controller/drone_controller.h",
            ]:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('#pragma once\n#include "support.h"\n')
            (root / "production_guards.inc").write_text(production_guards())
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
