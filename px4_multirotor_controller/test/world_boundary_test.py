#!/usr/bin/env python3
"""Compile the production worldBoundary parser, load-fact producer and geofence monitor.

Only ROS/controller/FSM boundaries are doubled. This does not run the full FSM,
ROS transport, MAVLink services, SITL, or a vehicle. JsonCpp comes from the build
image (libjsoncpp-dev) via pkg-config; the monitor binary needs no JSON library.
"""
from pathlib import Path
import os
import shlex
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
SUPPORT = r'''
#pragma once
#include <cstdint>
#include <string>
#include <utility>
#include <vector>
#include "px4_multirotor_controller/common/world_boundary.h"
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
PARSER_TEST = r'''
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <json/json.h>
#include "px4_multirotor_controller/common/world_boundary.h"
using namespace px4_multirotor_controller;

constexpr const char* kFull =
    "{\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"m\","
    "\"controlBounds\":{\"xMin\":-4.5,\"xMax\":4.5,\"yMin\":-3.25,\"yMax\":3.25,"
    "\"zMin\":0.0,\"zMax\":2.5},\"groundZ\":0.125}";

void expectInvalid(const std::string& raw) {
    try {
        parseWorldBoundary(raw);
    } catch (const std::invalid_argument&) {
        return;
    }
    std::cerr << "expected invalid_argument for: " << raw << "\n";
    std::abort();
}

Json::Value reparsed(const std::string& text) {
    Json::Value value;
    std::string errors;
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    bool ok = reader->parse(text.data(), text.data() + text.size(), &value, &errors);
    assert(ok);
    return value;
}

ControllerLoadFact makeFact() {
    ControllerLoadFact fact;
    fact.node_name = "/uav3/px4_multirotor_controller";
    fact.instance_id = "0f4b2c3a-1111-4222-8333-9abcdef01234";
    fact.canonical_pose_topic = "/uav3/pose";
    fact.local_pose_topic = "/uav3/mavros/local_position/pose";
    fact.world_pose_topic = "/uav3/alg/state_estimator/state";
    fact.tracking_backend = "dfbc";
    fact.position_distance_limit_metres = 1.0;
    // Beyond 2^53: only exact decimal strings survive the wire, never doubles.
    fact.ros_time_ns = 9223372036854775807ULL;
    fact.unix_time_ns = 18446744073709551615ULL;
    fact.monotonic_time_ns = 12345678901234567ULL;
    return fact;
}

void testNullAndWhitespaceNullAreUnset() {
    assert(!parseWorldBoundary("null").has_value());
    assert(!parseWorldBoundary("  null \n").has_value());
}

void testCompleteBoundsExactValuesNoOffset() {
    const auto boundary = parseWorldBoundary(kFull);
    assert(boundary && boundary->control_bounds);
    const auto& bounds = *boundary->control_bounds;
    assert(bounds.x_min == -4.5 && bounds.x_max == 4.5);
    assert(bounds.y_min == -3.25 && bounds.y_max == 3.25);
    assert(bounds.z_min == 0.0 && bounds.z_max == 2.5);
    assert(boundary->ground_z && *boundary->ground_z == 0.125);
}

void testNullMembers() {
    const auto no_bounds = parseWorldBoundary(
        "{\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"m\","
        "\"controlBounds\":null,\"groundZ\":0.5}");
    assert(no_bounds && !no_bounds->control_bounds);
    assert(no_bounds->ground_z && *no_bounds->ground_z == 0.5);
    const auto no_ground = parseWorldBoundary(
        "{\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"m\","
        "\"controlBounds\":{\"xMin\":-1,\"xMax\":1,\"yMin\":-1,\"yMax\":1,"
        "\"zMin\":0,\"zMax\":2},\"groundZ\":null}");
    assert(no_ground && no_ground->control_bounds && !no_ground->ground_z);
}

void testInvalidContractsThrowBeforeSetConfig() {
    expectInvalid("42");                       // non-object root
    expectInvalid("[1,2,3]");                  // array root
    expectInvalid("{} {}");                    // trailing garbage
    expectInvalid("{}");                       // missing keys
    expectInvalid(
        "{\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"m\",\"controlBounds\":null}");
    expectInvalid(                             // unknown extra key
        "{\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"m\",\"controlBounds\":null,"
        "\"groundZ\":null,\"extra\":0}");
    expectInvalid(                             // duplicate key
        "{\"schemaVersion\":1,\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"m\","
        "\"controlBounds\":null,\"groundZ\":null}");
    expectInvalid(                             // wrong schema version
        "{\"schemaVersion\":2,\"frameId\":\"world\",\"unit\":\"m\",\"controlBounds\":null,"
        "\"groundZ\":null}");
    expectInvalid(                             // schema version must be integer 1
        "{\"schemaVersion\":1.0,\"frameId\":\"world\",\"unit\":\"m\",\"controlBounds\":null,"
        "\"groundZ\":null}");
    expectInvalid(
        "{\"schemaVersion\":1,\"frameId\":\"map\",\"unit\":\"m\",\"controlBounds\":null,"
        "\"groundZ\":null}");
    expectInvalid(
        "{\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"cm\",\"controlBounds\":null,"
        "\"groundZ\":null}");
}

void testInvalidEndpointsThrow() {
    const char* prefix =
        "{\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"m\",\"controlBounds\":{";
    expectInvalid(std::string(prefix) +
        "\"xMin\":-1,\"xMax\":-1,\"yMin\":-1,\"yMax\":1,\"zMin\":0,\"zMax\":2},\"groundZ\":null}");
    expectInvalid(std::string(prefix) +
        "\"xMin\":2,\"xMax\":1,\"yMin\":-1,\"yMax\":1,\"zMin\":0,\"zMax\":2},\"groundZ\":null}");
    expectInvalid(std::string(prefix) +
        "\"xMin\":-1,\"xMax\":1,\"yMin\":1,\"yMax\":1,\"zMin\":0,\"zMax\":2},\"groundZ\":null}");
    expectInvalid(std::string(prefix) +
        "\"xMin\":-1,\"xMax\":1,\"yMin\":-1,\"yMax\":1,\"zMin\":3,\"zMax\":2},\"groundZ\":null}");
    expectInvalid(std::string(prefix) +       // missing zMax
        "\"xMin\":-1,\"xMax\":1,\"yMin\":-1,\"yMax\":1,\"zMin\":0},\"groundZ\":null}");
    expectInvalid(std::string(prefix) +       // non-finite overflow endpoint
        "\"xMin\":-1,\"xMax\":1e999,\"yMin\":-1,\"yMax\":1,\"zMin\":0,\"zMax\":2},\"groundZ\":null}");
    expectInvalid(std::string(prefix) +       // NaN literal is not JSON
        "\"xMin\":NaN,\"xMax\":1,\"yMin\":-1,\"yMax\":1,\"zMin\":0,\"zMax\":2},\"groundZ\":null}");
    expectInvalid(std::string(prefix) +       // bool is not a number
        "\"xMin\":true,\"xMax\":1,\"yMin\":-1,\"yMax\":1,\"zMin\":0,\"zMax\":2},\"groundZ\":null}");
    expectInvalid(                             // groundZ must be a finite number
        "{\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"m\",\"controlBounds\":null,"
        "\"groundZ\":\"0.0\"}");
    expectInvalid(
        "{\"schemaVersion\":1,\"frameId\":\"world\",\"unit\":\"m\",\"controlBounds\":null,"
        "\"groundZ\":-1e999}");
}

void testLoadFactKeepsClockStringsAndExactReadback() {
    const auto boundary = parseWorldBoundary(kFull);
    const std::string wire = controllerLoadFactJSON(boundary, makeFact());
    const Json::Value fact = reparsed(wire);
    assert(fact["schemaVersion"].asInt() == 1);
    assert(fact["source"]["kind"].asString() == "multirotor-controller");
    assert(fact["source"]["id"].asString() == "/uav3/px4_multirotor_controller");
    assert(fact["source"]["instanceId"].asString() ==
           "0f4b2c3a-1111-4222-8333-9abcdef01234");
    assert(fact["sequence"].asUInt() == 1);
    assert(fact["event"].asString() == "applied");
    const auto& clocks = fact["effectiveAt"];
    assert(clocks["rosTimeNs"].isString());
    assert(clocks["rosTimeNs"].asString() == "9223372036854775807");
    assert(clocks["unixTimeNs"].asString() == "18446744073709551615");
    assert(clocks["monotonicTimeNs"].asString() == "12345678901234567");
    const auto& values = fact["values"];
    assert(values["fenceEnabled"].asBool());
    assert(values["positionDistanceLimitMetres"].asDouble() == 1.0);
    assert(values["canonicalPoseTopic"].asString() == "/uav3/pose");
    assert(values["localPoseTopic"].asString() == "/uav3/mavros/local_position/pose");
    assert(values["worldPoseTopic"].asString() == "/uav3/alg/state_estimator/state");
    assert(values["trackingBackend"].asString() == "dfbc");
    const auto& readback = values["worldBoundary"];
    assert(readback["schemaVersion"].asInt() == 1);
    assert(readback["frameId"].asString() == "world");
    assert(readback["unit"].asString() == "m");
    const auto& bounds = readback["controlBounds"];
    // Readback is the exact producer-applied boundary: no offset, no rounding.
    assert(bounds["xMin"].asDouble() == -4.5 && bounds["xMax"].asDouble() == 4.5);
    assert(bounds["yMin"].asDouble() == -3.25 && bounds["yMax"].asDouble() == 3.25);
    assert(bounds["zMin"].asDouble() == 0.0 && bounds["zMax"].asDouble() == 2.5);
    assert(readback["groundZ"].asDouble() == 0.125);
    assert(fact["provenance"]["boundary"].asString() == "controller-setConfig-returned");
    assert(fact["provenance"]["parameter"].asString() == "~world_boundary_json");
}

void testLoadFactWithUnsetBoundary() {
    const std::string wire = controllerLoadFactJSON(std::nullopt, makeFact());
    const Json::Value fact = reparsed(wire);
    assert(fact["values"]["worldBoundary"].isNull());
    assert(!fact["values"]["fenceEnabled"].asBool());
    assert(fact["effectiveAt"]["rosTimeNs"].asString() == "9223372036854775807");
}

int main() {
    testNullAndWhitespaceNullAreUnset();
    testCompleteBoundsExactValuesNoOffset();
    testNullMembers();
    testInvalidContractsThrowBeforeSetConfig();
    testInvalidEndpointsThrow();
    testLoadFactKeepsClockStringsAndExactReadback();
    testLoadFactWithUnsetBoundary();
    std::cout << "7 production worldBoundary parser/load-fact cases passed\n";
}
'''
MONITOR_TEST = r'''
#include <cassert>
#include <iostream>
#include "px4_multirotor_controller/uav/state_machine/health_monitor_state.h"
using namespace px4_multirotor_controller;
struct Fixture {
    DroneController controller;
    HealthMonitorState health{controller};
    state_machine::StateContext context;
    void poseAt(double x, double y, double z) {
        controller.sensor.local_x = x;
        controller.sensor.local_y = y;
        controller.sensor.local_z = z;
        controller.sensor.local_pos_stats.is_new = true;
        health.tick(context);
        controller.sensor.local_pos_stats.is_new = false;
        controller.sensor.vrpn_pose_stats.is_new = false;
        controller.sensor.uav_state_estimate_stats.is_new = false;
        controller.sensor.imu_stats.is_new = false;
    }
    void setBounds(double x_min, double x_max, double y_min, double y_max,
                   double z_min, double z_max) {
        WorldBoundary boundary;
        boundary.control_bounds =
            WorldControlBounds{x_min, x_max, y_min, y_max, z_min, z_max};
        controller.config.safety.world_boundary = boundary;
    }
    void setBoundsNull() {
        controller.config.safety.world_boundary = WorldBoundary{};
    }
    int geofenceEvents() const {
        int count = 0;
        for (const auto& event : context.internals) {
            if (event.id == event_type::SAFE_GEOFENCE_VIOLATION) {
                ++count;
            }
        }
        return count;
    }
};
void testUnsetBoundaryDisablesGeofence() {
    Fixture f;
    f.poseAt(1e6, -1e6, 1e6);
    f.poseAt(0, 0, 0);
    assert(f.geofenceEvents() == 0);
    assert(f.context.internals.empty());
}
void testNullControlBoundsDisablesGeofence() {
    Fixture f;
    f.setBoundsNull();
    f.poseAt(1e6, 1e6, 1e6);
    assert(f.geofenceEvents() == 0);
}
void testBoundaryEdgeIsClosedAndEntryPostsOnce() {
    Fixture f;
    f.setBounds(-1.0, 1.0, -1.0, 1.0, 0.0, 2.0);
    f.poseAt(0, 0, 1);
    assert(f.geofenceEvents() == 0);
    // Exact endpoints are inside: violation is strict outside, never >= / <=.
    f.poseAt(1.0, -1.0, 2.0);
    f.poseAt(-1.0, 1.0, 0.0);
    assert(f.geofenceEvents() == 0);
    f.poseAt(1.0 + 1e-9, 0, 1);
    assert(f.geofenceEvents() == 1);
    f.poseAt(2, 0, 1);
    assert(f.geofenceEvents() == 1);   // staying outside reposts nothing
    f.poseAt(0, 0, 1);
    assert(f.geofenceEvents() == 1);   // recovery posts nothing
    f.poseAt(0, 0, -1e-9);
    assert(f.geofenceEvents() == 2);   // z below min is a fresh entry
    f.poseAt(0, 0, 1);
    assert(f.geofenceEvents() == 2);
}
int main() {
    testUnsetBoundaryDisablesGeofence();
    testNullControlBoundsDisablesGeofence();
    testBoundaryEdgeIsClosedAndEntryPostsOnce();
    std::cout << "3 production monitor boundary cases passed\n";
}
'''


def pkg_config(*args):
    tool = shutil.which("pkg-config")
    assert tool is not None, "pkg-config is required; do not silently skip"
    return shlex.split(subprocess.run(
        [tool, *args, "jsoncpp"], check=True, capture_output=True, text=True,
        timeout=30).stdout.strip())


class WorldBoundaryTest(unittest.TestCase):
    def compile_and_run(self, root, sources, compile_flags, link_flags, name):
        compiler = shutil.which(os.environ.get("CXX", "g++"))
        self.assertIsNotNone(compiler, "A C++17 compiler is required; do not silently skip")
        binary = root / name
        subprocess.run([
            compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
            *shlex.split(os.environ.get("CXXFLAGS", "")),
            "-I" + str(root), "-I" + str(PACKAGE / "include"),
            *compile_flags, *map(str, sources), *link_flags, "-o", str(binary),
        ], check=True, timeout=60)
        subprocess.run([str(binary)], check=True, timeout=30)

    def write_doubles(self, root):
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

    def test_parser_and_load_fact(self):
        with tempfile.TemporaryDirectory(prefix="world-boundary-parser-") as directory:
            root = Path(directory)
            (root / "parser_test.cpp").write_text(PARSER_TEST)
            includes = []
            for flag in pkg_config("--cflags-only-I"):
                includes += ["-isystem", flag[2:]] if flag.startswith("-I") else [flag]
            compile_flags = includes + pkg_config("--cflags-only-other")
            self.compile_and_run(root, [PACKAGE / "src/world_boundary.cpp",
                                        root / "parser_test.cpp"],
                                 compile_flags, pkg_config("--libs"),
                                 "world-boundary-parser-test")

    def test_monitor_boundary_edge(self):
        with tempfile.TemporaryDirectory(prefix="world-boundary-monitor-") as directory:
            root = Path(directory)
            self.write_doubles(root)
            (root / "monitor_test.cpp").write_text(MONITOR_TEST)
            self.compile_and_run(
                root, [PACKAGE / "src/uav/state_machine/health_monitor_state.cpp",
                       root / "monitor_test.cpp"],
                [], [], "world-boundary-monitor-test")

    def test_launch_requires_explicit_boundary_and_configs_drop_fence(self):
        root = ET.parse(PACKAGE / "launch/uav_nmpc_controller.launch").getroot()
        declared = {arg.get("name"): arg for arg in root.findall("arg")}
        self.assertIn("world_boundary_json", declared)
        self.assertIsNone(declared["world_boundary_json"].get("default"),
                          "world_boundary_json must stay a required launch arg")
        params = {param.get("name"): param for param in root.findall(".//param")}
        boundary = params.get("world_boundary_json")
        self.assertIsNotNone(boundary)
        self.assertEqual(boundary.get("type"), "str")
        self.assertEqual(boundary.get("value"), "$(arg world_boundary_json)")
        for name in ["px4_local_1m.yaml", "uav_nmpc.yaml"]:
            self.assertNotIn("fence_", (PACKAGE / "config" / name).read_text(), name)


if __name__ == "__main__":
    unittest.main()
