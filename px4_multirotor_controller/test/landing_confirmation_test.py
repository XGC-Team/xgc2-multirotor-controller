#!/usr/bin/env python3
"""Compile actual LandingState source/header with deterministic boundary doubles.

No MAVLink command, ROS transport, full FSM, or physical vehicle is exercised.
Run: python3 px4_multirotor_controller/test/landing_confirmation_test.py
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
SUPPORT = r'''
#pragma once
#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>
namespace state_machine {
struct ActionResult {};
struct Status { bool ok() const { return true; } };
struct EventTimestamp { double seconds; };
struct Event {
    uint32_t id;
    EventTimestamp timestamp;
    std::map<std::string, bool> payload;
    Event(uint32_t value, EventTimestamp stamp) : id(value), timestamp(stamp) {}
};
struct StateContext {
    double elapsed_seconds{0};
    std::vector<Event> outputs, internals;
    std::chrono::duration<double> elapsed(uint32_t) const {
        return std::chrono::duration<double>(elapsed_seconds);
    }
    Status emitOutput(Event event) { outputs.push_back(std::move(event)); return {}; }
    Status postInternalEvent(Event event) { internals.push_back(std::move(event)); return {}; }
};
struct State {
    virtual ~State() = default;
    virtual std::string name() const = 0;
    virtual ActionResult onEnter(StateContext&) = 0;
    virtual ActionResult onTick(StateContext&) = 0;
    virtual ActionResult onExit(StateContext&) = 0;
};
namespace runtime {
template<class Clock = void> struct Timer {
    void start() {}
    void stop() {}
    void reset() {}
    std::chrono::duration<double> elapsed() const { return std::chrono::duration<double>(1); }
};
}
}
namespace px4_multirotor_controller {
enum class TrackingBackend { PX4_LOCAL, NMPC, DFBC };
inline bool trackingUsesFusedEstimate(TrackingBackend backend) {
    return backend != TrackingBackend::PX4_LOCAL;
}
struct Stats { bool is_active{true}; bool is_new{true}; };
struct SensorData {
    Stats local_pos_stats, local_velocity_stats, uav_state_estimate_stats, state_stats;
    bool fcu_connected{true}, fcu_armed{true};
    double local_z{2}, local_vz{0}, z{2}, vz{0};
};
struct Config { TrackingBackend tracking_backend{TrackingBackend::PX4_LOCAL}; };
struct Setpoint {
    uint16_t type_mask{0};
    double x{0}, y{0}, z{0}, vx{0}, vy{0}, vz{0}, ax{0}, ay{0}, az{0};
    double qx{0}, qy{0}, qz{0}, qw{1}, yaw_rate{0};
};
namespace state_type { constexpr uint32_t Landing = 7; }
namespace event_type { constexpr uint32_t TOUCHDOWN = 10, LANDING_TIMEOUT = 11; }
namespace output_event_type {
constexpr uint32_t PUBLISH_SETPOINT = 20, REQUEST_ARMING = 21, REQUEST_KILL = 22;
}
inline bool shouldRunEvery(state_machine::runtime::Timer<>&, double, bool) { return true; }
struct DroneController {
    SensorData sensor;
    Config config;
    Setpoint setpoint;
    double now{1};
    std::vector<std::string> warnings;
    void clearCustom1Request() {}
    const SensorData& getSensorData() const { return sensor; }
    const Config& getConfig() const { return config; }
    Setpoint& getSetpoint() { return setpoint; }
    double getCurrentTime() const { return now; }
    template<class... Args> void logInfo(const char*, Args...) {}
    template<class... Args> void logWarn(const char* format, Args...) { warnings.emplace_back(format); }
};
namespace sensor_checks {
inline double worldZ(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) ? sensor.z : sensor.local_z;
}
inline double worldVz(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) ? sensor.vz : sensor.local_vz;
}
inline bool isWorldPoseNew(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) ? sensor.uav_state_estimate_stats.is_new
                                            : sensor.local_pos_stats.is_new;
}
inline bool isFcuArmed(const SensorData& sensor) { return sensor.fcu_armed; }
inline bool isFcuConnected(const SensorData& sensor) { return sensor.fcu_connected; }
}
}
'''
TEST = r'''
#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include "px4_multirotor_controller/uav/state_machine/landing_state.h"
using namespace px4_multirotor_controller;
struct TestLanding : LandingState {
    using LandingState::LandingState;
    using LandingState::onEnter;
    using LandingState::onTick;
    using LandingState::onExit;
};
size_t count(const std::vector<state_machine::Event>& events, uint32_t id) {
    return std::count_if(events.begin(), events.end(), [id](const auto& event) {return event.id == id;});
}
void tick(TestLanding& landing, DroneController& controller, state_machine::StateContext& ctx) {
    controller.now += .01;
    landing.onTick(ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_KILL) == 0);
}
void settle(TestLanding& landing, DroneController& controller, state_machine::StateContext& ctx) {
    controller.sensor.local_z = controller.sensor.z = .05;
    controller.sensor.local_vz = controller.sensor.vz = 0;
    for (int i = 0; i < 5; ++i) tick(landing, controller, ctx);
}
void testTimeoutAndLateDisarmConfirmation() {
    DroneController controller;
    TestLanding landing(controller);
    state_machine::StateContext ctx;
    landing.onEnter(ctx);
    ctx.elapsed_seconds = 100;
    tick(landing, controller, ctx);
    assert(count(ctx.internals, event_type::LANDING_TIMEOUT) == 0);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 0);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 0);
    assert(!controller.warnings.empty());
    const auto publications = count(ctx.outputs, output_event_type::PUBLISH_SETPOINT);
    tick(landing, controller, ctx);
    assert(count(ctx.outputs, output_event_type::PUBLISH_SETPOINT) > publications);
    assert(controller.setpoint.vz < 0);
    // Timeout must not stop subsequent touchdown observation.
    settle(landing, controller, ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 1);
    for (const auto& event : ctx.outputs) {
        if (event.id == output_event_type::REQUEST_ARMING) assert(!event.payload.at("arm"));
    }
    // A refused or uncompleted request leaves FCU armed; it is not success.
    for (int i = 0; i < 100; ++i) tick(landing, controller, ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 1);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 0);
    controller.sensor.fcu_armed = false;
    controller.sensor.state_stats.is_new = false;
    tick(landing, controller, ctx);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 0);
    controller.sensor.state_stats.is_new = true;
    controller.sensor.state_stats.is_active = false;
    tick(landing, controller, ctx);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 0);
    controller.sensor.state_stats.is_active = true;
    controller.sensor.fcu_connected = false;
    tick(landing, controller, ctx);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 0);
    controller.sensor.fcu_connected = true;
    tick(landing, controller, ctx);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 1);
    tick(landing, controller, ctx);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 1);
    landing.onExit(ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_KILL) == 0);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 1);
}
void testNoCompletionFromInvalidFeedback() {
    DroneController controller;
    TestLanding landing(controller);
    state_machine::StateContext ctx;
    landing.onEnter(ctx);
    controller.sensor.local_velocity_stats.is_active = false;
    settle(landing, controller, ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 0);
    controller.sensor.local_velocity_stats.is_active = true;
    controller.sensor.local_pos_stats.is_new = false;
    settle(landing, controller, ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 0);
    controller.sensor.local_pos_stats.is_new = true;
    controller.sensor.local_vz = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < 5; ++i) tick(landing, controller, ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 0);
    settle(landing, controller, ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 1);
    controller.sensor.local_pos_stats.is_active = false;
    controller.sensor.fcu_armed = false;
    tick(landing, controller, ctx);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 0);
    controller.sensor.local_pos_stats.is_active = true;
    settle(landing, controller, ctx);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 1);
}
void testFusedFeedbackAndReentry() {
    DroneController controller;
    controller.config.tracking_backend = TrackingBackend::NMPC;
    controller.sensor.local_pos_stats.is_active = false;
    TestLanding landing(controller);
    state_machine::StateContext ctx;
    landing.onEnter(ctx);
    settle(landing, controller, ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 1);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 0);
    controller.sensor.fcu_armed = false;
    tick(landing, controller, ctx);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 1);
    landing.onExit(ctx);
    ctx.outputs.clear(); ctx.internals.clear();
    controller.sensor.fcu_armed = true;
    landing.onEnter(ctx);
    settle(landing, controller, ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_ARMING) == 1);
    assert(count(ctx.internals, event_type::TOUCHDOWN) == 0);
    // Externally forced exit is not permission to kill or to claim touchdown.
    landing.onExit(ctx);
    assert(count(ctx.outputs, output_event_type::REQUEST_KILL) == 0);
}
int main() {
    testTimeoutAndLateDisarmConfirmation();
    testNoCompletionFromInvalidFeedback();
    testFusedFeedbackAndReentry();
    std::cout << "Landing timeout, normal-disarm confirmation, stale feedback, and reentry passed\n";
}
'''


class LandingConfirmationTest(unittest.TestCase):
    def test_production_landing_state(self):
        compiler = shutil.which(os.environ.get("CXX", "g++"))
        self.assertIsNotNone(compiler, "C++17 compiler is required")
        with tempfile.TemporaryDirectory(prefix="landing-confirmation-") as directory:
            root = Path(directory)
            (root / "support.hpp").write_text(SUPPORT)
            for name in (
                "state_machine/state_machine.hpp",
                "px4_multirotor_controller/common/types.h",
                "px4_multirotor_controller/state_machine/timing.h",
                "px4_multirotor_controller/common/sensor_checks.h",
                "px4_multirotor_controller/drone_controller.h",
            ):
                target = root / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text('#include "support.hpp"\n')
            main, binary = root / "test.cpp", root / "test"
            main.write_text(TEST)
            subprocess.run([
                compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
                "-I", str(root), "-I", str(PACKAGE / "include"),
                str(PACKAGE / "src/uav/state_machine/landing_state.cpp"), str(main),
                "-o", str(binary),
            ], check=True, timeout=30)
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == "__main__":
    unittest.main()
