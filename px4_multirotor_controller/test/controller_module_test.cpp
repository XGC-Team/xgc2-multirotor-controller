// The px4_multirotor_controller module, loaded as a shared library and driven through the module
// ABI by an in-test host and a scripted vehicle (vehicle_sim.h).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <module_support/json_config.hpp>
#include <module_support/test_host.hpp>
#include <stdexcept>
#include <string>
#include <thread>

#include "module_config.h"
#include "multirotor_reference_trajectory/payloads.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/payloads.h"
#include "vehicle_sim.h"

namespace {

using module_support::JsonConfig;
using module_support::ModuleLibrary;
using module_support::TestHost;
using vehicle_sim::kMs;
using vehicle_sim::Vehicle;
namespace pmc = px4_multirotor_controller;
namespace conv = px4_multirotor_controller::module;

// A configuration object: the mandatory world boundary (none) plus `extra` members.
std::string config(const std::string& extra = "") {
    return "{\"world_boundary_json\": \"null\"" + (extra.empty() ? "" : ", " + extra) + "}";
}

// The takeoff gate that disarms and waits for ALTCTL is the ROS parameter skip_takeoff_init_disarm;
// the scripted vehicle takes off from OFFBOARD directly.
const char* kFlightConfig = "\"skip_takeoff_init_disarm\": true, \"takeoff_altitude\": 1.5";

class ControllerModuleTest : public ::testing::Test {
   protected:
    void SetUp() override {
        library_ = std::make_unique<ModuleLibrary>(CONTROLLER_MODULE_PATH);
    }

    void TearDown() override {
        if (!HasFailure() || !host_)
            return;
        for (const auto& entry : host_->logs()) {
            if (entry.first >= 2) {
                std::fprintf(stderr, "module log [%d] %s\n", entry.first, entry.second.c_str());
            }
        }
    }

    void open(const std::string& json) {
        host_ = std::make_unique<TestHost>(library_->desc());
        host_->setNow(vehicle_sim::kStart);
        ASSERT_EQ(host_->create(json), XGC2_OK);
        ASSERT_EQ(host_->start(), XGC2_OK);
        vehicle_ = std::make_unique<Vehicle>(*host_);
    }

    // Runs the scripted vehicle until the controller reports `state`.
    bool reach(const std::string& state, double limit) {
        return vehicle_->runUntil([&] { return vehicle_->state() == state; }, limit);
    }

    std::unique_ptr<ModuleLibrary> library_;
    std::unique_ptr<TestHost> host_;
    std::unique_ptr<Vehicle> vehicle_;
};

TEST_F(ControllerModuleTest, DescriptorDeclaresTheAbiAndTheTypedPorts) {
    const xgc2_module_desc* desc = library_->desc();
    ASSERT_NE(desc, nullptr);
    EXPECT_EQ(desc->abi_major, XGC2_MODULE_ABI_MAJOR);
    EXPECT_EQ(desc->abi_minor, XGC2_MODULE_ABI_MINOR);
    EXPECT_STREQ(desc->name, "px4_multirotor_controller");
    EXPECT_GT(std::strlen(desc->version), 0U);

    struct Expected {
        const char* name;
        uint32_t direction;
        uint32_t kind;
        const char* schema;
        uint32_t size;
        uint32_t align;
        uint32_t depth;  // events only
        uint32_t flags;
    };
    const auto in = XGC2_PORT_IN, out = XGC2_PORT_OUT;
    const auto state = XGC2_PORT_STATE, event = XGC2_PORT_EVENT;
#define PORT(name, dir, kind, schema, type, depth, flags) \
    { name, dir, kind, schema, sizeof(type), alignof(type), depth, flags }
    const Expected expected[] = {
        PORT("state_estimate", in, event, "xgc2.px4.state_estimate.v1", xgc2_px4_state_estimate_v1,
             8, 0),
        PORT("local_pose", in, event, "xgc2.px4.pose.v1", xgc2_px4_pose_v1, 8, XGC2_PORT_REQUIRED),
        PORT("local_velocity", in, event, "xgc2.px4.velocity.v1", xgc2_px4_velocity_v1, 8,
             XGC2_PORT_REQUIRED),
        PORT("imu", in, event, "xgc2.px4.imu.v1", xgc2_px4_imu_v1, 16, XGC2_PORT_REQUIRED),
        PORT("fcu_state", in, event, "xgc2.px4.fcu_state.v1", xgc2_px4_fcu_state_v1, 8,
             XGC2_PORT_REQUIRED),
        PORT("battery", in, event, "xgc2.px4.battery.v1", xgc2_px4_battery_v1, 4, 0),
        PORT("vrpn_pose", in, event, "xgc2.px4.pose.v1", xgc2_px4_pose_v1, 8, XGC2_PORT_REQUIRED),
        PORT("command", in, event, "xgc2.px4.command.v1", xgc2_px4_command_v1, 8, 0),
        PORT("alg_setpoint", in, event, "xgc2.px4.position_target.v1", xgc2_px4_position_target_v1,
             8, 0),
        PORT("hover_thrust", in, event, "xgc2.px4.hover_thrust.v1", xgc2_px4_hover_thrust_v1, 8, 0),
        PORT("ref_active_analytic", in, state, "xgc2.px4.reference_analytic.v1",
             xgc2_px4_reference_analytic_v1, 0, 0),
        PORT("ref_active_sampled", in, state, "xgc2.px4.reference_sampled.v1",
             xgc2_px4_reference_sampled_v1, 0, 0),
        PORT("setpoint", out, state, "xgc2.px4.position_target.v1", xgc2_px4_position_target_v1, 0,
             0),
        PORT("attitude_rate", out, state, "xgc2.px4.attitude_rate_target.v1",
             xgc2_px4_attitude_rate_target_v1, 0, 0),
        PORT("fcu_request", out, event, "xgc2.px4.fcu_request.v1", xgc2_px4_fcu_request_v1, 8, 0),
        PORT("status", out, state, "xgc2.px4.controller_status.v1", xgc2_px4_controller_status_v1,
             0, 0),
        PORT("ref_request", out, event, "xgc2.px4.reference_analytic.v1",
             xgc2_px4_reference_analytic_v1, 4, 0),
    };
#undef PORT
    ASSERT_EQ(desc->port_count, sizeof expected / sizeof expected[0]);
    ASSERT_LE(desc->port_count, XGC2_MODULE_MAX_PORTS);
    bool outputs_started = false;
    for (uint32_t i = 0; i < desc->port_count; ++i) {
        const xgc2_port_desc& p = desc->ports[i];
        EXPECT_STREQ(p.name, expected[i].name);
        EXPECT_EQ(p.direction, expected[i].direction) << p.name;
        EXPECT_EQ(p.kind, expected[i].kind) << p.name;
        EXPECT_STREQ(p.schema_id, expected[i].schema) << p.name;
        EXPECT_EQ(p.size, expected[i].size) << p.name;
        EXPECT_EQ(p.align, expected[i].align) << p.name;
        EXPECT_EQ(p.queue_depth, expected[i].depth) << p.name;
        EXPECT_EQ(p.flags, expected[i].flags) << p.name;
        // The module numbers changed_inputs by the port index: every input precedes every output.
        outputs_started = outputs_started || p.direction == XGC2_PORT_OUT;
        EXPECT_FALSE(outputs_started && p.direction == XGC2_PORT_IN) << p.name;
    }
}

TEST(ControllerModuleConfig, KeysAreReadLikeTheRosParameters) {
    const std::string text = config(
        "\"takeoff_altitude\": 1.75, \"skip_takeoff_init_disarm\": true, "
        "\"tracking_backend\": \"dfbc\", \"local_type_mask\": 3135, \"max_velocity_xy\": 7.5, "
        "\"smc\": {\"k1\": 0.7, \"k2\": 0.8, \"boundary_layer\": 0.02}, "
        "\"dfbc\": {\"position_natural_frequency\": [1.0, 1.5, 2.0], \"tilt_gain\": 7.0, "
        "\"use_body_rate_feedforward\": false}, "
        "\"nmpc\": {\"control_period\": 0.02, \"reference_analytic_type\": 3, "
        "\"reference_radius\": 2.5}, \"hover_thrust\": {\"enabled\": true}");
    const xgc2_config raw{text.c_str(), text.size()};
    JsonConfig json(&raw);
    const pmc::ControllerConfig c = conv::readConfig(json, [](bool, const std::string&) {});
    EXPECT_DOUBLE_EQ(c.takeoff_altitude, 1.75);
    EXPECT_TRUE(c.skip_takeoff_init_disarm);
    EXPECT_EQ(c.tracking_backend, pmc::TrackingBackend::DFBC);
    EXPECT_EQ(c.local_type_mask, 3135U);
    EXPECT_DOUBLE_EQ(c.safety.max_velocity_xy, 7.5);
    EXPECT_DOUBLE_EQ(c.smc.k1, 0.7);
    EXPECT_DOUBLE_EQ(c.smc.k2, 0.8);
    EXPECT_DOUBLE_EQ(c.smc.boundary_layer, 0.02);
    EXPECT_DOUBLE_EQ(c.dfbc.position_natural_frequency.y(), 1.5);
    EXPECT_DOUBLE_EQ(c.dfbc.tilt_gain, 7.0);
    EXPECT_FALSE(c.dfbc.use_body_rate_feedforward);
    EXPECT_DOUBLE_EQ(c.nmpc.control_period, 0.02);
    EXPECT_EQ(c.nmpc.reference_analytic_type, 3);
    EXPECT_DOUBLE_EQ(c.nmpc.reference_radius, 2.5);
    EXPECT_FALSE(c.safety.world_boundary.has_value());
}

TEST(ControllerModuleConfig, AbsentKeysKeepTheValuesOfTheOwningProfile) {
    const std::string text = config();
    const xgc2_config raw{text.c_str(), text.size()};
    JsonConfig json(&raw);
    const pmc::ControllerConfig c = conv::readConfig(json, [](bool, const std::string&) {});
    // config/uav_nmpc.yaml, not the C++ defaults of ControllerConfig (1.5 m, 3.0 m/s, ...).
    EXPECT_DOUBLE_EQ(c.takeoff_altitude, 2.3);
    EXPECT_EQ(c.tracking_backend, pmc::TrackingBackend::PX4_LOCAL);
    EXPECT_EQ(c.local_type_mask, 3072U);
    EXPECT_DOUBLE_EQ(c.safety.position_jump_threshold, 100.0);
    EXPECT_TRUE(c.dfbc.acceleration_correction_enabled);
    EXPECT_TRUE(c.nmpc.hover_thrust_enabled);
}

TEST(ControllerModuleConfig, WorldBoundaryIsReadAsTheNodeReadsIt) {
    const std::string boundary =
        "{\\\"schemaVersion\\\":1,\\\"frameId\\\":\\\"world\\\",\\\"unit\\\":\\\"m\\\","
        "\\\"groundZ\\\":0.0,\\\"controlBounds\\\":{\\\"xMin\\\":-5,\\\"xMax\\\":5,\\\"yMin\\\":-4,"
        "\\\"yMax\\\":4,\\\"zMin\\\":0,\\\"zMax\\\":6}}";
    const std::string text = "{\"world_boundary_json\": \"" + boundary + "\"}";
    const xgc2_config raw{text.c_str(), text.size()};
    JsonConfig json(&raw);
    const pmc::ControllerConfig c = conv::readConfig(json, [](bool, const std::string&) {});
    ASSERT_TRUE(c.safety.world_boundary.has_value());
    ASSERT_TRUE(c.safety.world_boundary->control_bounds.has_value());
    EXPECT_TRUE(c.safety.world_boundary->control_bounds->outside(6.0, 0.0, 1.0));
    EXPECT_FALSE(c.safety.world_boundary->control_bounds->outside(1.0, 1.0, 1.0));
}

TEST_F(ControllerModuleTest, RefusesAnInvalidConfiguration) {
    struct Bad {
        std::string json;
        std::string message;
    };
    const Bad bad[] = {
        {"{}", "world_boundary_json must explicitly contain"},  // the geofence is never defaulted
        {"[]", "not a JSON object"},
        {"{", "not a JSON object"},
        {config("\"takeoff_altitude\": \"high\""), "'takeoff_altitude' must be a number"},
        {config("\"local_type_mask\": 3.5"), "'local_type_mask' must be an integer"},
        {config("\"skip_takeoff_init_disarm\": 1"), "'skip_takeoff_init_disarm' must be a boolean"},
        {config("\"dfbc\": {\"position_natural_frequency\": [1.0, \"x\"]}"),
         "must be an array of numbers"},
        {config("\"takeof_altitude\": 2.0"), "unknown configuration key 'takeof_altitude'"},
        {config("\"nmpc\": {\"control_perod\": 0.01}"),
         "unknown configuration key 'nmpc/control_perod'"},
        {"{\"world_boundary_json\": \"{}\"}", "world_boundary_json"},
    };
    for (const Bad& b : bad) {
        TestHost host(library_->desc());
        EXPECT_EQ(host.create(b.json), XGC2_ERR_INVALID) << b.json;
        EXPECT_EQ(host.instance(), nullptr) << b.json;
        EXPECT_TRUE(host.logged(3, b.message)) << b.json;
    }
}

TEST_F(ControllerModuleTest, StartAsksForTheControlPeriodAndReportsTheFlightState) {
    open(config());
    EXPECT_EQ(host_->periodNs(), 1000000);  // the node's 1 kHz loop
    EXPECT_EQ(host_->lastReportDetail(), "SelfCheck");
    EXPECT_EQ(host_->reports().front().first, 0);
}

TEST_F(ControllerModuleTest, BecomesReadyWithTheSensorsAndLeavesReadyWithoutThem) {
    open(config());
    ASSERT_TRUE(reach("Ready", 3.0));
    // The receive statistics use the time the samples were received: when they stop, the sensors
    // time out (the node's 2.5 s) and the controller leaves Ready.
    vehicle_->setSensorsOff(true);
    EXPECT_FALSE(vehicle_->runUntil([&] { return vehicle_->state() != "Ready"; }, 1.5));
    EXPECT_TRUE(vehicle_->runUntil([&] { return vehicle_->state() != "Ready"; }, 3.0));
    EXPECT_FALSE(vehicle_->failed);
}

TEST_F(ControllerModuleTest, TakeoffRequestsOffboardAndArmingAndStreamsSetpoints) {
    open(config(kFlightConfig));
    ASSERT_TRUE(reach("Ready", 3.0));
    xgc2_px4_command_v1 command{};
    std::strcpy(command.text, "takeoff");
    ASSERT_TRUE(host_->push("command", command, vehicle_->now()));
    ASSERT_TRUE(reach("Hover", 20.0)) << "state " << vehicle_->state();

    const std::vector<std::string> expected_states = {
        "SelfCheck",        "Ready", "TakeoffInit", "TakeoffOffboardRequest", "TakeoffArmRequest",
        "TakeoffAscending", "Hover"};
    EXPECT_EQ(vehicle_->states, expected_states);
    ASSERT_GE(vehicle_->requests.size(), 2U);
    bool offboard = false, arm = false;
    for (const auto& r : vehicle_->requests) {
        if (r.kind == XGC2_PX4_FCU_REQUEST_MODE && std::strcmp(r.mode, "OFFBOARD") == 0)
            offboard = true;
        if (r.kind == XGC2_PX4_FCU_REQUEST_ARM && r.arm == 1)
            arm = true;
    }
    EXPECT_TRUE(offboard);
    EXPECT_TRUE(arm);
    EXPECT_GT(vehicle_->setpoints, 100U);
    EXPECT_NEAR(vehicle_->last_setpoint.position[2], 1.5, 1e-9);
    EXPECT_EQ(vehicle_->last_setpoint.type_mask, pmc::kHoverPositionVelocityTypeMask);  // Hover
    EXPECT_NEAR(vehicle_->altitude(), 1.5, 0.1);
    EXPECT_FALSE(vehicle_->failed);
    // Every output is stamped with the step's time.
    EXPECT_GE(vehicle_->last_setpoint.stamp_sec, 100U);
}

TEST_F(ControllerModuleTest, CommandsAreMappedAsTheRosNodeMapsThem) {
    open(config(kFlightConfig));
    ASSERT_TRUE(reach("Ready", 3.0));
    xgc2_px4_command_v1 unknown{};
    std::strcpy(unknown.text, "dance");
    ASSERT_TRUE(host_->push("command", unknown, vehicle_->now()));
    vehicle_->run(0.01);
    EXPECT_TRUE(host_->logged(2, "unknown command: dance"));
    EXPECT_EQ(vehicle_->state(), "Ready");
    xgc2_px4_command_v1 alias{};
    std::strcpy(alias.text, "TAKEOFF");
    ASSERT_TRUE(host_->push("command", alias, vehicle_->now()));
    EXPECT_TRUE(reach("TakeoffInit", 1.0));
}

TEST_F(ControllerModuleTest, ConfigureOnTheGroundTakesEffectAndIsRefusedInFlight) {
    open(config());
    ASSERT_TRUE(reach("Ready", 3.0));
    ASSERT_EQ(host_->configure(config(kFlightConfig)), XGC2_OK);
    // The new takeoff altitude is the setpoint of the next takeoff.
    xgc2_px4_command_v1 command{};
    std::strcpy(command.text, "takeoff");
    ASSERT_TRUE(host_->push("command", command, vehicle_->now()));
    ASSERT_TRUE(reach("TakeoffInit", 1.0));
    vehicle_->run(0.1);
    EXPECT_NEAR(vehicle_->last_setpoint.position[2], 1.5, 1e-9);
    // Flying: refused, and nothing changes.
    EXPECT_EQ(host_->configure(config("\"takeoff_altitude\": 4.0")), XGC2_ERR_STATE);
    EXPECT_TRUE(host_->logged(2, "configuration refused"));
    // An invalid configuration is refused first, wherever the controller is.
    EXPECT_EQ(host_->configure("{}"), XGC2_ERR_INVALID);
}

TEST_F(ControllerModuleTest, StopTearsTheControllerDownAndStartBeginsFresh) {
    open(config(kFlightConfig));
    ASSERT_TRUE(reach("Ready", 3.0));
    ASSERT_EQ(host_->stop(), XGC2_OK);
    host_->setNow(host_->now() + kMs);
    EXPECT_EQ(host_->step(), XGC2_ERR_STATE);
    ASSERT_EQ(host_->start(), XGC2_OK);
    EXPECT_EQ(host_->lastReportDetail(), "SelfCheck");  // no history: sensors must be seen again
    EXPECT_EQ(host_->start(), XGC2_ERR_STATE);          // already started
    EXPECT_EQ(host_->stop(), XGC2_OK);
    EXPECT_EQ(host_->stop(), XGC2_OK);  // stopping twice is harmless
}

TEST_F(ControllerModuleTest, AFullFcuRequestQueueDegradesTheHealth) {
    open(config(kFlightConfig));
    ASSERT_TRUE(reach("Ready", 3.0));
    host_->limitOutput("fcu_request", 0);
    xgc2_px4_command_v1 command{};
    std::strcpy(command.text, "takeoff");
    ASSERT_TRUE(host_->push("command", command, vehicle_->now()));
    ASSERT_TRUE(vehicle_->runUntil([&] { return host_->refusedWrites("fcu_request") > 0; }, 10.0));
    vehicle_->run(0.01);
    EXPECT_EQ(host_->reports().back().first, 1);
    EXPECT_TRUE(host_->logged(3, "output fcu_request is not writable"));
}

// DFBC and NMPC take the fused estimate, the hover thrust estimate and a reference trajectory. The
// scripted vehicle hovers; the controller's reference activation request goes to the reference
// module and its active reference comes back, as the host's channels carry them.
class ControllerChainTest : public ControllerModuleTest {
   protected:
    void SetUp() override {
        ControllerModuleTest::SetUp();
        reference_library_ = std::make_unique<ModuleLibrary>(referenceModulePath());
    }

    // The reference module is built by the other package of this repository: look for it in the
    // devel space (the merged one of catkin build, the single one of catkin_make).
    static std::string referenceModulePath() {
        std::string candidates = REFERENCE_MODULE_DIRECTORIES;
        size_t begin = 0;
        while (begin <= candidates.size()) {
            const size_t end = std::min(candidates.find(':', begin), candidates.size());
            const std::string path =
                candidates.substr(begin, end - begin) + "/libpx4_multirotor_reference_module.so";
            if (std::ifstream(path).good())
                return path;
            begin = end + 1;
        }
        throw std::runtime_error("libpx4_multirotor_reference_module.so is not built in " +
                                 candidates);
    }

    void openChain(const std::string& backend) {
        open(config(std::string(kFlightConfig) + ", \"tracking_backend\": \"" + backend + "\""));
        reference_ = std::make_unique<TestHost>(reference_library_->desc());
        reference_->setNow(vehicle_sim::kStart);
        ASSERT_EQ(reference_->create("{}"), XGC2_OK);
        ASSERT_EQ(reference_->start(), XGC2_OK);
        vehicle_->attachReference(reference_.get());
    }

    // To Hover, then the operator's custom1 command.
    void takeOffAndTrack() {
        ASSERT_TRUE(reach("Ready", 3.0));
        xgc2_px4_command_v1 command{};
        std::strcpy(command.text, "takeoff");
        ASSERT_TRUE(host_->push("command", command, vehicle_->now()));
        ASSERT_TRUE(reach("Hover", 20.0)) << vehicle_->state();
        std::strcpy(command.text, "custom1");
        ASSERT_TRUE(host_->push("command", command, vehicle_->now()));
        ASSERT_TRUE(reach("Custom1", 1.0));
    }

    std::unique_ptr<ModuleLibrary> reference_library_;
    std::unique_ptr<TestHost> reference_;
};

TEST_F(ControllerChainTest, Custom1RequestsAReferenceAndDfbcTracksIt) {
    openChain("dfbc");
    takeOffAndTrack();
    // Custom1 asks the reference generator for a trajectory that starts at the estimate ...
    ASSERT_TRUE(vehicle_->runUntil([&] { return !vehicle_->reference_requests.empty(); }, 1.0));
    const auto& request = vehicle_->reference_requests.front();
    EXPECT_STREQ(request.header.frame_id, "map");
    EXPECT_EQ(request.request_id, 1U);
    EXPECT_NEAR(request.origin_position[2], vehicle_->altitude(), 1e-9);
    // ... which the reference module accepts and publishes as the active reference, and the
    // controller then streams attitude-rate commands for it.
    ASSERT_TRUE(vehicle_->runUntil([&] { return vehicle_->attitude.size() > 20; }, 5.0));
    for (const auto& a : vehicle_->attitude) {
        EXPECT_TRUE(std::isfinite(a.thrust));
        EXPECT_GE(a.thrust, 0.0);
        EXPECT_LE(a.thrust, 1.0);
        EXPECT_TRUE(std::isfinite(a.body_rate[0]) && std::isfinite(a.body_rate[1]) &&
                    std::isfinite(a.body_rate[2]));
    }
    EXPECT_FALSE(vehicle_->failed);
}

TEST_F(ControllerChainTest, NmpcResultWakesTheHostAndIsConsumedWithoutWaitingForThePeriod) {
    openChain("nmpc");
    takeOffAndTrack();

    // Step until the solver worker reports a result: the module wakes the host from the worker's
    // thread, so the host can run the consuming step at once.
    bool woken = false;
    for (int i = 0; i < 3000 && !woken; ++i) {
        const uint64_t seen = host_->wakeCount();
        vehicle_->step();
        woken = host_->waitWake(seen, std::chrono::milliseconds(15));
    }
    ASSERT_TRUE(woken) << "state " << vehicle_->state();
    EXPECT_NE(host_->lastWakeThread(), std::this_thread::get_id());

    // The extra step runs at the same clock time (no period elapsed) and publishes the command.
    const size_t before = vehicle_->attitude.size();
    vehicle_->wakeStep();
    ASSERT_EQ(vehicle_->attitude.size(), before + 1);
    const auto& command = vehicle_->attitude.back();
    EXPECT_TRUE(std::isfinite(command.thrust));
    EXPECT_GE(command.thrust, 0.1);  // normalized_thrust_min of the profile
    EXPECT_LE(command.thrust, 0.9);  // normalized_thrust_max
    EXPECT_EQ(static_cast<int64_t>(command.stamp_sec) * 1000000000LL + command.stamp_nsec,
              vehicle_->now());
    EXPECT_FALSE(vehicle_->failed);

    // Stopping joins the worker: no wake after stop() returns.
    const uint64_t wakes = host_->wakeCount();
    ASSERT_EQ(host_->stop(), XGC2_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(host_->wakeCount(), wakes);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
