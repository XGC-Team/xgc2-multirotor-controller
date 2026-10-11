// The px4_multirotor_controller module against the ROS node's behavior
// (replay/controller_node_path.h) on the same messages: the recorded software-plant flights
// (test/replay/data) and a scripted flight that reaches the DFBC and NMPC tracking the recordings
// do not.
//
// Each 1 ms tick delivers the messages received since the previous tick to the node path (the
// callbacks of ros::spinOnce()) and, mapped to payloads as the ROS edge does
// (ros_payload_mapping.h), to the module's ports with their receipt time, then updates both at the
// same nanosecond time. Everything the node's output consumers would send must come out of the
// module's ports, in the same order and bit for bit, and both must be in the same flight state
// after every tick.
//
// The NMPC request is solved inline by the node path and on the solver's worker thread by the
// module; the test waits for the module's wake after a tick that requested a solve, so the result
// arrives at the next update on both sides.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <module_support/test_host.hpp>
#include <set>
#include <string>
#include <vector>

#include "module_config.h"
#include "payload_conversion.h"
#include "replay/controller_node_path.h"
#include "replay/ros_payload_mapping.h"
#include "replay/scripted_flight.h"

namespace {

using controller_replay::OutputEvent;
using controller_replay::Record;
using module_support::ModuleLibrary;
using module_support::TestHost;
namespace pmc = px4_multirotor_controller;
namespace conv = px4_multirotor_controller::module;
namespace edge = ros_payload_mapping;

constexpr uint64_t kTickNs = 1000000;

// The ROS edge: a received message becomes a sample of the module's port.
void deliver(TestHost& host, const Record& r) {
    const int64_t receipt = static_cast<int64_t>(r.t_ns);
    using namespace controller_replay;
    switch (r.kind) {
        case kEstimate:
            ASSERT_TRUE(
                host.push("state_estimate",
                          edge::map(decode<rigid_state_estimator_msgs::RigidStateEstimate>(r.data)),
                          receipt));
            break;
        case kLocalPose:
            ASSERT_TRUE(host.push("local_pose",
                                  edge::map(decode<geometry_msgs::PoseStamped>(r.data)), receipt));
            break;
        case kLocalVelocity:
            ASSERT_TRUE(host.push("local_velocity",
                                  edge::map(decode<geometry_msgs::TwistStamped>(r.data)), receipt));
            break;
        case kImu:
            ASSERT_TRUE(host.push(
                "imu", edge::imuSample(decode<sensor_msgs::Imu>(r.data).header.stamp), receipt));
            break;
        case kFcuState:
            ASSERT_TRUE(
                host.push("fcu_state", edge::map(decode<mavros_msgs::State>(r.data)), receipt));
            break;
        case kBattery:
            ASSERT_TRUE(host.push("battery", edge::map(decode<sensor_msgs::BatteryState>(r.data)),
                                  receipt));
            break;
        case kVrpnPose:
            ASSERT_TRUE(host.push("vrpn_pose",
                                  edge::map(decode<geometry_msgs::PoseStamped>(r.data)), receipt));
            break;
        case kCommand:
            ASSERT_TRUE(host.push("command", edge::map(decode<std_msgs::String>(r.data)), receipt));
            break;
        case kAlgSetpoint:
            ASSERT_TRUE(host.push("alg_setpoint",
                                  edge::map(decode<mavros_msgs::PositionTarget>(r.data)), receipt));
            break;
        case kHoverThrust:
            ASSERT_TRUE(host.push(
                "hover_thrust",
                edge::map(decode<hover_thrust_estimator_msgs::HoverThrustEstimate>(r.data)),
                receipt));
            break;
        case kActiveAnalytic:
            ASSERT_TRUE(host.push(
                "ref_active_analytic",
                edge::map(decode<multirotor_reference_trajectory_msgs::AnalyticReference>(r.data)),
                receipt));
            break;
        case kActiveSampled: {
            const auto payload =
                edge::map(decode<multirotor_reference_trajectory_msgs::SampledReference>(r.data));
            ASSERT_TRUE(host.push("ref_active_sampled", *payload, receipt));
            break;
        }
        default:
            FAIL() << "unknown record kind " << static_cast<int>(r.kind);
    }
}

// What the node's output consumers send for an output event, as payloads.
struct Expected {
    std::vector<xgc2_px4_position_target_v1> setpoints;
    std::vector<xgc2_px4_attitude_rate_target_v1> attitudes;
    std::vector<xgc2_px4_fcu_request_v1> requests;
    std::vector<std::string> statuses;
    std::vector<pmc::reference::AnalyticReference> activations;
    size_t solves{0};
};

void stampOf(const pmc::Time& t, uint32_t& sec, uint32_t& nsec) {
    sec = t.sec;
    nsec = t.nsec;
}

Expected expectedOutputs(const std::vector<OutputEvent>& events, const pmc::Time& now) {
    Expected e;
    for (const OutputEvent& o : events) {
        switch (o.event.id) {
            case pmc::output_event_type::PUBLISH_SETPOINT: {
                xgc2_px4_position_target_v1 p;
                std::memset(&p, 0, sizeof p);
                stampOf(now, p.stamp_sec, p.stamp_nsec);
                const pmc::Setpoint& s = o.setpoint;
                p.position[0] = s.x;
                p.position[1] = s.y;
                p.position[2] = s.z;
                p.velocity[0] = s.vx;
                p.velocity[1] = s.vy;
                p.velocity[2] = s.vz;
                p.acceleration[0] = s.ax;
                p.acceleration[1] = s.ay;
                p.acceleration[2] = s.az;
                p.yaw = pmc::quaternionToYaw(s.qx, s.qy, s.qz, s.qw);
                p.yaw_rate = s.yaw_rate;
                p.type_mask = s.type_mask;
                p.coordinate_frame = s.coordinate_frame;
                e.setpoints.push_back(p);
                break;
            }
            case pmc::output_event_type::PUBLISH_ATTITUDE_RATE_TARGET: {
                xgc2_px4_attitude_rate_target_v1 p;
                std::memset(&p, 0, sizeof p);
                stampOf(now, p.stamp_sec, p.stamp_nsec);
                p.body_rate[0] = o.attitude.body_rate_x;
                p.body_rate[1] = o.attitude.body_rate_y;
                p.body_rate[2] = o.attitude.body_rate_z;
                p.thrust = pmc::clamp(o.attitude.thrust, 0.0, 1.0);
                e.attitudes.push_back(p);
                break;
            }
            case pmc::output_event_type::REQUEST_ARMING: {
                const auto it = o.event.payload.find("arm");
                if (it == o.event.payload.end() || !std::holds_alternative<bool>(it->second))
                    break;
                xgc2_px4_fcu_request_v1 p;
                std::memset(&p, 0, sizeof p);
                stampOf(now, p.stamp_sec, p.stamp_nsec);
                p.kind = XGC2_PX4_FCU_REQUEST_ARM;
                p.arm = std::get<bool>(it->second) ? 1U : 0U;
                e.requests.push_back(p);
                break;
            }
            case pmc::output_event_type::REQUEST_MODE: {
                const auto it = o.event.payload.find("mode");
                if (it == o.event.payload.end() || !std::holds_alternative<std::string>(it->second))
                    break;
                xgc2_px4_fcu_request_v1 p;
                std::memset(&p, 0, sizeof p);
                stampOf(now, p.stamp_sec, p.stamp_nsec);
                p.kind = XGC2_PX4_FCU_REQUEST_MODE;
                std::strncpy(p.mode, std::get<std::string>(it->second).c_str(), sizeof p.mode - 1);
                e.requests.push_back(p);
                break;
            }
            case pmc::output_event_type::PUBLISH_CONTROLLER_STATUS:
                e.statuses.push_back(o.status);
                break;
            case pmc::output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION:
                e.activations.push_back(o.activation);
                break;
            case pmc::output_event_type::REQUEST_NMPC_SOLVE:
                ++e.solves;
                break;
            default:
                break;
        }
    }
    return e;
}

template <class T>
bool sameBytes(const T& a, const T& b) {
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

bool sameBits(double a, double b) {
    return std::memcmp(&a, &b, sizeof a) == 0;
}

// The fields of an attitude-rate command as exact hexadecimal doubles, for a failure message.
std::string describe(const xgc2_px4_attitude_rate_target_v1& a) {
    char text[256];
    std::snprintf(text, sizeof text, "stamp %u.%09u rate %a %a %a thrust %a", a.stamp_sec,
                  a.stamp_nsec, a.body_rate[0], a.body_rate[1], a.body_rate[2], a.thrust);
    return text;
}

bool sameActivation(const pmc::reference::AnalyticReference& a,
                    const pmc::reference::AnalyticReference& b) {
    if (a.header.stamp != b.header.stamp || a.request_id != b.request_id ||
        a.trajectory_id != b.trajectory_id || a.revision != b.revision ||
        a.analytic_type != b.analytic_type || a.flags != b.flags || a.start_time != b.start_time ||
        !sameBits(a.duration, b.duration) || a.params.size() != b.params.size()) {
        return false;
    }
    const double pa[] = {a.origin.position.x,    a.origin.position.y,    a.origin.position.z,
                         a.origin.orientation.x, a.origin.orientation.y, a.origin.orientation.z,
                         a.origin.orientation.w};
    const double pb[] = {b.origin.position.x,    b.origin.position.y,    b.origin.position.z,
                         b.origin.orientation.x, b.origin.orientation.y, b.origin.orientation.z,
                         b.origin.orientation.w};
    for (int i = 0; i < 7; ++i) {
        if (!sameBits(pa[i], pb[i]))
            return false;
    }
    for (size_t i = 0; i < a.params.size(); ++i) {
        if (!sameBits(a.params[i], b.params[i]))
            return false;
    }
    return true;
}

struct Totals {
    size_t ticks{0}, setpoints{0}, attitudes{0}, requests{0}, statuses{0}, activations{0},
        solves{0};
    std::vector<std::string> states;  // the flight states in the order they were entered
};

// Runs the node path and the module on the same records and compares them after every tick.
void replay(const std::vector<Record>& records, const std::string& module_config,
            const pmc::ControllerConfig& oracle_config, Totals& totals) {
    ASSERT_FALSE(records.empty());
    ModuleLibrary library(CONTROLLER_MODULE_PATH);
    TestHost host(library.desc());
    const uint64_t t0_ns = records.front().t_ns / kTickNs * kTickNs;
    const uint64_t t_end_ns = records.back().t_ns + 500000000ULL;
    host.setNow(static_cast<int64_t>(t0_ns));
    ASSERT_EQ(host.create(module_config), XGC2_OK);
    ASSERT_EQ(host.start(), XGC2_OK);
    controller_replay::NodePath node(oracle_config, pmc::Time().fromNSec(t0_ns).toSec());

    size_t next = 0;
    for (uint64_t k = 0;; ++k) {
        const uint64_t tick_ns = t0_ns + k * kTickNs;
        if (tick_ns > t_end_ns)
            break;
        while (next < records.size() && records[next].t_ns <= tick_ns) {
            node.receive(records[next]);
            deliver(host, records[next]);
            if (::testing::Test::HasFatalFailure())
                return;
            ++next;
        }
        const pmc::Time now = pmc::Time().fromNSec(tick_ns);
        host.setNow(static_cast<int64_t>(tick_ns));
        const uint64_t wakes = host.wakeCount();
        ASSERT_EQ(host.step(), XGC2_OK) << "tick " << k;
        const std::vector<OutputEvent> handled = node.update(now.toSec());
        const Expected want = expectedOutputs(handled, now);
        // A solve requested in this tick is done by the module's worker: wait for its wake, so its
        // result is posted before the next update, as the inline solve's is.
        if (want.solves > 0) {
            ASSERT_TRUE(host.waitWake(wakes, std::chrono::seconds(5))) << "tick " << k;
        }

        const auto setpoints = host.outputsAs<xgc2_px4_position_target_v1>("setpoint");
        ASSERT_EQ(setpoints.size(), want.setpoints.size()) << "setpoints at tick " << k;
        for (size_t i = 0; i < setpoints.size(); ++i) {
            ASSERT_TRUE(sameBytes(setpoints[i], want.setpoints[i]))
                << "setpoint " << i << " at tick " << k;
        }
        const auto attitudes = host.outputsAs<xgc2_px4_attitude_rate_target_v1>("attitude_rate");
        ASSERT_EQ(attitudes.size(), want.attitudes.size()) << "attitude rates at tick " << k;
        for (size_t i = 0; i < attitudes.size(); ++i) {
            ASSERT_TRUE(sameBytes(attitudes[i], want.attitudes[i]))
                << "attitude rate " << i << " at tick " << k
                << "\nmodule: " << describe(attitudes[i])
                << "\nnode:   " << describe(want.attitudes[i]);
        }
        const auto requests = host.outputsAs<xgc2_px4_fcu_request_v1>("fcu_request");
        ASSERT_EQ(requests.size(), want.requests.size()) << "fcu requests at tick " << k;
        for (size_t i = 0; i < requests.size(); ++i) {
            ASSERT_TRUE(sameBytes(requests[i], want.requests[i]))
                << "fcu request " << i << " at tick " << k;
        }
        const auto statuses = host.outputsAs<xgc2_px4_controller_status_v1>("status");
        ASSERT_EQ(statuses.size(), want.statuses.size()) << "statuses at tick " << k;
        for (size_t i = 0; i < statuses.size(); ++i) {
            ASSERT_STREQ(statuses[i].state, want.statuses[i].c_str())
                << "status " << i << " at tick " << k;
        }
        const auto activations = host.outputsAs<xgc2_px4_reference_analytic_v1>("ref_request");
        ASSERT_EQ(activations.size(), want.activations.size())
            << "activation requests at tick " << k;
        for (size_t i = 0; i < activations.size(); ++i) {
            pmc::reference::AnalyticReference got;
            ASSERT_TRUE(conv::toCore(activations[i], got));
            ASSERT_STREQ(activations[i].header.frame_id, "map");
            ASSERT_TRUE(sameActivation(got, want.activations[i]))
                << "activation " << i << " at tick " << k;
        }
        ASSERT_EQ(host.lastReportDetail(), node.state()) << "flight state at tick " << k;

        totals.setpoints += setpoints.size();
        totals.attitudes += attitudes.size();
        totals.requests += requests.size();
        totals.statuses += statuses.size();
        totals.activations += activations.size();
        totals.solves += want.solves;
        ++totals.ticks;
        if (totals.states.empty() || totals.states.back() != node.state())
            totals.states.push_back(node.state());
        host.clearOutputs();
    }
    ASSERT_EQ(next, records.size());
    EXPECT_FALSE(host.logged(3, "refused"));  // no input or output failed
}

std::string recording(const char* name) {
    return std::string(CONTROLLER_REPLAY_DATA_DIR) + "/" + name + ".stream.gz";
}

// The recorded flights were replayed with the circle-entry reference of the recordings.
std::string recordedConfig(const char* backend) {
    return std::string("{\"world_boundary_json\": \"null\", \"tracking_backend\": \"") + backend +
           "\", \"nmpc\": {\"reference_analytic_type\": 3}}";
}

pmc::ControllerConfig recordedOracle(pmc::TrackingBackend backend) {
    pmc::ControllerConfig config = controller_replay::NodePath::profileConfig(backend);
    config.nmpc.reference_analytic_type = 3;
    return config;
}

TEST(ControllerModuleParity, RecordedPx4LocalFlightTakeoffHoverTrackingAndLanding) {
    Totals t;
    replay(controller_replay::readStream(recording("px4_local")), recordedConfig("px4_local"),
           recordedOracle(pmc::TrackingBackend::PX4_LOCAL), t);
    // Guard against a vacuous comparison: the recording is a whole flight.
    const std::vector<std::string> flight = {"SelfCheck",
                                             "Ready",
                                             "TakeoffInit",
                                             "TakeoffOffboardRequest",
                                             "TakeoffArmRequest",
                                             "TakeoffAscending",
                                             "Hover",
                                             "Custom1",
                                             "Hover",
                                             "Landing"};
    EXPECT_EQ(t.states, flight);
    EXPECT_GT(t.setpoints, 700U);
    EXPECT_GE(t.requests, 5U);
    EXPECT_GT(t.statuses, 100U);
}

TEST(ControllerModuleParity, RecordedDfbcFlightWithEstimateHoverThrustAndReference) {
    Totals t;
    replay(controller_replay::readStream(recording("dfbc")), recordedConfig("dfbc"),
           recordedOracle(pmc::TrackingBackend::DFBC), t);
    const std::vector<std::string> flight = {"SelfCheck", "Ready", "Landing"};
    EXPECT_EQ(t.states, flight);
    EXPECT_GT(t.setpoints, 100U);
    EXPECT_GT(t.statuses, 50U);
}

TEST(ControllerModuleParity, RecordedNmpcFlightWithEstimateHoverThrustAndReference) {
    Totals t;
    replay(controller_replay::readStream(recording("nmpc")), recordedConfig("nmpc"),
           recordedOracle(pmc::TrackingBackend::NMPC), t);
    const std::vector<std::string> flight = {"SelfCheck", "Ready", "Landing"};
    EXPECT_EQ(t.states, flight);
    EXPECT_GT(t.setpoints, 100U);
}

// The scripted flight reaches Custom1: the module must request the reference exactly as the node
// does, and track it with attitude-rate commands that equal the node's.
std::string scriptedConfig(const char* backend) {
    return std::string(
               "{\"world_boundary_json\": \"null\", \"skip_takeoff_init_disarm\": true, "
               "\"takeoff_altitude\": 1.5, \"tracking_backend\": \"") +
           backend + "\"}";
}

pmc::ControllerConfig scriptedOracle(pmc::TrackingBackend backend) {
    pmc::ControllerConfig config = controller_replay::NodePath::profileConfig(backend);
    config.skip_takeoff_init_disarm = true;
    config.takeoff_altitude = 1.5;
    return config;
}

TEST(ControllerModuleParity, ScriptedFlightWithDfbcTracking) {
    Totals t;
    replay(controller_replay::scriptedFlight({}), scriptedConfig("dfbc"),
           scriptedOracle(pmc::TrackingBackend::DFBC), t);
    ASSERT_FALSE(t.states.empty());
    EXPECT_EQ(t.states.back(), "Custom1");
    EXPECT_EQ(t.activations, 1U);
    EXPECT_GT(t.attitudes, 100U);
}

TEST(ControllerModuleParity, ScriptedFlightWithNmpcTrackingSolvedOnTheWorkerThread) {
    Totals t;
    replay(controller_replay::scriptedFlight({}), scriptedConfig("nmpc"),
           scriptedOracle(pmc::TrackingBackend::NMPC), t);
    ASSERT_FALSE(t.states.empty());
    EXPECT_EQ(t.states.back(), "Custom1");
    EXPECT_EQ(t.activations, 1U);
    EXPECT_GT(t.solves, 100U);
    EXPECT_GT(t.attitudes, 100U);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
