// Same test source links against the reviewed old ELF for the red receipt and
// the owner driver/core ELF for green. It drives real FSM transitions, not a
// second control strategy or a copied buffer implementation.
#include <stdexcept>
#define VERIFY(condition) do { if (!(condition)) throw std::runtime_error("check failed: " #condition); } while (false)
#include <iostream>
#include <limits>
#include <string>
#include <type_traits>
#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/common/state_estimate_status.h"
using namespace px4_multirotor_controller;
template<class T, class = void> struct HasGeneration : std::false_type {};
template<class T> struct HasGeneration<T, std::void_t<decltype(T{}.control_generation)>> : std::true_type {};
template<class T> void token(T& result, const ::state_machine::Event& event) {
    if constexpr (HasGeneration<T>::value) result.control_generation =
        static_cast<uint64_t>(std::get<int64_t>(event.payload.at("control_generation")));
}
int main(int argc, char** argv) {
    SensorData sensor;
    sensor.local_pos_stats.is_active = sensor.local_velocity_stats.is_active =
        sensor.imu_stats.is_active = sensor.state_stats.is_active =
        sensor.uav_state_estimate_stats.is_active = sensor.vrpn_pose_stats.is_active = true;
    sensor.local_pos_stats.is_new = sensor.vrpn_pose_stats.is_new = true;
    sensor.uav_state_estimator_state = state_estimate::STATE_RUNNING;
    sensor.fcu_connected = true;
    sensor.local_z = sensor.vrpn_z = sensor.z = 1.0;
    sensor.qw = sensor.local_qw = sensor.vrpn_qw = 1.0;
    DroneController controller(sensor);
    ControllerConfig config; config.tracking_backend = TrackingBackend::NMPC;
    controller.setConfig(config);
    double now = 100.0;
    auto tick = [&] { controller.update(now += 0.001); };
    auto request = [&](uint32_t id) { VERIFY(controller.getStateMachine().postEvent(
        ::state_machine::Event(id, ::state_machine::EventTimestamp{now})).ok()); tick(); };
    tick();
    VERIFY(controller.getStateMachine().currentState(region_type::CONTROL) == state_type::Ready);
    request(event_type::TAKEOFF_REQUESTED); request(event_type::ALTCTL_READY);
    request(event_type::OFFBOARD_READY); sensor.fcu_armed = true;
    request(event_type::ARM_READY); request(event_type::ALTITUDE_REACHED);
    VERIFY(controller.getStateMachine().currentState(region_type::CONTROL) == state_type::Hover);
    auto seed_reference = [&] {
        reference::AnalyticReference ref; ref.start_time = ref.header.stamp = Time(now);
        ref.duration = 60.0; ref.analytic_type = reference::AnalyticReference::ANALYTIC_HOLD;
        ref.origin.position.z = 1.0; ref.origin.orientation.w = 1.0;
        VERIFY(controller.activeTrajectoryCache().updateAnalytic(ref, Time(now)));
    };
    request(event_type::TRAJECTORY_TRACKING_REQUESTED); seed_reference(); tick();
    ::state_machine::Event solve;
    bool found = false;
    for (const auto& event : controller.getStateMachine().currentOutputEvents()) {
        if (event.id == output_event_type::REQUEST_NMPC_SOLVE) { solve = event; found = true; }
    }
    VERIFY(found);
    NmpcSolveResult success; success.sequence = solve.correlation_id; token(success, solve);
    success.success = true; success.stamp = Time(now); success.target.thrust = 0.61;
    controller.nmpcResultBuffer().store(success); tick();
    bool published = false;
    for (const auto& e : controller.getStateMachine().currentOutputEvents())
        published |= e.id == output_event_type::PUBLISH_ATTITUDE_RATE_TARGET;
    VERIFY(published && controller.getAttitudeRateTarget().thrust == 0.61);
    request(event_type::HOVER_REQUESTED);
    request(event_type::TRAJECTORY_TRACKING_REQUESTED); seed_reference(); tick();
    for (const auto& e : controller.getStateMachine().currentOutputEvents()) {
        if (e.id == output_event_type::PUBLISH_ATTITUDE_RATE_TARGET) {
            std::cerr << "OLD GENERATION TARGET WAS REPUBLISHED ON REENTRY\n"; return 1;
        }
    }
    // The old solve arriving after exit/reentry must also be rejected.
    controller.nmpcResultBuffer().store(success); tick();
    for (const auto& e : controller.getStateMachine().currentOutputEvents()) {
        if (e.id == output_event_type::PUBLISH_ATTITUDE_RATE_TARGET) {
            std::cerr << "LATE OLD GENERATION TARGET WAS REPUBLISHED\n"; return 2;
        }
    }
    std::cout << "success -> exit -> reenter and late old result: no stale target\n";

    struct TimestampCase {
        const char* name;
        int64_t age_ns;
        bool malformed;
        bool publish;
        double timeout;
    };
    const TimestampCase cases[] = {
        {"same_tick", 0, false, true, 0.1},
        {"timeout_boundary", 100000000, false, true, 0.1},
        {"expired_before_solve_deadline", 100000001, false, false, 0.1},
        {"future_one_nanosecond", -1, false, false, 0.1},
        {"future_one_second", -1000000000, false, false, 0.1},
        {"noncanonical_nanoseconds", 0, true, false, 0.1},
        {"age_timeout_disabled", 100000001, false, true, 0.0},
        {"future_timeout_disabled", -1, false, false, 0.0},
        {"negative_timeout_disabled", 100000001, false, true, -1.0},
    };
    unsigned checked = 0;
    for (const auto& test : cases) {
        if (argc > 1 && std::string(argv[1]) != test.name) continue;
        ++checked;
        config.nmpc.result_timeout = test.timeout; controller.setConfig(config);
        request(event_type::HOVER_REQUESTED);
        request(event_type::TRAJECTORY_TRACKING_REQUESTED); seed_reference(); tick();
        VERIFY(controller.getStateMachine().currentState(region_type::CONTROL) == state_type::Custom1);
        found = false;
        for (const auto& event : controller.getStateMachine().currentOutputEvents()) {
            if (event.id == output_event_type::REQUEST_NMPC_SOLVE) { solve = event; found = true; }
        }
        VERIFY(found);
        const Time consume_time(now + 0.001);
        NmpcSolveResult result; result.sequence = solve.correlation_id; token(result, solve);
        result.success = true; result.target.thrust = 0.73;
        const uint64_t stamp_ns = test.age_ns >= 0
            ? consume_time.toNSec() - static_cast<uint64_t>(test.age_ns)
            : consume_time.toNSec() + static_cast<uint64_t>(-test.age_ns);
        result.stamp.fromNSec(stamp_ns);
        if (test.malformed) {
            --result.stamp.sec;
            result.stamp.nsec += 1000000000U;  // same numeric seconds, invalid representation
        }
        const double previous_thrust = controller.getAttitudeRateTarget().thrust;
        VERIFY(controller.nmpcResultBuffer().store(result) == !test.malformed);
        tick();
        published = false;
        for (const auto& event : controller.getStateMachine().currentOutputEvents()) {
            if (event.id == output_event_type::PUBLISH_ATTITUDE_RATE_TARGET) {
                published = true;
                VERIFY(controller.getAttitudeRateTarget().thrust == 0.73);
            }
        }
        if (published != test.publish) {
            std::cerr << "timestamp case " << test.name << ": expected publish="
                      << test.publish << ", actual=" << published << "\n";
            return 3;
        }
        if (!test.publish) VERIFY(controller.getAttitudeRateTarget().thrust == previous_thrust);
        // A replayed request cannot refresh or replace a target already consumed.
        result.target.thrust = 0.91;
        VERIFY(!controller.nmpcResultBuffer().store(result)); tick();
        for (const auto& event : controller.getStateMachine().currentOutputEvents()) {
            VERIFY(event.id != output_event_type::PUBLISH_ATTITUDE_RATE_TARGET);
        }
        // An out-of-order result cannot supersede this generation's stored request.
        if (result.sequence > 1) {
            --result.sequence;
            VERIFY(!controller.nmpcResultBuffer().store(result));
        }
        std::cout << "PASS " << test.name << " publish=" << published
                  << " target_before=" << previous_thrust
                  << " target_after=" << controller.getAttitudeRateTarget().thrust
                  << " duplicate/older request rejected\n";
    }
    VERIFY(checked > 0);
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(), -1.0}) {
        bool rejected = false;
        try { (void)Time(invalid); } catch (const std::runtime_error&) { rejected = true; }
        VERIFY(rejected);
    }
    std::cout << "PASS original Time rejects NaN/infinity/negative stamps\n";
}
