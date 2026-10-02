// Same test source links against the reviewed old ELF for the red receipt and
// the owner driver/core ELF for green. It drives real FSM transitions, not a
// second control strategy or a copied buffer implementation.
#include <stdexcept>
#define VERIFY(condition) do { if (!(condition)) throw std::runtime_error("check failed: " #condition); } while (false)
#include <iostream>
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
int main() {
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
}
