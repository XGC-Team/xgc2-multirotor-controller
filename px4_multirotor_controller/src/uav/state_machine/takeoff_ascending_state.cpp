#include "px4_multirotor_controller/uav/state_machine/takeoff_ascending_state.h"

#include <chrono>
#include <cmath>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

TakeoffAscendingState::TakeoffAscendingState(DroneController& controller)
    : controller_(controller) {}

::state_machine::ActionResult TakeoffAscendingState::onEnter(::state_machine::StateContext& ctx) {
    controller_.logInfo("[TakeoffAscendingState] Starting ascent");

    altitude_reached_event_posted_ = false;
    timeout_event_posted_ = false;
    confirmed_settled_frames_ = 0;

    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                               ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    setpoint_publish_timer_.start();
    status_log_timer_.start();
    return {};
}

::state_machine::ActionResult TakeoffAscendingState::onTick(::state_machine::StateContext& ctx) {
    publishSetpointIfDue(ctx);
    logStatusIfDue();
    if (postTimeoutIfNeeded(ctx)) {
        return {};
    }
    updateAltitudeConfirmation();
    postAltitudeReachedOnce(ctx);
    return {};
}

::state_machine::ActionResult TakeoffAscendingState::onEvent(::state_machine::StateContext&,
                                                             const ::state_machine::Event& event) {
    if (event.id == event_type::TRAJECTORY_TRACKING_REQUESTED) {
        controller_.requestCustom1Tracking();
        if (!controller_.custom1ReferenceReady()) {
            controller_.logWarn(
                "[TakeoffAscendingState] custom1 latched; waiting for ready initial reference");
        }
    }
    return {};
}

void TakeoffAscendingState::publishSetpointIfDue(::state_machine::StateContext& ctx) {
    if (shouldRunEvery(setpoint_publish_timer_, SETPOINT_PUBLISH_INTERVAL, true)) {
        ctx.emitOutput(
            ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                                   ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    }
}

void TakeoffAscendingState::logStatusIfDue() {
    if (status_log_timer_.elapsed().count() < 1.0) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    const auto& setpoint = controller_.getSetpoint();
    controller_.logInfo(
        "[TakeoffAscendingState] Altitude: %.2f m (target: %.2f m, vz: "
        "%.2f m/s, settled frames: %d/%d)",
        sensor_checks::worldZ(sensor_data, controller_.getConfig().tracking_backend), setpoint.z,
        sensor_checks::worldVz(sensor_data, controller_.getConfig().tracking_backend),
        confirmed_settled_frames_, CONSECUTIVE_SETTLED_FRAMES);
    status_log_timer_.reset();
}

bool TakeoffAscendingState::postTimeoutIfNeeded(::state_machine::StateContext& ctx) {
    if (timeout_event_posted_) {
        return true;
    }

    const double elapsed_time =
        std::chrono::duration<double>(ctx.elapsed(state_type::TakeoffAscending)).count();
    if (elapsed_time <= ASCENDING_TIMEOUT) {
        return false;
    }

    controller_.logWarn("[TakeoffAscendingState] Ascending timeout after %.1f seconds",
                        elapsed_time);
    timeout_event_posted_ = true;
    ctx.postInternalEvent(
        ::state_machine::Event(event_type::TAKEOFF_TIMEOUT,
                               ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    return true;
}

void TakeoffAscendingState::updateAltitudeConfirmation() {
    if (altitude_reached_event_posted_ || timeout_event_posted_) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    const auto backend = controller_.getConfig().tracking_backend;
    const bool sensor_updated = sensor_checks::isWorldPoseNew(sensor_data, backend);
    if (!sensor_updated) {
        return;
    }

    const auto& setpoint = controller_.getSetpoint();
    const bool altitude_ok =
        sensor_checks::worldZ(sensor_data, backend) >= setpoint.z - ALTITUDE_THRESHOLD;
    const bool velocity_ok =
        std::abs(sensor_checks::worldVz(sensor_data, backend)) < VELOCITY_THRESHOLD;
    if (!altitude_ok || !velocity_ok) {
        confirmed_settled_frames_ = 0;
        return;
    }

    if (confirmed_settled_frames_ < CONSECUTIVE_SETTLED_FRAMES) {
        ++confirmed_settled_frames_;
    }
}

void TakeoffAscendingState::postAltitudeReachedOnce(::state_machine::StateContext& ctx) {
    if (altitude_reached_event_posted_ || confirmed_settled_frames_ < CONSECUTIVE_SETTLED_FRAMES) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    controller_.logInfo(
        "[TakeoffAscendingState] Target altitude reached: %.2f m (%.1f s "
        "elapsed)",
        sensor_checks::worldZ(sensor_data, controller_.getConfig().tracking_backend),
        std::chrono::duration<double>(ctx.elapsed(state_type::TakeoffAscending)).count());
    altitude_reached_event_posted_ = true;
    ctx.postInternalEvent(
        ::state_machine::Event(event_type::ALTITUDE_REACHED,
                               ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
}

::state_machine::ActionResult TakeoffAscendingState::onExit(::state_machine::StateContext& ctx) {
    const auto& sensor_data = controller_.getSensorData();
    const double elapsed_time =
        std::chrono::duration<double>(ctx.elapsed(state_type::TakeoffAscending)).count();
    controller_.logInfo(
        "[TakeoffAscendingState] Exiting (altitude: %.2f m, settled frames: "
        "%d/%d, elapsed: %.2f s)",
        sensor_checks::worldZ(sensor_data, controller_.getConfig().tracking_backend),
        confirmed_settled_frames_, CONSECUTIVE_SETTLED_FRAMES, elapsed_time);
    status_log_timer_.stop();
    setpoint_publish_timer_.stop();
    return {};
}

}  // namespace px4_multirotor_controller
