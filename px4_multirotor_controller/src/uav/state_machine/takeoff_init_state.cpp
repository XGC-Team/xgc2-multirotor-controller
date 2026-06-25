#include "px4_multirotor_controller/uav/state_machine/takeoff_init_state.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

TakeoffInitState::TakeoffInitState(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult TakeoffInitState::onEnter(::state_machine::StateContext& ctx) {
    controller_.logInfo("[TakeoffInitState] Entering Takeoff Initialization");

    skip_altctl_gate_ = controller_.getConfig().skip_takeoff_init_disarm;
    altctl_frame_count_ = skip_altctl_gate_ ? REQUIRED_ALTCTL_FRAMES : 0;
    altctl_ready_event_posted_ = false;

    configureTakeoffSetpoint();
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                               ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    setpoint_publish_timer_.start();
    requestDisarmIfRequired(ctx);

    if (!skip_altctl_gate_) {
        altctl_request_timer_.start();
    }
    return {};
}

void TakeoffInitState::configureTakeoffSetpoint() {
    const auto& sensor_data = controller_.getSensorData();
    initial_altitude_ = sensor_data.z;

    const double configured_target_altitude = controller_.getConfig().takeoff_altitude;
    target_altitude_ = std::max(configured_target_altitude, initial_altitude_);

    Setpoint& setpoint = controller_.getSetpoint();
    setpoint.x = sensor_data.x;
    setpoint.y = sensor_data.y;
    setpoint.z = target_altitude_;
    setpoint.vx = 0.0;
    setpoint.vy = 0.0;
    setpoint.vz = 0.0;
    setpoint.ax = 0.0;
    setpoint.ay = 0.0;
    setpoint.az = 0.0;
    setpoint.qx = 0.0;
    setpoint.qy = 0.0;
    setpoint.qz = 0.0;
    setpoint.qw = 1.0;
    setpoint.yaw_rate = 0.0;
    setpoint.type_mask = TAKEOFF_POSITION_TYPE_MASK;

    if (target_altitude_ > configured_target_altitude) {
        controller_.logWarn(
            "[TakeoffInitState] Configured target %.2f m is below current "
            "altitude %.2f m, using current altitude",
            configured_target_altitude, initial_altitude_);
    }
    controller_.logInfo("[TakeoffInitState] Initial altitude: %.2f m, Target: %.2f m",
                        initial_altitude_, target_altitude_);
}

void TakeoffInitState::requestDisarmIfRequired(::state_machine::StateContext& ctx) {
    if (skip_altctl_gate_) {
        controller_.logInfo(
            "[TakeoffInitState] Skipping DISARM request and ALTCTL gate by "
            "parameter");
        return;
    }

    controller_.logInfo("[TakeoffInitState] Requesting Disarm");
    ::state_machine::Event event(output_event_type::REQUEST_ARMING,
                                 ::state_machine::EventTimestamp{controller_.getCurrentTime()});
    event.payload["arm"] = false;
    ctx.emitOutput(std::move(event));
}

::state_machine::ActionResult TakeoffInitState::onTick(::state_machine::StateContext& ctx) {
    if (shouldRunEvery(setpoint_publish_timer_, SETPOINT_PUBLISH_INTERVAL, true)) {
        ctx.emitOutput(
            ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                                   ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    }

    if (!skip_altctl_gate_) {
        requestAltctlModeIfDue(ctx);
        updateAltctlConfirmation();
    }

    postAltctlReadyOnce(ctx);
    return {};
}

void TakeoffInitState::requestAltctlModeIfDue(::state_machine::StateContext& ctx) {
    if (altctl_request_timer_.elapsed().count() < ALTCTL_REQUEST_INTERVAL) {
        return;
    }

    controller_.logInfo("[TakeoffInitState] Requesting ALTCTL mode (periodic)");
    ::state_machine::Event event(output_event_type::REQUEST_MODE,
                                 ::state_machine::EventTimestamp{controller_.getCurrentTime()});
    event.payload["mode"] = std::string("ALTCTL");
    ctx.emitOutput(std::move(event));
    altctl_request_timer_.reset();
}

void TakeoffInitState::updateAltctlConfirmation() {
    const auto& sensor_data = controller_.getSensorData();
    if (!sensor_data.state_stats.is_new) {
        return;
    }

    if (sensor_data.fcu_mode == "ALTCTL") {
        altctl_frame_count_ = std::min(altctl_frame_count_ + 1, REQUIRED_ALTCTL_FRAMES);
        controller_.logInfo("[TakeoffInitState] Received ALTCTL frame %d/%d", altctl_frame_count_,
                            REQUIRED_ALTCTL_FRAMES);
        return;
    }

    if (altctl_frame_count_ > 0) {
        controller_.logWarn(
            "[TakeoffInitState] Mode changed to %s, resetting ALTCTL counter "
            "(was %d)",
            sensor_data.fcu_mode.c_str(), altctl_frame_count_);
        altctl_frame_count_ = 0;
    }
}

void TakeoffInitState::postAltctlReadyOnce(::state_machine::StateContext& ctx) {
    if (altctl_ready_event_posted_ || altctl_frame_count_ < REQUIRED_ALTCTL_FRAMES) {
        return;
    }

    if (skip_altctl_gate_) {
        controller_.logInfo("[TakeoffInitState] ALTCTL mode request/check skipped by parameter");
    } else {
        controller_.logInfo("[TakeoffInitState] ALTCTL mode ready (%d frames received)",
                            altctl_frame_count_);
    }

    altctl_ready_event_posted_ = true;
    ctx.postInternalEvent(::state_machine::Event(
        event_type::ALTCTL_READY, ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
}

::state_machine::ActionResult TakeoffInitState::onExit(::state_machine::StateContext& ctx) {
    const double elapsed_time =
        std::chrono::duration<double>(ctx.elapsed(state_type::TakeoffInit)).count();
    controller_.logInfo(
        "[TakeoffInitState] Exiting (ALTCTL frames received: %d, elapsed: "
        "%.2f s)",
        altctl_frame_count_, elapsed_time);
    altctl_request_timer_.stop();
    setpoint_publish_timer_.stop();
    return {};
}

}  // namespace px4_multirotor_controller
