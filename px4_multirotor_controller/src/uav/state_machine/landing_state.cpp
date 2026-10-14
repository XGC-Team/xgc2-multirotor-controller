#include "px4_multirotor_controller/uav/state_machine/landing_state.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

LandingState::LandingState(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult LandingState::onEnter(::state_machine::StateContext& ctx) {
    controller_.clearCustom1Request();
    controller_.logInfo("[LandingState] Entering Landing State");

    const auto& sensor_data = controller_.getSensorData();
    initial_altitude_ =
        sensor_checks::worldZ(sensor_data, controller_.getConfig().tracking_backend);

    // Preserve the existing timeout diagnostic budget, independently of the FCU's landing surface.
    max_landing_duration_ = std::min(
        std::max(initial_altitude_, 0.0) / LANDING_DIAGNOSTIC_DESCENT_RATE + LANDING_TIME_MARGIN,
        LANDING_TIMEOUT);
    controller_.logInfo("[LandingState] Native AUTO.LAND handoff, diagnostic duration: %.1f s",
                        max_landing_duration_);

    ::state_machine::Event event(
        output_event_type::REQUEST_MODE,
        ::state_machine::EventTimestamp{controller_.getCurrentTime()});
    event.payload["mode"] = std::string("AUTO.LAND");
    ctx.emitOutput(std::move(event));
    log_timer_.start();

    touchdown_event_posted_ = false;
    timeout_reported_ = false;
    exit_reason_ = ExitReason::UNKNOWN;
    return {};
}

::state_machine::ActionResult LandingState::onTick(::state_machine::StateContext& ctx) {
    logStatusIfDue();
    reportTimeoutIfNeeded(ctx);
    postTouchdownOnce(ctx);
    return {};
}

void LandingState::logStatusIfDue() {
    if (log_timer_.elapsed().count() < 1.0) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    controller_.logInfo(
        "[LandingState] Waiting for native auto-disarm: connected=%s, mode=%s, armed=%s",
        sensor_checks::isFcuConnected(sensor_data) ? "yes" : "no",
        sensor_data.fcu_mode.c_str(), sensor_checks::isFcuArmed(sensor_data) ? "yes" : "no");
    log_timer_.reset();
}

void LandingState::reportTimeoutIfNeeded(::state_machine::StateContext& ctx) {
    if (timeout_reported_) {
        return;
    }
    const double elapsed_time =
        std::chrono::duration<double>(ctx.elapsed(state_type::Landing)).count();
    if (elapsed_time < max_landing_duration_) {
        return;
    }
    timeout_reported_ = true;
    controller_.logWarn(
        "[LandingState] Landing timeout after %.1f s; NOT confirmed landed/disarmed. "
        "Keeping Landing state; operator intervention required. No controller disarm request.",
        elapsed_time);
}

void LandingState::postTouchdownOnce(::state_machine::StateContext& ctx) {
    const auto& sensor = controller_.getSensorData();
    if (touchdown_event_posted_ || !sensor.state_stats.is_active || !sensor.state_stats.is_new ||
        !sensor_checks::isFcuConnected(sensor) || sensor_checks::isFcuArmed(sensor)) {
        return;
    }
    touchdown_event_posted_ = true;
    exit_reason_ = ExitReason::TOUCHDOWN;
    ctx.postInternalEvent(::state_machine::Event(
        event_type::TOUCHDOWN, ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
}

::state_machine::ActionResult LandingState::onExit(::state_machine::StateContext& ctx) {
    controller_.logInfo("[LandingState] Exiting Landing State");
    log_timer_.stop();

    (void)ctx;
    if (exit_reason_ == ExitReason::TOUCHDOWN) {
        controller_.logInfo("[LandingState] Landing complete: fresh FCU disarm confirmed");
    } else {
        controller_.logWarn("[LandingState] Exited without confirmed touchdown/disarm; "
                            "no controller disarm request issued");
    }
    return {};
}

}  // namespace px4_multirotor_controller
