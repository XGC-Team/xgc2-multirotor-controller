#include "px4_multirotor_controller/uav/state_machine/landing_state.h"

#include <algorithm>
#include <chrono>
#include <cmath>
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

    const double slowest_descent_rate = std::max(
        std::min(std::abs(LANDING_VZ_HIGH_ALTITUDE), std::abs(LANDING_VZ_LOW_ALTITUDE)), 1e-3);
    const double positive_initial_altitude = std::max(initial_altitude_, 0.0);
    max_landing_duration_ =
        (positive_initial_altitude / slowest_descent_rate) + LANDING_TIME_MARGIN;
    max_landing_duration_ = std::min(max_landing_duration_, LANDING_TIMEOUT);

    configureLandingSetpoint();

    controller_.logInfo("[LandingState] Initial altitude: %.2f m, max duration: %.1f s",
                        initial_altitude_, max_landing_duration_);

    emitLandingSetpoint(ctx);
    setpoint_publish_timer_.start();
    log_timer_.start();

    touchdown_event_posted_ = false;
    timeout_reported_ = false;
    disarm_requested_ = false;
    confirmed_landed_frames_ = 0;
    exit_reason_ = ExitReason::UNKNOWN;
    return {};
}

double LandingState::getDescentVelocity(double altitude) {
    return altitude > LANDING_ALTITUDE_THRESHOLD ? LANDING_VZ_HIGH_ALTITUDE
                                                 : LANDING_VZ_LOW_ALTITUDE;
}

void LandingState::configureLandingSetpoint() {
    const auto& sensor_data = controller_.getSensorData();
    landing_setpoint_.type_mask = LANDING_VELOCITY_TYPE_MASK;
    landing_setpoint_.x = 0.0;
    landing_setpoint_.y = 0.0;
    landing_setpoint_.z = 0.0;
    landing_setpoint_.vx = 0.0;
    landing_setpoint_.vy = 0.0;
    landing_setpoint_.vz = getDescentVelocity(
        sensor_checks::worldZ(sensor_data, controller_.getConfig().tracking_backend));
    landing_setpoint_.ax = 0.0;
    landing_setpoint_.ay = 0.0;
    landing_setpoint_.az = 0.0;
    landing_setpoint_.qx = 0.0;
    landing_setpoint_.qy = 0.0;
    landing_setpoint_.qz = 0.0;
    landing_setpoint_.qw = 1.0;
    landing_setpoint_.yaw_rate = 0.0;
}

void LandingState::emitLandingSetpoint(::state_machine::StateContext& ctx) {
    controller_.getSetpoint() = landing_setpoint_;
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                               ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
}

::state_machine::ActionResult LandingState::onTick(::state_machine::StateContext& ctx) {
    updateDescentVelocityIfNeeded();
    publishSetpointIfDue(ctx);
    logStatusIfDue();
    reportTimeoutIfNeeded(ctx);
    updateTouchdownConfirmation();
    postTouchdownOnce(ctx);
    return {};
}

void LandingState::updateDescentVelocityIfNeeded() {
    const auto& sensor_data = controller_.getSensorData();
    const auto backend = controller_.getConfig().tracking_backend;
    if (sensor_checks::isWorldPoseNew(sensor_data, backend)) {
        landing_setpoint_.vz = getDescentVelocity(sensor_checks::worldZ(sensor_data, backend));
    }
}

void LandingState::publishSetpointIfDue(::state_machine::StateContext& ctx) {
    if (shouldRunEvery(setpoint_publish_timer_, SETPOINT_PUBLISH_INTERVAL, true)) {
        emitLandingSetpoint(ctx);
    }
}

void LandingState::logStatusIfDue() {
    if (log_timer_.elapsed().count() < 1.0) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    controller_.logInfo(
        "[LandingState] Altitude: %.2f m, Descent rate: %.2f m/s, landed "
        "frames: %d/%d, disarm requested: %s",
        sensor_checks::worldZ(sensor_data, controller_.getConfig().tracking_backend),
        landing_setpoint_.vz, confirmed_landed_frames_, CONSECUTIVE_SETTLED_FRAMES,
        disarm_requested_ ? "yes (awaiting telemetry)" : "no");
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
        "Keeping Landing output; operator intervention required. No automatic force-disarm.",
        elapsed_time);
}

bool LandingState::landingFeedbackActive() const {
    const auto& sensor = controller_.getSensorData();
    const auto backend = controller_.getConfig().tracking_backend;
    const bool active = trackingUsesFusedEstimate(backend)
                            ? sensor.uav_state_estimate_stats.is_active
                            : sensor.local_pos_stats.is_active &&
                                  sensor.local_velocity_stats.is_active;
    return active && std::isfinite(sensor_checks::worldZ(sensor, backend)) &&
           std::isfinite(sensor_checks::worldVz(sensor, backend));
}

void LandingState::updateTouchdownConfirmation() {
    if (touchdown_event_posted_) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    const auto backend = controller_.getConfig().tracking_backend;
    if (!landingFeedbackActive()) {
        confirmed_landed_frames_ = 0;
        return;
    }
    const bool sensor_updated = sensor_checks::isWorldPoseNew(sensor_data, backend);
    if (!sensor_updated) {
        return;
    }

    const bool velocity_settled =
        std::abs(sensor_checks::worldVz(sensor_data, backend)) <= TOUCHDOWN_VELOCITY_THRESHOLD;
    const bool altitude_settled = sensor_checks::worldZ(sensor_data, backend) <= GROUND_ALTITUDE;
    if (!velocity_settled || !altitude_settled) {
        confirmed_landed_frames_ = 0;
        return;
    }

    if (confirmed_landed_frames_ < CONSECUTIVE_SETTLED_FRAMES) {
        ++confirmed_landed_frames_;
    }
}

void LandingState::postTouchdownOnce(::state_machine::StateContext& ctx) {
    if (touchdown_event_posted_ || confirmed_landed_frames_ < CONSECUTIVE_SETTLED_FRAMES) {
        return;
    }

    const auto& sensor = controller_.getSensorData();
    if (!landingFeedbackActive() || !sensor.state_stats.is_active ||
        !sensor_checks::isFcuConnected(sensor)) {
        return;
    }
    if (!sensor_checks::isFcuArmed(sensor)) {
        // A cached or inactive armed=false is not a new FCU acknowledgement.
        if (!sensor.state_stats.is_new) {
            return;
        }
        touchdown_event_posted_ = true;
        exit_reason_ = ExitReason::TOUCHDOWN;
        ctx.postInternalEvent(::state_machine::Event(
            event_type::TOUCHDOWN, ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
        return;
    }
    if (!disarm_requested_) {
        ::state_machine::Event event(
            output_event_type::REQUEST_ARMING,
            ::state_machine::EventTimestamp{controller_.getCurrentTime()});
        event.payload["arm"] = false;
        const auto status = ctx.emitOutput(std::move(event));
        if (status.ok()) {
            disarm_requested_ = true;
            controller_.logInfo(
                "[LandingState] Normal disarm requested; waiting for fresh FCU armed=false");
        }
    }
}

::state_machine::ActionResult LandingState::onExit(::state_machine::StateContext& ctx) {
    controller_.logInfo("[LandingState] Exiting Landing State");
    log_timer_.stop();
    setpoint_publish_timer_.stop();

    (void)ctx;
    if (exit_reason_ == ExitReason::TOUCHDOWN) {
        controller_.logInfo("[LandingState] Landing complete: fresh FCU disarm confirmed");
    } else {
        controller_.logWarn("[LandingState] Exited without confirmed touchdown/disarm; "
                            "no automatic force-disarm issued");
    }
    return {};
}

}  // namespace px4_multirotor_controller
