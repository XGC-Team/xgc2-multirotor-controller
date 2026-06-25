#include "px4_multirotor_controller/uav/state_machine/landing_state.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

LandingState::LandingState(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult LandingState::onEnter(::state_machine::StateContext& ctx) {
    controller_.logInfo("[LandingState] Entering Landing State");

    const auto& sensor_data = controller_.getSensorData();
    initial_altitude_ = sensor_data.z;

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
    timeout_event_posted_ = false;
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
    landing_setpoint_.vz = getDescentVelocity(sensor_data.z);
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
    if (postTimeoutIfNeeded(ctx)) {
        return {};
    }
    updateTouchdownConfirmation();
    postTouchdownOnce(ctx);
    return {};
}

void LandingState::updateDescentVelocityIfNeeded() {
    const auto& sensor_data = controller_.getSensorData();
    if (sensor_data.uav_state_estimate_stats.is_new) {
        landing_setpoint_.vz = getDescentVelocity(sensor_data.z);
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
        "frames: %d/%d",
        sensor_data.z, landing_setpoint_.vz, confirmed_landed_frames_, CONSECUTIVE_SETTLED_FRAMES);
    log_timer_.reset();
}

bool LandingState::postTimeoutIfNeeded(::state_machine::StateContext& ctx) {
    if (timeout_event_posted_) {
        return true;
    }

    const double elapsed_time =
        std::chrono::duration<double>(ctx.elapsed(state_type::Landing)).count();
    if (elapsed_time < max_landing_duration_) {
        return false;
    }

    timeout_event_posted_ = true;
    exit_reason_ = ExitReason::LANDING_TIMEOUT;
    ctx.postInternalEvent(
        ::state_machine::Event(event_type::LANDING_TIMEOUT,
                               ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    return true;
}

void LandingState::updateTouchdownConfirmation() {
    if (touchdown_event_posted_ || timeout_event_posted_) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    const bool sensor_updated = sensor_data.uav_state_estimate_stats.is_new;
    if (!sensor_updated) {
        return;
    }

    const bool velocity_settled = std::abs(sensor_data.vz) <= TOUCHDOWN_VELOCITY_THRESHOLD;
    const bool altitude_settled = sensor_data.z <= GROUND_ALTITUDE;
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

    touchdown_event_posted_ = true;
    exit_reason_ = ExitReason::TOUCHDOWN;
    ctx.postInternalEvent(::state_machine::Event(
        event_type::TOUCHDOWN, ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
}

::state_machine::ActionResult LandingState::onExit(::state_machine::StateContext& ctx) {
    controller_.logInfo("[LandingState] Exiting Landing State");
    log_timer_.stop();
    setpoint_publish_timer_.stop();

    const auto& sensor_data = controller_.getSensorData();
    const double elapsed_time =
        std::chrono::duration<double>(ctx.elapsed(state_type::Landing)).count();

    // 根据退出原因选择不同的处理方式
    switch (exit_reason_) {
        case ExitReason::LANDING_TIMEOUT:
            // 超时着陆：打印警告并强制上锁
            controller_.logWarn(
                "[LandingState] Forced touchdown after %.1f s at altitude %.2f m "
                "(vz=%.2f m/s)",
                elapsed_time, sensor_data.z, sensor_data.vz);
            ctx.emitOutput(::state_machine::Event(
                output_event_type::REQUEST_KILL,
                ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
            break;

        case ExitReason::TOUCHDOWN:
            // 正常着陆：打印信息并上锁
            controller_.logInfo("[LandingState] Landed at altitude: %.2f m (vz=%.2f m/s)",
                                sensor_data.z, sensor_data.vz);
            {
                ::state_machine::Event event(
                    output_event_type::REQUEST_ARMING,
                    ::state_machine::EventTimestamp{controller_.getCurrentTime()});
                event.payload["arm"] = false;
                ctx.emitOutput(std::move(event));
            }
            break;

        case ExitReason::UNKNOWN:
        case ExitReason::OTHER:
        default:
            // 其他原因（例如紧急停止）：不执行特定操作
            controller_.logInfo("[LandingState] Exited due to unknown/other reason");
            break;
    }
    return {};
}

}  // namespace px4_multirotor_controller
