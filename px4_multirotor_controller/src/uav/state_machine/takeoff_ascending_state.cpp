#include "px4_multirotor_controller/uav/state_machine/takeoff_ascending_state.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TakeoffAscendingState::TakeoffAscendingState(DroneController& controller) : controller_(controller) {}

std::optional<double> TakeoffAscendingState::nextWake() const {
    return earlierDeadline(setpoint_publish_timer_.dueAt(SETPOINT_PUBLISH_INTERVAL),
                           status_log_timer_.dueAt(1.0));
}

std::optional<double> TakeoffAscendingState::timeoutAfterEntry() const {
    return timeout_event_posted_ ? std::nullopt : std::optional<double>{ascending_timeout_s_};
}

void TakeoffAscendingState::onEnter() {
    controller_.logInfo("[TakeoffAscendingState] Starting ascent");

    altitude_reached_event_posted_ = false;
    timeout_event_posted_ = false;
    confirmed_settled_frames_ = 0;

    initializeAscentProfile();
    controller_.emitFlightEffect(FlightSetpoint{controller_.getCurrentTime()});
    setpoint_publish_timer_.start();
    status_log_timer_.start();
    return;
}

void TakeoffAscendingState::initializeAscentProfile() {
    const auto explicit_vz_max = controller_.explicitTakeoffVzMax();
    if (!explicit_vz_max.has_value()) {
        profile_active_ = false;
        target_altitude_ = controller_.getSetpoint().z;
        profile_duration_s_ = 0.0;
        ascending_timeout_s_ = ASCENDING_TIMEOUT;
        return;
    }

    profile_active_ = true;
    ascent_start_time_s_ = controller_.getCurrentTime();
    const auto& config = controller_.getConfig();
    const double measured_z =
        sensor_checks::worldZ(controller_.getSensorData(), config.tracking_backend);
    ascent_start_z_ = std::isfinite(measured_z) ? measured_z : controller_.takeoffInitialAltitude();
    target_altitude_ = std::max(controller_.takeoffTargetAltitude(), ascent_start_z_);

    const double height = std::max(0.0, target_altitude_ - ascent_start_z_);
    if (height <= 1e-6) {
        climb_v_peak_ = 0.0;
        ramp_duration_s_ = 0.0;
        cruise_duration_s_ = 0.0;
        profile_duration_s_ = 0.0;
    } else {
        const double v_cap =
            std::max(std::min(kNominalTakeoffClimbVz, kTakeoffVzSafetyFactor * (*explicit_vz_max)),
                     std::min(1e-3, 0.5 * (*explicit_vz_max)));
        const double a_max = std::max(std::min(kMaxTakeoffClimbAccel, 1.5 * v_cap), 1e-3);
        const double full_ramp_s = (kPi * 0.5) * v_cap / a_max;
        const double full_ramp_dist = v_cap * full_ramp_s;
        if (height >= full_ramp_dist) {
            climb_v_peak_ = v_cap;
            ramp_duration_s_ = full_ramp_s;
            cruise_duration_s_ = (height - full_ramp_dist) / v_cap;
        } else {
            climb_v_peak_ = std::sqrt(height * a_max / (kPi * 0.5));
            ramp_duration_s_ = (kPi * 0.5) * climb_v_peak_ / a_max;
            cruise_duration_s_ = 0.0;
        }
        profile_duration_s_ = 2.0 * ramp_duration_s_ + cruise_duration_s_;
    }

    ascending_timeout_s_ = std::max(ASCENDING_TIMEOUT, profile_duration_s_ + 15.0);
    updateAscentSetpoint(ascent_start_time_s_);
    controller_.logInfo(
        "[TakeoffAscendingState] Velocity-bounded ascent: z=%.2f->%.2f m, v_peak=%.2f m/s "
        "(vzMax=%.2f m/s), duration=%.2f s",
        ascent_start_z_, target_altitude_, climb_v_peak_, *explicit_vz_max, profile_duration_s_);
}

void TakeoffAscendingState::updateAscentSetpoint(double now_s) {
    const double tau = std::isfinite(now_s) ? std::max(0.0, now_s - ascent_start_time_s_) : 0.0;
    double z_ref = target_altitude_;
    double vz_ref = 0.0;

    if (profile_duration_s_ > 0.0 && tau < profile_duration_s_) {
        if (tau <= 0.0) {
            z_ref = ascent_start_z_;
            vz_ref = 0.0;
        } else if (tau < ramp_duration_s_) {
            const double theta = kPi * tau / ramp_duration_s_;
            vz_ref = 0.5 * climb_v_peak_ * (1.0 - std::cos(theta));
            z_ref = ascent_start_z_ +
                    0.5 * climb_v_peak_ * (tau - (ramp_duration_s_ / kPi) * std::sin(theta));
        } else if (tau < ramp_duration_s_ + cruise_duration_s_) {
            vz_ref = climb_v_peak_;
            z_ref = ascent_start_z_ + 0.5 * climb_v_peak_ * ramp_duration_s_ +
                    climb_v_peak_ * (tau - ramp_duration_s_);
        } else {
            const double u = tau - (ramp_duration_s_ + cruise_duration_s_);
            const double theta = kPi * u / ramp_duration_s_;
            vz_ref = 0.5 * climb_v_peak_ * (1.0 + std::cos(theta));
            z_ref = ascent_start_z_ + 0.5 * climb_v_peak_ * ramp_duration_s_ +
                    climb_v_peak_ * cruise_duration_s_ +
                    0.5 * climb_v_peak_ * (u + (ramp_duration_s_ / kPi) * std::sin(theta));
        }
    }

    Setpoint& sp = controller_.getSetpoint();
    sp.z = std::clamp(z_ref, ascent_start_z_, target_altitude_);
    sp.vx = 0.0;
    sp.vy = 0.0;
    sp.vz = vz_ref;
    sp.ax = 0.0;
    sp.ay = 0.0;
    sp.az = 0.0;
    sp.yaw_rate = 0.0;
    sp.type_mask = kHoverPositionVelocityTypeMask;
    if (controller_.getConfig().enable_yaw_control) {
        sp.type_mask &= ~kIgnoreYawBit;
    }
}

bool TakeoffAscendingState::ascentProfileCompleted(double now_s) const {
    if (!profile_active_ || profile_duration_s_ <= 0.0) {
        return true;
    }
    return std::isfinite(now_s) && (now_s - ascent_start_time_s_) >= profile_duration_s_;
}

void TakeoffAscendingState::onTick() {
    publishSetpointIfDue();
    logStatusIfDue();
    if (postTimeoutIfNeeded()) {
        return;
    }
    updateAltitudeConfirmation();
    postAltitudeReachedOnce();
    return;
}

void TakeoffAscendingState::onEvent(FlightCommand command) {
    if (command == FlightCommand::Track) {
        controller_.requestCustom1Tracking();
        if (!controller_.custom1ReferenceReady()) {
            controller_.logWarn(
                "[TakeoffAscendingState] custom1 latched; waiting for ready initial reference");
        }
    }
    return;
}

void TakeoffAscendingState::publishSetpointIfDue() {
    if (shouldRunEvery(setpoint_publish_timer_, SETPOINT_PUBLISH_INTERVAL, true)) {
        if (profile_active_) {
            updateAscentSetpoint(controller_.getCurrentTime());
        }
        controller_.emitFlightEffect(FlightSetpoint{controller_.getCurrentTime()});
    }
}

void TakeoffAscendingState::logStatusIfDue() {
    if (status_log_timer_.elapsed().count() < 1.0) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    const double displayed_target_z =
        profile_active_ ? target_altitude_ : controller_.getSetpoint().z;
    controller_.logInfo(
        "[TakeoffAscendingState] Altitude: %.2f m (target: %.2f m, vz: "
        "%.2f m/s, settled frames: %d/%d)",
        sensor_checks::worldZ(sensor_data, controller_.getConfig().tracking_backend),
        displayed_target_z,
        sensor_checks::worldVz(sensor_data, controller_.getConfig().tracking_backend),
        confirmed_settled_frames_, CONSECUTIVE_SETTLED_FRAMES);
    status_log_timer_.reset();
}

bool TakeoffAscendingState::postTimeoutIfNeeded() {
    if (timeout_event_posted_) {
        return true;
    }

    const double elapsed_time =
        std::chrono::duration<double>(controller_.flightElapsed()).count();
    if (elapsed_time <= ascending_timeout_s_) {
        return false;
    }

    controller_.logWarn("[TakeoffAscendingState] Ascending timeout after %.1f seconds",
                        elapsed_time);
    timeout_event_posted_ = true;
    controller_.emitFlightSignal(FlightSignal::TakeoffTimeout);
    return true;
}

void TakeoffAscendingState::updateAltitudeConfirmation() {
    if (altitude_reached_event_posted_ || timeout_event_posted_) {
        return;
    }
    if (profile_active_ && !ascentProfileCompleted(controller_.getCurrentTime())) {
        confirmed_settled_frames_ = 0;
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    const auto backend = controller_.getConfig().tracking_backend;
    const bool sensor_updated = sensor_checks::isWorldPoseNew(sensor_data, backend);
    if (!sensor_updated) {
        return;
    }

    const double final_target_z =
        profile_active_ ? target_altitude_ : controller_.getSetpoint().z;
    // Two-sided: an overshoot apex (vz ~ 0 above the target) must not confirm, because Hover
    // latches the current altitude and would hold the overshoot.
    const bool altitude_ok =
        std::abs(sensor_checks::worldZ(sensor_data, backend) - final_target_z) <=
        ALTITUDE_THRESHOLD;
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

void TakeoffAscendingState::postAltitudeReachedOnce() {
    if (altitude_reached_event_posted_ || confirmed_settled_frames_ < CONSECUTIVE_SETTLED_FRAMES) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    controller_.logInfo(
        "[TakeoffAscendingState] Target altitude reached: %.2f m (%.1f s "
        "elapsed)",
        sensor_checks::worldZ(sensor_data, controller_.getConfig().tracking_backend),
        std::chrono::duration<double>(controller_.flightElapsed()).count());
    altitude_reached_event_posted_ = true;
    controller_.emitFlightSignal(FlightSignal::AltitudeReached);
}

void TakeoffAscendingState::onExit() {
    const auto& sensor_data = controller_.getSensorData();
    const double elapsed_time =
        std::chrono::duration<double>(controller_.flightElapsed()).count();
    controller_.logInfo(
        "[TakeoffAscendingState] Exiting (altitude: %.2f m, settled frames: "
        "%d/%d, elapsed: %.2f s)",
        sensor_checks::worldZ(sensor_data, controller_.getConfig().tracking_backend),
        confirmed_settled_frames_, CONSECUTIVE_SETTLED_FRAMES, elapsed_time);
    status_log_timer_.stop();
    setpoint_publish_timer_.stop();
    return;
}

}  // namespace px4_multirotor_controller
