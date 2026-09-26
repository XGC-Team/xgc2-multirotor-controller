#include "px4_multirotor_controller/tracking/smc_acceleration_strategy.h"

#include "px4_multirotor_controller/common/core_log.h"
#include "px4_multirotor_controller/common/sensor_checks.h"

namespace px4_multirotor_controller {

void SmcAccelerationStrategy::configure(const ControllerConfig& config) {
    config_ = config;
    smc_config_.k1 = config.smc.k1;
    smc_config_.k2 = config.smc.k2;
    smc_config_.boundary_layer = config.smc.boundary_layer;
}

bool SmcAccelerationStrategy::enter(const SensorData& sensor, const Time&) {
    if (!sensor_checks::arePassThroughSensorsActive(sensor)) {
        PMC_LOG_WARN_THROTTLE(1.0,
                              "[SmcAccelerationStrategy] Waiting for MAVROS local position and "
                              "velocity");
        return false;
    }
    entered_ = true;
    last_log_time_ = Time();
    PMC_LOG_INFO("[SmcAccelerationStrategy] SMC acceleration tracking started");
    return true;
}

void SmcAccelerationStrategy::exit() {
    entered_ = false;
}

bool SmcAccelerationStrategy::update(const TrackingStrategyInput& input,
                                     TrackingStrategyResult& result) {
    result = TrackingStrategyResult{};
    if (!entered_) {
        result.message = "SMC strategy not entered";
        return false;
    }

    Eigen::Vector3d measured_position;
    Eigen::Vector3d measured_velocity;
    if (!measuredMotion(input.sensor, measured_position, measured_velocity)) {
        result.message = "MAVROS local position or velocity unavailable";
        return false;
    }
    if (!input.reference.position.allFinite() || !input.reference.velocity.allFinite() ||
        !input.reference.acceleration.allFinite()) {
        result.message = "reference PVA is not finite";
        return false;
    }

    const auto smc = xgc2_math::control::computeSmcTracking(
        smc_config_, measured_position, measured_velocity, input.reference.position,
        input.reference.velocity, input.reference.acceleration);
    if (!smc.success) {
        result.message = "SMC compute failed";
        return false;
    }

    Setpoint command;
    command.ax = smc.acceleration.x();
    command.ay = smc.acceleration.y();
    command.az = smc.acceleration.z();
    command.type_mask = kSmcAccelerationTypeMask;
    command.coordinate_frame = 1;
    result.local_setpoint = command;
    result.output_kind = TrackingStrategyResult::OutputKind::LocalSetpoint;
    result.success = true;

    if (config_.nmpc.enable_timing_log &&
        (last_log_time_.isZero() ||
         (input.now - last_log_time_).toSec() >= config_.dfbc.log_period)) {
        PMC_LOG_INFO(
            "[SmcAccelerationStrategy] a=[%.3f %.3f %.3f] e_p=[%.3f %.3f %.3f] "
            "e_v=[%.3f %.3f %.3f] s=[%.3f %.3f %.3f] u=[%.3f %.3f %.3f]",
            smc.acceleration.x(), smc.acceleration.y(), smc.acceleration.z(),
            smc.position_error.x(), smc.position_error.y(), smc.position_error.z(),
            smc.velocity_error.x(), smc.velocity_error.y(), smc.velocity_error.z(),
            smc.sliding.x(), smc.sliding.y(), smc.sliding.z(), smc.feedback.x(),
            smc.feedback.y(), smc.feedback.z());
        last_log_time_ = input.now;
    }
    return true;
}

double SmcAccelerationStrategy::period() const {
    return config_.nmpc.control_period;
}

bool SmcAccelerationStrategy::measuredMotion(const SensorData& sensor, Eigen::Vector3d& position,
                                             Eigen::Vector3d& velocity) const {
    if (!sensor_checks::arePassThroughSensorsActive(sensor)) {
        return false;
    }
    position << sensor.local_x, sensor.local_y, sensor.local_z;
    velocity << sensor.local_vx, sensor.local_vy, sensor.local_vz;
    return position.allFinite() && velocity.allFinite();
}

}  // namespace px4_multirotor_controller
