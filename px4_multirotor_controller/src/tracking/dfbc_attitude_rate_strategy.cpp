#include "px4_multirotor_controller/tracking/dfbc_attitude_rate_strategy.h"

#include <algorithm>
#include <cmath>

#include "px4_multirotor_controller/common/sensor_checks.h"

namespace px4_multirotor_controller {
namespace {

double clampScalar(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

}  // namespace

void DfbcAttitudeRateStrategy::configure(const ControllerConfig& config) {
    config_ = config;

    xgc2_math::control::DfbcGeometricConfig dfbc_config;
    dfbc_config.position_natural_frequency = config.dfbc.position_natural_frequency;
    dfbc_config.position_damping_ratio = config.dfbc.position_damping_ratio;
    dfbc_config.tilt_gain = config.dfbc.tilt_gain;
    dfbc_config.tilt_rate_damping = config.dfbc.tilt_rate_damping;
    dfbc_config.yaw_gain = config.dfbc.yaw_gain;
    dfbc_config.yaw_rate_damping = config.dfbc.yaw_rate_damping;
    dfbc_config.gravity = config.nmpc.gravity;
    dfbc_config.min_specific_thrust = 0.1;
    dfbc_config.use_body_rate_feedforward = config.dfbc.use_body_rate_feedforward;
    dfbc_config.enable_yaw_control = config.enable_yaw_control;
    dfbc_config.acceleration_correction_enabled = config.dfbc.acceleration_correction_enabled;
    dfbc_config.acceleration_correction_gain = config.dfbc.acceleration_correction_gain;
    dfbc_config.acceleration_correction_limit = config.dfbc.acceleration_correction_limit;
    dfbc_config.acceleration_correction_filter_tau = config.dfbc.acceleration_correction_filter_tau;
    controller_.configure(dfbc_config);
}

bool DfbcAttitudeRateStrategy::enter(const SensorData& sensor, const ros::Time& now) {
    if (!hoverThrustReady(sensor, now)) {
        ROS_WARN_THROTTLE(1.0, "[DfbcAttitudeRateStrategy] Waiting for hover thrust estimate");
        return false;
    }
    entered_ = true;
    last_log_time_ = ros::Time();
    controller_.reset();
    ROS_INFO("[DfbcAttitudeRateStrategy] DFBC attitude-rate tracking started");
    return true;
}

void DfbcAttitudeRateStrategy::exit() {
    entered_ = false;
    controller_.reset();
}

bool DfbcAttitudeRateStrategy::update(const TrackingStrategyInput& input,
                                      TrackingStrategyResult& result) {
    result = TrackingStrategyResult{};
    if (!entered_) {
        result.message = "DFBC strategy not entered";
        return false;
    }
    if (!hoverThrustReady(input.sensor, input.now)) {
        result.message = "hover thrust estimate unavailable";
        return false;
    }

    xgc2_math::control::Se3State current;
    if (!feedbackState(input.sensor, current)) {
        result.message = "feedback state unavailable";
        return false;
    }

    xgc2_math::control::DfbcGeometricInput dfbc_input;
    dfbc_input.current = current;
    dfbc_input.reference = flatReference(input.reference);
    dfbc_input.dt = period();
    dfbc_input.has_measured_acceleration =
        measuredAcceleration(input.sensor, input.now, dfbc_input.measured_acceleration);
    const auto dfbc_output = controller_.compute(dfbc_input);
    if (!dfbc_output.success) {
        result.message = "DFBC compute failed";
        return false;
    }

    const Eigen::Vector3d body_rate = clampBodyRate(dfbc_output.body_rate_command);
    const double normalized_thrust = mapSpecificThrustToNormalized(
        dfbc_output.specific_thrust, input.sensor.hover_thrust_estimate);
    if (!std::isfinite(normalized_thrust) || normalized_thrust <= 0.0) {
        result.message = "normalized thrust unavailable";
        return false;
    }

    result.attitude_rate_target.body_rate_x = body_rate.x();
    result.attitude_rate_target.body_rate_y = body_rate.y();
    result.attitude_rate_target.body_rate_z = config_.enable_yaw_control ? body_rate.z() : 0.0;
    result.attitude_rate_target.thrust = normalized_thrust;
    result.success = true;

    if (config_.nmpc.enable_timing_log &&
        (last_log_time_.isZero() ||
         (input.now - last_log_time_).toSec() >= config_.dfbc.log_period)) {
        ROS_INFO(
            "[DfbcAttitudeRateStrategy] thrust=%.3f thrust_norm=%.3f "
            "omega_cmd=[%.3f %.3f %.3f] e_p=[%.3f %.3f %.3f] "
            "e_v=[%.3f %.3f %.3f] e_tilt=[%.3f %.3f %.3f] e_yaw=%.3f",
            dfbc_output.specific_thrust, normalized_thrust, body_rate.x(), body_rate.y(),
            body_rate.z(), dfbc_output.position_error.x(), dfbc_output.position_error.y(),
            dfbc_output.position_error.z(), dfbc_output.velocity_error.x(),
            dfbc_output.velocity_error.y(), dfbc_output.velocity_error.z(),
            dfbc_output.thrust_direction_error.x(), dfbc_output.thrust_direction_error.y(),
            dfbc_output.thrust_direction_error.z(), dfbc_output.yaw_error);
        if (dfbc_output.acceleration_correction_active) {
            ROS_INFO(
                "[DfbcAttitudeRateStrategy] accel_fix a_nom=[%.3f %.3f %.3f] "
                "a_meas=[%.3f %.3f %.3f] e_a_lpf=[%.3f %.3f %.3f] "
                "a_corr=[%.3f %.3f %.3f] a_cmd=[%.3f %.3f %.3f]",
                dfbc_output.nominal_acceleration.x(), dfbc_output.nominal_acceleration.y(),
                dfbc_output.nominal_acceleration.z(), dfbc_output.measured_acceleration.x(),
                dfbc_output.measured_acceleration.y(), dfbc_output.measured_acceleration.z(),
                dfbc_output.acceleration_error.x(), dfbc_output.acceleration_error.y(),
                dfbc_output.acceleration_error.z(), dfbc_output.acceleration_correction.x(),
                dfbc_output.acceleration_correction.y(), dfbc_output.acceleration_correction.z(),
                dfbc_output.corrected_acceleration.x(), dfbc_output.corrected_acceleration.y(),
                dfbc_output.corrected_acceleration.z());
        }
        last_log_time_ = input.now;
    }
    return true;
}

double DfbcAttitudeRateStrategy::period() const {
    return config_.nmpc.control_period;
}

bool DfbcAttitudeRateStrategy::feedbackState(const SensorData& sensor,
                                             xgc2_math::control::Se3State& state) {
    if (!sensor_checks::isStateEstimateUsableForControl(sensor)) {
        return false;
    }
    state.position << sensor.x, sensor.y, sensor.z;
    state.velocity << sensor.vx, sensor.vy, sensor.vz;
    state.attitude = Eigen::Quaterniond(sensor.qw, sensor.qx, sensor.qy, sensor.qz);
    if (!std::isfinite(state.attitude.norm()) || state.attitude.norm() < 1.0e-9) {
        return false;
    }
    state.attitude.normalize();
    state.body_rate << sensor.wx, sensor.wy, sensor.wz;
    return state.position.array().isFinite().all() && state.velocity.array().isFinite().all() &&
           state.body_rate.array().isFinite().all();
}

bool DfbcAttitudeRateStrategy::measuredAcceleration(const SensorData& sensor, const ros::Time& now,
                                                    Eigen::Vector3d& acceleration) const {
    if (!config_.dfbc.acceleration_correction_enabled) {
        return false;
    }
    if (!std::isfinite(sensor.uav_state_estimate_stamp)) {
        return false;
    }
    const double age = now.toSec() - sensor.uav_state_estimate_stamp;
    if (!std::isfinite(age) || age < -0.05 || age > config_.dfbc.acceleration_measurement_timeout) {
        return false;
    }
    acceleration << sensor.ax, sensor.ay, sensor.az;
    return acceleration.array().isFinite().all();
}

xgc2_math::trajectory::FlatOutput3 DfbcAttitudeRateStrategy::flatReference(
    const UavReferencePoint& reference) {
    xgc2_math::trajectory::FlatOutput3 flat;
    flat.position = reference.position;
    flat.velocity = reference.velocity;
    flat.acceleration = reference.acceleration;
    flat.jerk = reference.jerk;
    flat.snap = reference.snap;
    flat.yaw = reference.yaw;
    flat.yaw_rate = reference.yaw_rate;
    flat.yaw_accel = reference.yaw_accel;
    flat.flags = reference.flags;
    return flat;
}

bool DfbcAttitudeRateStrategy::hoverThrustReady(const SensorData& sensor,
                                                const ros::Time& now) const {
    if (!config_.nmpc.hover_thrust_enabled) {
        return false;
    }
    if (!sensor.hover_thrust_estimate_available || !std::isfinite(sensor.hover_thrust_estimate) ||
        sensor.hover_thrust_estimate < config_.nmpc.min_hover_thrust ||
        sensor.hover_thrust_estimate > config_.nmpc.max_hover_thrust ||
        !std::isfinite(sensor.hover_thrust_estimate_stamp)) {
        return false;
    }

    const double age = now.toSec() - sensor.hover_thrust_estimate_stamp;
    return std::isfinite(age) && age >= -0.05 && age <= config_.nmpc.hover_thrust_timeout;
}

double DfbcAttitudeRateStrategy::mapSpecificThrustToNormalized(double specific_thrust,
                                                               double hover_thrust) const {
    if (!std::isfinite(specific_thrust) || config_.nmpc.gravity <= 1.0e-6 ||
        !std::isfinite(hover_thrust) || hover_thrust < config_.nmpc.min_hover_thrust ||
        hover_thrust > config_.nmpc.max_hover_thrust) {
        return 0.0;
    }
    return clampScalar(hover_thrust * (specific_thrust / config_.nmpc.gravity),
                       config_.nmpc.normalized_thrust_min, config_.nmpc.normalized_thrust_max);
}

Eigen::Vector3d DfbcAttitudeRateStrategy::clampBodyRate(const Eigen::Vector3d& body_rate) const {
    Eigen::Vector3d clamped = body_rate;
    const double rp_limit = std::max(0.0, config_.nmpc.max_roll_pitch_body_rate);
    const double yaw_limit = std::max(0.0, config_.nmpc.max_yaw_body_rate);
    clamped.x() = clampScalar(clamped.x(), -rp_limit, rp_limit);
    clamped.y() = clampScalar(clamped.y(), -rp_limit, rp_limit);
    clamped.z() = clampScalar(clamped.z(), -yaw_limit, yaw_limit);
    return clamped;
}

}  // namespace px4_multirotor_controller
