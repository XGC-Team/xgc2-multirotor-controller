#include "px4_multirotor_controller/uav/nmpc_tracking_backend.h"

#include <ros/console.h>

#include <algorithm>
#include <cmath>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"

namespace px4_multirotor_controller {
namespace {

constexpr double kMaxYawReferenceErrorRad = 0.01;
constexpr double kThrustTimeConstant = 0.15;
constexpr double kSaturationTolerance = 1.0e-6;

double yawFromStateVector(const Se3StateVector& x0) {
    const double qw = x0(6);
    const double qx = x0(7);
    const double qy = x0(8);
    const double qz = x0(9);
    if (!std::isfinite(qw) || !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz)) {
        return 0.0;
    }
    return quaternionToYaw(qx, qy, qz, qw);
}

void setReferenceYaw(Se3Reference& reference, double yaw) {
    const Eigen::Vector3d body_z = reference.state.attitude.toRotationMatrix().col(2);
    reference.state.attitude = Eigen::Quaterniond(rotationFromBodyZ(body_z, yaw));
    reference.state.attitude.normalize();
}

void recomputeReferenceBodyRates(std::vector<Se3Reference>& references, double stage_dt) {
    if (references.size() < 2U || !std::isfinite(stage_dt) || stage_dt <= 1e-9) {
        for (auto& reference : references) {
            reference.state.body_rate.setZero();
            reference.control.angular_acceleration.setZero();
        }
        return;
    }

    std::vector<Eigen::Vector3d> body_rates(references.size(), Eigen::Vector3d::Zero());
    for (size_t i = 0; i + 1U < references.size(); ++i) {
        body_rates[i] = bodyRateFromRotationDelta(references[i].state.attitude,
                                                  references[i + 1U].state.attitude, stage_dt);
    }
    body_rates.back() = body_rates[body_rates.size() - 2U];

    for (size_t i = 0; i < references.size(); ++i) {
        references[i].state.body_rate = body_rates[i];
        references[i].control.angular_acceleration.setZero();
    }
}

void holdCurrentYaw(std::vector<Se3Reference>& references, double yaw, double stage_dt) {
    for (auto& reference : references) {
        setReferenceYaw(reference, yaw);
    }
    recomputeReferenceBodyRates(references, stage_dt);
}

void limitYawAuthority(std::vector<Se3Reference>& references, double current_yaw, double stage_dt) {
    for (auto& reference : references) {
        const Eigen::Quaterniond& q = reference.state.attitude;
        const double desired_yaw = quaternionToYaw(q.x(), q.y(), q.z(), q.w());
        const double yaw_error = normalizeAngle(desired_yaw - current_yaw);
        const double limited_yaw_error =
            std::max(-kMaxYawReferenceErrorRad, std::min(kMaxYawReferenceErrorRad, yaw_error));
        setReferenceYaw(reference, normalizeAngle(current_yaw + limited_yaw_error));
    }
    recomputeReferenceBodyRates(references, stage_dt);
}

Eigen::Vector3d referenceAccelerationWorld(const Se3Reference& reference, double gravity) {
    const Eigen::Vector3d body_z = reference.state.attitude.toRotationMatrix().col(2);
    return body_z * reference.control.body_z_specific_force - gravity * Eigen::Vector3d::UnitZ();
}

Eigen::Vector3d equivalentAngularAcceleration(const Eigen::Vector3d& current_body_rate,
                                              const Eigen::Vector3d& body_rate_command,
                                              double time_constant) {
    if (!std::isfinite(time_constant) || time_constant <= 1.0e-9) {
        return Eigen::Vector3d::Zero();
    }
    return (body_rate_command - current_body_rate) / time_constant;
}

}  // namespace

void UavNmpcTrackingBackend::configure(const ControllerConfig& config) {
    config_ = config;
}

bool UavNmpcTrackingBackend::enter(const SensorData& sensor) {
    if (!solver_.configureAngularAccelerationWeights(config_.nmpc.angular_acceleration_weight) ||
        !solver_.initialize()) {
        ROS_ERROR(
            "[UavNmpcTrackingBackend] Cannot enter NMPC tracking: solver "
            "init failed");
        entered_ = false;
        return false;
    }

    solver_.resetWarmStart();
    last_control_time_ = ros::Time(0);
    last_log_time_ = ros::Time(0);
    input_bounds_locked_ = false;
    initial_hover_thrust_ = 0.0;
    effective_specific_thrust_min_ = 0.0;
    effective_specific_thrust_max_ = 0.0;
    thrust_actual_initialized_ = false;
    thrust_actual_estimate_ = config_.nmpc.gravity;
    last_commanded_specific_thrust_initialized_ = false;
    last_commanded_specific_thrust_ = config_.nmpc.gravity;
    last_commanded_body_rate_initialized_ = false;
    last_commanded_body_rate_.setZero();
    entered_ = true;

    ROS_INFO(
        "[UavNmpcTrackingBackend] NMPC tracking started "
        "(external reference required, hover_thrust=required)");
    return true;
}

bool UavNmpcTrackingBackend::compute(const SensorData& sensor, const MpcTrajectoryState& reference,
                                     const ros::Time& now, AttitudeRateTarget& target) {
    if (!entered_) {
        return false;
    }

    if (!reference.is_valid) {
        ROS_WARN_THROTTLE(1.0, "[UavNmpcTrackingBackend] Waiting for external reference");
        return false;
    }

    if (!last_control_time_.isZero() &&
        (now - last_control_time_).toSec() < config_.nmpc.control_period) {
        return false;
    }

    const auto refs = buildReferenceHorizon(reference, now);
    return compute(sensor, refs, now, target);
}

bool UavNmpcTrackingBackend::compute(const SensorData& sensor,
                                     const std::vector<Se3Reference>& references,
                                     const ros::Time& now, AttitudeRateTarget& target) {
    if (!entered_) {
        return false;
    }

    if (references.size() < static_cast<size_t>(UavNmpcSolver::horizonSteps()) + 2U) {
        ROS_WARN_THROTTLE(1.0, "[UavNmpcTrackingBackend] Reference horizon too short");
        return false;
    }

    Se3StateVector x0;
    if (!feedbackState(sensor, x0)) {
        ROS_WARN_THROTTLE(1.0, "[UavNmpcTrackingBackend] Waiting for feedback state");
        return false;
    }

    if (!hoverThrustReady(sensor, now)) {
        ROS_WARN_THROTTLE(1.0, "[UavNmpcTrackingBackend] Waiting for hover thrust estimate");
        return false;
    }
    if (!lockInputBounds(sensor.hover_thrust_estimate)) {
        ROS_WARN_THROTTLE(1.0, "[UavNmpcTrackingBackend] Invalid NMPC thrust bounds");
        return false;
    }

    std::vector<Se3Reference> tracking_references = references;
    const double current_yaw = yawFromStateVector(x0);
    const double stage_dt =
        config_.nmpc.prediction_horizon / static_cast<double>(UavNmpcSolver::horizonSteps());
    if (!config_.enable_yaw_control) {
        holdCurrentYaw(tracking_references, current_yaw, stage_dt);
    } else {
        limitYawAuthority(tracking_references, current_yaw, stage_dt);
    }

    ensureThrustActualEstimate(tracking_references.front());
    ensureLastCommandedSpecificThrust(tracking_references.front());
    ensureLastCommandedBodyRate(tracking_references.front());
    const double thrust_actual_before_solve = thrust_actual_estimate_;
    const double last_commanded_thrust_before_solve = last_commanded_specific_thrust_;
    const Eigen::Vector3d last_commanded_body_rate_before_solve = last_commanded_body_rate_;
    const bool success =
        solver_.solve(x0, thrust_actual_before_solve, last_commanded_thrust_before_solve,
                      last_commanded_body_rate_before_solve, tracking_references);
    last_debug_ = NmpcDebugData{};
    last_debug_.valid = true;
    last_debug_.success = success;
    last_debug_.solver_status = solver_.status();
    last_debug_.solve_time_ms = solver_.solveTimeMs();
    last_debug_.state_estimate_stamp_sec = sensor.uav_state_estimate_stamp;
    last_debug_.filter_inertial_stamp_sec = sensor.uav_state_filter_inertial_stamp;
    last_debug_.filter_pose_stamp_sec = sensor.uav_state_filter_pose_stamp;
    last_debug_.last_vrpn_pose_stamp_sec = sensor.uav_state_last_vrpn_pose_stamp;
    last_debug_.state = x0;
    last_debug_.reference = control::packState(tracking_references.front().state);
    last_debug_.horizon_reference =
        control::packState(tracking_references[UavNmpcSolver::horizonSteps()].state);
    last_debug_.reference_control = control::packControl(tracking_references.front().control);
    // The production OCP deliberately damps angular acceleration around zero.
    // Keep the debug topic aligned with the cost actually sent to acados;
    // trajectory-derived angular-acceleration feed-forward is not enabled.
    last_debug_.reference_control.segment<3>(1).setZero();
    last_debug_.position_error = x0.segment<3>(0) - last_debug_.reference.segment<3>(0);
    last_debug_.velocity_error = x0.segment<3>(3) - last_debug_.reference.segment<3>(3);
    last_debug_.omega_error = x0.segment<3>(10) - last_debug_.reference.segment<3>(10);
    last_debug_.reference_acceleration =
        referenceAccelerationWorld(tracking_references.front(), config_.nmpc.gravity);
    last_debug_.hover_thrust = sensor.hover_thrust_estimate;
    last_debug_.initial_hover_thrust = initial_hover_thrust_;
    last_debug_.thrust_actual_estimate = thrust_actual_before_solve;
    last_debug_.last_commanded_specific_thrust = last_commanded_thrust_before_solve;
    last_debug_.effective_specific_thrust_min = effective_specific_thrust_min_;
    last_debug_.effective_specific_thrust_max = effective_specific_thrust_max_;
    if (success) {
        const Se3ControlVector u = solver_.optimalControl();
        const Eigen::Vector3d body_rate = bodyRateCommandFromPredictedBodyRate(
            u.segment<3>(1), config_.nmpc.max_roll_pitch_body_rate, config_.nmpc.max_yaw_body_rate);
        const Eigen::Vector3d angular_acceleration = equivalentAngularAcceleration(
            x0.segment<3>(10), body_rate, config_.nmpc.body_rate_time_constant);
        target.body_rate_x = body_rate.x();
        target.body_rate_y = body_rate.y();
        target.body_rate_z = body_rate.z();
        target.thrust = mapSpecificThrustToNormalized(u(0), initial_hover_thrust_);
        updateLastCommandedSpecificThrust(u(0));
        updateLastCommandedBodyRate(body_rate);
        updateThrustActualEstimate(u(0));
        last_debug_.optimal_control = u;
        last_debug_.body_rate_command = body_rate;
        last_debug_.predicted_body_rate = solver_.predictedBodyRate();
        last_debug_.angular_acceleration_command = angular_acceleration;
        last_debug_.normalized_thrust_raw =
            initial_hover_thrust_ * (u(0) / std::max(config_.nmpc.gravity, 1.0e-9));
        last_debug_.normalized_thrust_command = target.thrust;
        last_debug_.normalized_thrust_min_saturated =
            last_debug_.normalized_thrust_raw <=
            config_.nmpc.normalized_thrust_min + kSaturationTolerance;
        last_debug_.normalized_thrust_max_saturated =
            last_debug_.normalized_thrust_raw >=
            config_.nmpc.normalized_thrust_max - kSaturationTolerance;
        last_debug_.roll_rate_saturated =
            std::abs(body_rate.x()) >= config_.nmpc.max_roll_pitch_body_rate - kSaturationTolerance;
        last_debug_.pitch_rate_saturated =
            std::abs(body_rate.y()) >= config_.nmpc.max_roll_pitch_body_rate - kSaturationTolerance;
        last_debug_.yaw_rate_saturated =
            std::abs(body_rate.z()) >= config_.nmpc.max_yaw_body_rate - kSaturationTolerance;
        last_debug_.roll_alpha_saturated =
            std::abs(angular_acceleration.x()) >=
            config_.nmpc.max_roll_pitch_angular_acceleration - kSaturationTolerance;
        last_debug_.pitch_alpha_saturated =
            std::abs(angular_acceleration.y()) >=
            config_.nmpc.max_roll_pitch_angular_acceleration - kSaturationTolerance;
        last_debug_.yaw_alpha_saturated =
            std::abs(angular_acceleration.z()) >=
            config_.nmpc.max_yaw_angular_acceleration - kSaturationTolerance;
    } else {
        const Se3StateVector ref0_x = control::packState(tracking_references.front().state);
        const Se3ControlVector ref0_u = control::packControl(tracking_references.front().control);
        const Se3StateVector ref1_x = control::packState(tracking_references[1].state);
        const Eigen::Vector3d position_error = x0.segment<3>(0) - ref0_x.segment<3>(0);
        ROS_WARN_THROTTLE(1.0,
                          "[UavNmpcTrackingBackend] input debug: x0_p=[%.3f %.3f %.3f] "
                          "x0_v=[%.3f %.3f %.3f] x0_q=[%.3f %.3f %.3f %.3f] "
                          "x0_w=[%.3f %.3f %.3f] ref0_p=[%.3f %.3f %.3f] "
                          "ref0_v=[%.3f %.3f %.3f] ref0_q=[%.3f %.3f %.3f %.3f] "
                          "ref0_w=[%.3f %.3f %.3f] ref0_u=[%.3f %.3f %.3f %.3f] "
                          "ref1_p=[%.3f %.3f %.3f] pos_err_norm=%.3f",
                          x0(0), x0(1), x0(2), x0(3), x0(4), x0(5), x0(6), x0(7), x0(8), x0(9),
                          x0(10), x0(11), x0(12), ref0_x(0), ref0_x(1), ref0_x(2), ref0_x(3),
                          ref0_x(4), ref0_x(5), ref0_x(6), ref0_x(7), ref0_x(8), ref0_x(9),
                          ref0_x(10), ref0_x(11), ref0_x(12), ref0_u(0), ref0_u(1), ref0_u(2),
                          ref0_u(3), ref1_x(0), ref1_x(1), ref1_x(2), position_error.norm());
    }

    if (config_.nmpc.enable_timing_log &&
        (last_log_time_.isZero() || (now - last_log_time_).toSec() >= config_.nmpc.log_period)) {
        const Se3ControlVector u = solver_.optimalControl();
        const Eigen::Vector3d omega_pred = solver_.predictedBodyRate();
        const double thrust_norm = mapSpecificThrustToNormalized(u(0), initial_hover_thrust_);
        const Eigen::Vector3d omega_cmd = bodyRateCommandFromPredictedBodyRate(
            u.segment<3>(1), config_.nmpc.max_roll_pitch_body_rate, config_.nmpc.max_yaw_body_rate);
        const Eigen::Vector3d alpha_cmd = equivalentAngularAcceleration(
            x0.segment<3>(10), omega_cmd, config_.nmpc.body_rate_time_constant);
        const Se3StateVector ref0_x = control::packState(tracking_references.front().state);
        const Se3ControlVector ref0_u = control::packControl(tracking_references.front().control);
        const Se3StateVector refn_x =
            control::packState(tracking_references[UavNmpcSolver::horizonSteps()].state);
        const Eigen::Vector3d pos_err = x0.segment<3>(0) - ref0_x.segment<3>(0);
        const Eigen::Vector3d vel_err = x0.segment<3>(3) - ref0_x.segment<3>(3);
        ROS_INFO(
            "[UavNmpcTrackingBackend] solve %.2f ms status=%d u=[%.3f %.3f "
            "%.3f %.3f] omega_cmd=[%.3f %.3f %.3f] omega_pred=[%.3f %.3f %.3f] hover=%.3f "
            "initial_hover=%.3f thrust_norm=%.3f thrust_bounds=[%.3f %.3f] thrust_actual=%.3f "
            "last_thrust_cmd=%.3f "
            "alpha_cmd=[%.3f %.3f %.3f] q_norm_err=%.3e success=%s "
            "x0_p=[%.3f %.3f %.3f] ref0_p=[%.3f %.3f %.3f] refN_p=[%.3f %.3f %.3f] "
            "e_p=[%.3f %.3f %.3f] e_v=[%.3f %.3f %.3f] ref0_u=[%.3f %.3f %.3f "
            "%.3f]",
            solver_.solveTimeMs(), solver_.status(), u(0), u(1), u(2), u(3), omega_cmd.x(),
            omega_cmd.y(), omega_cmd.z(), omega_pred.x(), omega_pred.y(), omega_pred.z(),
            sensor.hover_thrust_estimate, initial_hover_thrust_, thrust_norm,
            effective_specific_thrust_min_, effective_specific_thrust_max_,
            thrust_actual_before_solve, last_commanded_thrust_before_solve, alpha_cmd.x(),
            alpha_cmd.y(), alpha_cmd.z(), solver_.maxQuaternionNormError(),
            success ? "true" : "false", x0(0), x0(1), x0(2), ref0_x(0), ref0_x(1), ref0_x(2),
            refn_x(0), refn_x(1), refn_x(2), pos_err.x(), pos_err.y(), pos_err.z(), vel_err.x(),
            vel_err.y(), vel_err.z(), ref0_u(0), ref0_u(1), ref0_u(2), ref0_u(3));
        last_log_time_ = now;
    }

    last_control_time_ = now;
    return success;
}

void UavNmpcTrackingBackend::exit() {
    entered_ = false;
}

bool UavNmpcTrackingBackend::feedbackState(const SensorData& sensor, Se3StateVector& x0) const {
    if (!sensor_checks::isControlStateUsableForControl(sensor)) {
        return false;
    }

    Eigen::Vector3d position(sensor.x, sensor.y, sensor.z);
    Eigen::Vector3d velocity(sensor.vx, sensor.vy, sensor.vz);
    Eigen::Quaterniond attitude(sensor.qw, sensor.qx, sensor.qy, sensor.qz);

    if (!std::isfinite(attitude.norm()) || attitude.norm() < 1e-9) {
        return false;
    }
    attitude.normalize();

    x0.setZero();
    x0.segment<3>(0) = position;
    x0.segment<3>(3) = velocity;
    x0.segment<4>(6) = control::quaternionToVectorWxyz(attitude);
    x0.segment<3>(10) << sensor.wx, sensor.wy, sensor.wz;

    return control::isFinite(x0);
}

std::vector<Se3Reference> UavNmpcTrackingBackend::buildReferenceHorizon(
    const MpcTrajectoryState& reference, const ros::Time& now) const {
    std::vector<Se3Reference> refs;
    refs.reserve(static_cast<size_t>(UavNmpcSolver::horizonSteps()) + 2U);

    const double stage_dt =
        config_.nmpc.prediction_horizon / static_cast<double>(UavNmpcSolver::horizonSteps());
    for (int i = 0; i <= UavNmpcSolver::horizonSteps() + 1; ++i) {
        const double dt = (now + ros::Duration(i * stage_dt) - reference.planning_time).toSec();
        refs.push_back(sampleReference(reference, dt));
    }
    return refs;
}

Se3Reference UavNmpcTrackingBackend::sampleReference(const MpcTrajectoryState& reference,
                                                     double dt) const {
    Se3Reference sample;
    if (dt < 0.0) {
        dt = 0.0;
    }
    if (dt > config_.planning_period) {
        dt = config_.planning_period;
    }

    const Eigen::Vector3d position =
        reference.position_k + reference.velocity_k * dt + 0.5 * reference.acceleration_k * dt * dt;
    const Eigen::Vector3d velocity = reference.velocity_k + reference.acceleration_k * dt;

    Eigen::Quaterniond attitude(reference.qw, reference.qx, reference.qy, reference.qz);
    if (!std::isfinite(attitude.norm()) || attitude.norm() < 1e-9) {
        attitude = Eigen::Quaterniond::Identity();
    }
    attitude.normalize();

    sample.state.position = position;
    sample.state.velocity = velocity;
    sample.state.attitude = attitude;
    sample.state.body_rate << 0.0, 0.0, reference.yaw_rate;

    const Eigen::Vector3d thrust =
        reference.acceleration_k + config_.nmpc.gravity * Eigen::Vector3d::UnitZ();
    sample.control.body_z_specific_force = thrust.norm();
    sample.control.angular_acceleration.setZero();
    return sample;
}

bool UavNmpcTrackingBackend::hoverThrustReady(const SensorData& sensor,
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

bool UavNmpcTrackingBackend::lockInputBounds(double hover_thrust) {
    if (input_bounds_locked_) {
        return true;
    }
    if (!std::isfinite(hover_thrust) || hover_thrust <= 1e-6 || config_.nmpc.gravity <= 1e-6) {
        return false;
    }
    if (config_.nmpc.normalized_thrust_min < 0.0 ||
        config_.nmpc.normalized_thrust_max <= config_.nmpc.normalized_thrust_min ||
        config_.nmpc.normalized_thrust_max > 1.0) {
        return false;
    }

    initial_hover_thrust_ = hover_thrust;
    effective_specific_thrust_min_ =
        config_.nmpc.gravity * config_.nmpc.normalized_thrust_min / initial_hover_thrust_;
    effective_specific_thrust_max_ =
        config_.nmpc.gravity * config_.nmpc.normalized_thrust_max / initial_hover_thrust_;
    if (!solver_.configureInputBounds(
            effective_specific_thrust_min_, effective_specific_thrust_max_,
            config_.nmpc.max_roll_pitch_body_rate, config_.nmpc.max_yaw_body_rate,
            config_.nmpc.max_roll_pitch_angular_acceleration,
            config_.nmpc.max_yaw_angular_acceleration)) {
        initial_hover_thrust_ = 0.0;
        effective_specific_thrust_min_ = 0.0;
        effective_specific_thrust_max_ = 0.0;
        return false;
    }
    input_bounds_locked_ = true;
    ROS_INFO(
        "[UavNmpcTrackingBackend] Locked NMPC thrust bounds from hover=%.3f norm=[%.3f %.3f] "
        "specific=[%.3f %.3f]",
        initial_hover_thrust_, config_.nmpc.normalized_thrust_min,
        config_.nmpc.normalized_thrust_max, effective_specific_thrust_min_,
        effective_specific_thrust_max_);
    return true;
}

void UavNmpcTrackingBackend::ensureThrustActualEstimate(const Se3Reference& reference) {
    if (thrust_actual_initialized_ && std::isfinite(thrust_actual_estimate_)) {
        return;
    }
    double thrust_actual = reference.control.body_z_specific_force;
    if (!std::isfinite(thrust_actual) || thrust_actual <= 0.0) {
        thrust_actual = config_.nmpc.gravity;
    }
    thrust_actual_estimate_ = thrust_actual;
    thrust_actual_initialized_ = true;
}

void UavNmpcTrackingBackend::updateThrustActualEstimate(double commanded_specific_thrust) {
    if (!std::isfinite(commanded_specific_thrust)) {
        return;
    }
    if (!thrust_actual_initialized_ || !std::isfinite(thrust_actual_estimate_)) {
        thrust_actual_estimate_ = commanded_specific_thrust;
        thrust_actual_initialized_ = true;
        return;
    }
    const double dt = std::max(0.0, config_.nmpc.control_period);
    const double response = 1.0 - std::exp(-dt / kThrustTimeConstant);
    thrust_actual_estimate_ += response * (commanded_specific_thrust - thrust_actual_estimate_);
}

void UavNmpcTrackingBackend::ensureLastCommandedSpecificThrust(const Se3Reference& reference) {
    if (last_commanded_specific_thrust_initialized_ &&
        std::isfinite(last_commanded_specific_thrust_)) {
        return;
    }
    double commanded_thrust = reference.control.body_z_specific_force;
    if (!std::isfinite(commanded_thrust) || commanded_thrust <= 0.0) {
        commanded_thrust = config_.nmpc.gravity;
    }
    last_commanded_specific_thrust_ = commanded_thrust;
    last_commanded_specific_thrust_initialized_ = true;
}

void UavNmpcTrackingBackend::updateLastCommandedSpecificThrust(double commanded_specific_thrust) {
    if (!std::isfinite(commanded_specific_thrust)) {
        return;
    }
    last_commanded_specific_thrust_ = commanded_specific_thrust;
    last_commanded_specific_thrust_initialized_ = true;
}

void UavNmpcTrackingBackend::ensureLastCommandedBodyRate(const Se3Reference& reference) {
    if (last_commanded_body_rate_initialized_ &&
        last_commanded_body_rate_.array().isFinite().all()) {
        return;
    }
    Eigen::Vector3d body_rate = reference.state.body_rate;
    if (!body_rate.array().isFinite().all()) {
        body_rate.setZero();
    }
    last_commanded_body_rate_ = bodyRateCommandFromPredictedBodyRate(
        body_rate, config_.nmpc.max_roll_pitch_body_rate, config_.nmpc.max_yaw_body_rate);
    last_commanded_body_rate_initialized_ = true;
}

void UavNmpcTrackingBackend::updateLastCommandedBodyRate(const Eigen::Vector3d& body_rate) {
    if (!body_rate.array().isFinite().all()) {
        return;
    }
    last_commanded_body_rate_ = body_rate;
    last_commanded_body_rate_initialized_ = true;
}

double UavNmpcTrackingBackend::mapSpecificThrustToNormalized(double specific_thrust,
                                                             double hover_thrust) const {
    if (!std::isfinite(specific_thrust) || config_.nmpc.gravity <= 1e-6) {
        return 0.0;
    }
    if (!std::isfinite(hover_thrust) || hover_thrust < config_.nmpc.min_hover_thrust ||
        hover_thrust > config_.nmpc.max_hover_thrust) {
        return 0.0;
    }

    return clamp(hover_thrust * (specific_thrust / config_.nmpc.gravity),
                 config_.nmpc.normalized_thrust_min, config_.nmpc.normalized_thrust_max);
}

}  // namespace px4_multirotor_controller
