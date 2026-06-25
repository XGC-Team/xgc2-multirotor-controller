#include "px4_multirotor_controller/uav/nmpc_tracking_backend.h"

#include <ros/console.h>

#include <cmath>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"

namespace px4_multirotor_controller {

void UavNmpcTrackingBackend::configure(const ControllerConfig& config) {
    config_ = config;
}

bool UavNmpcTrackingBackend::enter(const SensorData& sensor) {
    if (!solver_.initialize()) {
        ROS_ERROR(
            "[UavNmpcTrackingBackend] Cannot enter NMPC tracking: solver "
            "init failed");
        entered_ = false;
        return false;
    }

    solver_.resetWarmStart();
    last_control_time_ = ros::Time(0);
    last_log_time_ = ros::Time(0);
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

    const bool success = solver_.solve(x0, references);
    if (success) {
        const Se3ControlVector u = solver_.optimalControl();
        const Eigen::Vector3d body_rate =
            x0.segment<3>(10) + u.tail<3>() * config_.nmpc.control_period;
        target.body_rate_x = body_rate.x();
        target.body_rate_y = body_rate.y();
        target.body_rate_z = body_rate.z();
        target.thrust = mapSpecificThrustToNormalized(u(0), sensor.hover_thrust_estimate);
    } else {
        const Se3StateVector ref0_x = control::packState(references.front().state);
        const Se3ControlVector ref0_u = control::packControl(references.front().control);
        const Se3StateVector ref1_x = control::packState(references[1].state);
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
        const Eigen::Vector3d omega_cmd =
            x0.segment<3>(10) + u.tail<3>() * config_.nmpc.control_period;
        const Se3StateVector ref0_x = control::packState(references.front().state);
        const Se3ControlVector ref0_u = control::packControl(references.front().control);
        const Se3StateVector refn_x =
            control::packState(references[UavNmpcSolver::horizonSteps()].state);
        const Eigen::Vector3d pos_err = x0.segment<3>(0) - ref0_x.segment<3>(0);
        const Eigen::Vector3d vel_err = x0.segment<3>(3) - ref0_x.segment<3>(3);
        ROS_INFO(
            "[UavNmpcTrackingBackend] solve %.2f ms status=%d u=[%.3f %.3f "
            "%.3f %.3f] omega_cmd=[%.3f %.3f %.3f] hover=%.3f "
            "q_norm_err=%.3e success=%s x0_p=[%.3f %.3f %.3f] "
            "ref0_p=[%.3f %.3f %.3f] refN_p=[%.3f %.3f %.3f] "
            "e_p=[%.3f %.3f %.3f] e_v=[%.3f %.3f %.3f] ref0_u=[%.3f %.3f %.3f "
            "%.3f]",
            solver_.solveTimeMs(), solver_.status(), u(0), u(1), u(2), u(3), omega_cmd.x(),
            omega_cmd.y(), omega_cmd.z(), sensor.hover_thrust_estimate,
            solver_.maxQuaternionNormError(), success ? "true" : "false", x0(0), x0(1), x0(2),
            ref0_x(0), ref0_x(1), ref0_x(2), refn_x(0), refn_x(1), refn_x(2), pos_err.x(),
            pos_err.y(), pos_err.z(), vel_err.x(), vel_err.y(), vel_err.z(), ref0_u(0), ref0_u(1),
            ref0_u(2), ref0_u(3));
        last_log_time_ = now;
    }

    last_control_time_ = now;
    return success;
}

void UavNmpcTrackingBackend::exit() {
    entered_ = false;
}

bool UavNmpcTrackingBackend::feedbackState(const SensorData& sensor, Se3StateVector& x0) const {
    if (!sensor_checks::isStateEstimateUsableForControl(sensor)) {
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

double UavNmpcTrackingBackend::mapSpecificThrustToNormalized(double specific_thrust,
                                                             double hover_thrust) const {
    if (!std::isfinite(specific_thrust) || config_.nmpc.gravity <= 1e-6) {
        return 0.0;
    }
    if (!std::isfinite(hover_thrust) || hover_thrust < config_.nmpc.min_hover_thrust ||
        hover_thrust > config_.nmpc.max_hover_thrust) {
        return 0.0;
    }

    double bounded_thrust = specific_thrust;
    if (config_.nmpc.specific_thrust_max > config_.nmpc.specific_thrust_min) {
        bounded_thrust = clamp(bounded_thrust, config_.nmpc.specific_thrust_min,
                               config_.nmpc.specific_thrust_max);
    }

    return clamp(hover_thrust * (bounded_thrust / config_.nmpc.gravity), 0.0, 1.0);
}

}  // namespace px4_multirotor_controller
