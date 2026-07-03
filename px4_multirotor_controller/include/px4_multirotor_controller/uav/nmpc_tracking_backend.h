#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <vector>

#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"

namespace px4_multirotor_controller {

struct NmpcDebugData {
    bool valid{false};
    bool success{false};
    int solver_status{0};
    double solve_time_ms{0.0};
    double state_estimate_stamp_sec{0.0};
    double filter_inertial_stamp_sec{0.0};
    double filter_pose_stamp_sec{0.0};
    double last_vrpn_pose_stamp_sec{0.0};

    Se3StateVector state{Se3StateVector::Zero()};
    Se3StateVector reference{Se3StateVector::Zero()};
    Se3StateVector horizon_reference{Se3StateVector::Zero()};
    Se3ControlVector reference_control{Se3ControlVector::Zero()};
    Se3ControlVector optimal_control{Se3ControlVector::Zero()};

    Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_error{Eigen::Vector3d::Zero()};
    Eigen::Vector3d reference_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d body_rate_command{Eigen::Vector3d::Zero()};
    Eigen::Vector3d predicted_body_rate{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_acceleration_command{Eigen::Vector3d::Zero()};

    double normalized_thrust_raw{0.0};
    double normalized_thrust_command{0.0};
    double hover_thrust{0.0};
    double initial_hover_thrust{0.0};
    double thrust_actual_estimate{0.0};
    double last_commanded_specific_thrust{0.0};
    double effective_specific_thrust_min{0.0};
    double effective_specific_thrust_max{0.0};

    bool normalized_thrust_min_saturated{false};
    bool normalized_thrust_max_saturated{false};
    bool roll_rate_saturated{false};
    bool pitch_rate_saturated{false};
    bool yaw_rate_saturated{false};
    bool roll_alpha_saturated{false};
    bool pitch_alpha_saturated{false};
    bool yaw_alpha_saturated{false};
};

class UavNmpcTrackingBackend {
   public:
    void configure(const ControllerConfig& config);
    bool enter(const SensorData& sensor);
    bool compute(const SensorData& sensor, const MpcTrajectoryState& reference,
                 const ros::Time& now, AttitudeRateTarget& target);
    bool compute(const SensorData& sensor, const std::vector<Se3Reference>& references,
                 const ros::Time& now, AttitudeRateTarget& target);
    void exit();

    int status() const {
        return solver_.status();
    }
    double solveTimeMs() const {
        return solver_.solveTimeMs();
    }
    const std::array<Se3StateVector, UAV_NMPC_N + 1>& predictedStates() const {
        return solver_.predictedStates();
    }
    size_t predictedStateCount() const {
        return solver_.predictedStateCount();
    }
    const NmpcDebugData& lastDebugData() const {
        return last_debug_;
    }

   private:
    bool feedbackState(const SensorData& sensor, Se3StateVector& x0) const;
    std::vector<Se3Reference> buildReferenceHorizon(const MpcTrajectoryState& reference,
                                                    const ros::Time& now) const;
    Se3Reference sampleReference(const MpcTrajectoryState& reference, double dt) const;
    bool hoverThrustReady(const SensorData& sensor, const ros::Time& now) const;
    bool lockInputBounds(double hover_thrust);
    double mapSpecificThrustToNormalized(double specific_thrust, double hover_thrust) const;
    void ensureThrustActualEstimate(const Se3Reference& reference);
    void updateThrustActualEstimate(double commanded_specific_thrust);
    void ensureLastCommandedSpecificThrust(const Se3Reference& reference);
    void updateLastCommandedSpecificThrust(double commanded_specific_thrust);
    void ensureLastCommandedBodyRate(const Se3Reference& reference);
    void updateLastCommandedBodyRate(const Eigen::Vector3d& body_rate);

    ControllerConfig config_{};
    UavNmpcSolver solver_;

    bool entered_{false};
    bool input_bounds_locked_{false};
    double initial_hover_thrust_{0.0};
    double effective_specific_thrust_min_{0.0};
    double effective_specific_thrust_max_{0.0};
    bool thrust_actual_initialized_{false};
    double thrust_actual_estimate_{0.0};
    bool last_commanded_specific_thrust_initialized_{false};
    double last_commanded_specific_thrust_{0.0};
    bool last_commanded_body_rate_initialized_{false};
    Eigen::Vector3d last_commanded_body_rate_{Eigen::Vector3d::Zero()};

    ros::Time last_control_time_;
    ros::Time last_log_time_;
    NmpcDebugData last_debug_;
};

}  // namespace px4_multirotor_controller
