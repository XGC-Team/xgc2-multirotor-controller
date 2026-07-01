#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <vector>

#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"

namespace px4_multirotor_controller {

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

    ros::Time last_control_time_;
    ros::Time last_log_time_;
};

}  // namespace px4_multirotor_controller
