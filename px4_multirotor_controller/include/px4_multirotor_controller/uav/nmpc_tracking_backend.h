#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
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

   private:
    bool feedbackState(const SensorData& sensor, Se3StateVector& x0) const;
    std::vector<Se3Reference> buildReferenceHorizon(const MpcTrajectoryState& reference,
                                                    const ros::Time& now) const;
    Se3Reference sampleReference(const MpcTrajectoryState& reference, double dt) const;
    bool hoverThrustReady(const SensorData& sensor, const ros::Time& now) const;
    double mapSpecificThrustToNormalized(double specific_thrust, double hover_thrust) const;

    ControllerConfig config_{};
    UavNmpcSolver solver_;

    bool entered_{false};

    ros::Time last_control_time_;
    ros::Time last_log_time_;
};

}  // namespace px4_multirotor_controller
