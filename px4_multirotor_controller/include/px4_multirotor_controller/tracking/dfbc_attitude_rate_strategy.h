#pragma once

#include <ros/ros.h>

#include <xgc2_math/control.hpp>

#include "px4_multirotor_controller/state_machine/timing.h"
#include "px4_multirotor_controller/tracking/tracking_strategy.h"

namespace px4_multirotor_controller {

class DfbcAttitudeRateStrategy final : public TrackingStrategy {
   public:
    void configure(const ControllerConfig& config) override;
    bool enter(const SensorData& sensor, const ros::Time& now) override;
    void exit() override;
    bool update(const TrackingStrategyInput& input, TrackingStrategyResult& result) override;
    double period() const override;
    bool isAsync() const override {
        return false;
    }

   private:
    bool feedbackState(const SensorData& sensor, xgc2_math::control::Se3State& state) const;
    static xgc2_math::trajectory::FlatOutput3 flatReference(const UavReferencePoint& reference);
    bool measuredAcceleration(const SensorData& sensor, const ros::Time& now,
                              Eigen::Vector3d& acceleration) const;
    bool hoverThrustReady(const SensorData& sensor, const ros::Time& now) const;
    double mapSpecificThrustToNormalized(double specific_thrust, double hover_thrust) const;
    Eigen::Vector3d clampBodyRate(const Eigen::Vector3d& body_rate) const;

    ControllerConfig config_{};
    xgc2_math::control::DfbcGeometricController controller_;
    bool entered_{false};
    ros::Time last_log_time_;
};

}  // namespace px4_multirotor_controller
