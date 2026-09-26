#pragma once

#include <xgc2_math/control/smc_tracking_controller.hpp>

#include "px4_multirotor_controller/common/time.h"
#include "px4_multirotor_controller/tracking/tracking_strategy.h"

namespace px4_multirotor_controller {

// Translational SMC only. computeSmcTracking returns world acceleration
// a_ref + feedback, without gravity. That acceleration is the PositionTarget
// command. This strategy does not call the geometric attitude loop, does not
// read hover thrust, and does not produce body rate or normalized thrust.
class SmcAccelerationStrategy final : public TrackingStrategy {
   public:
    void configure(const ControllerConfig& config) override;
    bool enter(const SensorData& sensor, const Time& now) override;
    void exit() override;
    bool update(const TrackingStrategyInput& input, TrackingStrategyResult& result) override;
    double period() const override;
    bool isAsync() const override {
        return false;
    }

   private:
    bool measuredMotion(const SensorData& sensor, Eigen::Vector3d& position,
                        Eigen::Vector3d& velocity) const;

    ControllerConfig config_{};
    xgc2_math::control::SmcTrackingConfig smc_config_{};
    bool entered_{false};
    Time last_log_time_;
};

}  // namespace px4_multirotor_controller
