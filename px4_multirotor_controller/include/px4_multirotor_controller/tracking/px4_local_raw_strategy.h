#pragma once

#include "px4_multirotor_controller/tracking/tracking_strategy.h"

namespace px4_multirotor_controller {

class Px4LocalRawStrategy final : public TrackingStrategy {
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
    ControllerConfig config_{};
    bool entered_{false};
};

}  // namespace px4_multirotor_controller
