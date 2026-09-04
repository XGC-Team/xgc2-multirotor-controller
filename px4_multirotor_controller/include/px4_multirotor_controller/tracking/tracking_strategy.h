#pragma once

#include <ros/ros.h>

#include <string>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/uav/active_trajectory_cache.h"

namespace px4_multirotor_controller {

struct TrackingStrategyInput {
    SensorData sensor;
    UavReferencePoint reference;
    ros::Time now;
    ros::Time stamp;
    uint16_t type_mask{0};
};

struct TrackingStrategyResult {
    enum class OutputKind {
        AttitudeRate,
        LocalSetpoint,
    };

    bool success{false};
    std::string message;
    OutputKind output_kind{OutputKind::AttitudeRate};
    AttitudeRateTarget attitude_rate_target;
    Setpoint local_setpoint;
};

class TrackingStrategy {
   public:
    virtual ~TrackingStrategy() = default;
    virtual void configure(const ControllerConfig& config) = 0;
    virtual bool enter(const SensorData& sensor, const ros::Time& now) = 0;
    virtual void exit() = 0;
    virtual bool update(const TrackingStrategyInput& input, TrackingStrategyResult& result) = 0;
    virtual double period() const = 0;
    virtual bool isAsync() const = 0;
};

}  // namespace px4_multirotor_controller
