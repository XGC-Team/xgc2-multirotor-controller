#include "px4_multirotor_controller/tracking/px4_local_raw_strategy.h"

#include <cmath>

namespace px4_multirotor_controller {
namespace {

constexpr uint16_t kIgnoreYawAndYawRateMask = (1U << 10U) | (1U << 11U);

bool finiteReference(const UavReferencePoint& reference) {
    return reference.position.array().isFinite().all() &&
           reference.velocity.array().isFinite().all() &&
           reference.acceleration.array().isFinite().all();
}

}  // namespace

void Px4LocalRawStrategy::configure(const ControllerConfig& config) {
    config_ = config;
}

bool Px4LocalRawStrategy::enter(const SensorData&, const ros::Time&) {
    entered_ = true;
    ROS_INFO("[Px4LocalRawStrategy] PX4 local raw pos+vel+acc tracking started");
    return true;
}

void Px4LocalRawStrategy::exit() {
    entered_ = false;
}

bool Px4LocalRawStrategy::update(const TrackingStrategyInput& input,
                                 TrackingStrategyResult& result) {
    result = TrackingStrategyResult{};
    if (!entered_) {
        result.message = "PX4 local raw strategy not entered";
        return false;
    }
    if (!finiteReference(input.reference)) {
        result.message = "reference contains non-finite pos/vel/acc";
        return false;
    }

    Setpoint setpoint;
    setpoint.x = input.reference.position.x();
    setpoint.y = input.reference.position.y();
    setpoint.z = input.reference.position.z();
    setpoint.vx = input.reference.velocity.x();
    setpoint.vy = input.reference.velocity.y();
    setpoint.vz = input.reference.velocity.z();
    setpoint.ax = input.reference.acceleration.x();
    setpoint.ay = input.reference.acceleration.y();
    setpoint.az = input.reference.acceleration.z();
    setpoint.qw = 1.0;
    setpoint.yaw_rate = 0.0;
    setpoint.coordinate_frame = 1;
    setpoint.type_mask = kIgnoreYawAndYawRateMask;

    result.local_setpoint = setpoint;
    result.output_kind = TrackingStrategyResult::OutputKind::LocalSetpoint;
    result.success = true;
    return true;
}

double Px4LocalRawStrategy::period() const {
    return config_.nmpc.control_period;
}

}  // namespace px4_multirotor_controller
