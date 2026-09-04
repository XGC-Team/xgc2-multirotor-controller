#include "px4_multirotor_controller/tracking/px4_local_raw_strategy.h"

#include "px4_multirotor_controller/control/trajectory_lifter.h"

namespace px4_multirotor_controller {

void Px4LocalRawStrategy::configure(const ControllerConfig& config) {
    config_ = config;
}

bool Px4LocalRawStrategy::enter(const SensorData&, const ros::Time&) {
    entered_ = true;
    ROS_INFO("[Px4LocalStrategy] world-frame PV/PVA pass-through started");
    return true;
}

void Px4LocalRawStrategy::exit() {
    entered_ = false;
}

bool Px4LocalRawStrategy::update(const TrackingStrategyInput& input,
                                 TrackingStrategyResult& result) {
    result = TrackingStrategyResult{};
    if (!entered_) {
        result.message = "PX4 local strategy not entered";
        return false;
    }
    if (!input.reference.position.array().isFinite().all() ||
        !input.reference.velocity.array().isFinite().all() ||
        !input.reference.acceleration.array().isFinite().all()) {
        result.message = "reference contains non-finite pos/vel/acc";
        return false;
    }

    MpcTrajectoryState sample;
    sample.position_k = input.reference.position;
    sample.velocity_k = input.reference.velocity;
    sample.acceleration_k = input.reference.acceleration;
    sample.planning_time = input.stamp.isZero() ? input.now : input.stamp;
    sample.type_mask = input.type_mask;
    sample.coordinate_frame = 1;
    sample.is_valid = true;

    result.local_setpoint =
        liftWorldLocal(sample, input.now, config_.local_type_mask, config_.enable_yaw_control);
    result.output_kind = TrackingStrategyResult::OutputKind::LocalSetpoint;
    result.success = true;
    return true;
}

double Px4LocalRawStrategy::period() const {
    return config_.nmpc.control_period;
}

}  // namespace px4_multirotor_controller
