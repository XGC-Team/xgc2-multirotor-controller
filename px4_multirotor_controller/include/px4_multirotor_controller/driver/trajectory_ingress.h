#pragma once

#include <cmath>
#include <cstdint>

#include "px4_multirotor_controller/common/time.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

// The part of the planner setpoint and hover thrust inputs that decides whether and how a sample is
// used. The ROS input producer and the module call these, so both treat a sample the same way; each
// edge only maps its own message to the arguments and decides what to log.

enum class PlannerSetpointResult {
    kNotUsed,                // the tracking backend takes no planner setpoint
    kRejectedEffectiveTime,  // the backend needs a finite PVA and a header stamp
    kRejectedSmcFrame,       // SMC needs world frame 1 and every P/V/A axis, with FORCE clear
    kCached,                 // cached as the next trajectory segment: post the input event
};

// A PositionTarget from a planner (alg/setpoint_raw/local), received at ingress.receipt_time.
inline PlannerSetpointResult ingestPlannerSetpoint(DroneController& controller,
                                                   const PositionTargetIngress& ingress) {
    const ControllerConfig config = controller.getConfig();
    if (config.tracking_backend != TrackingBackend::PX4_LOCAL &&
        config.tracking_backend != TrackingBackend::SMC) {
        return PlannerSetpointResult::kNotUsed;
    }
    const MpcTrajectoryState trajectory =
        ingestPositionTarget(ingress, config.tracking_backend, config.px4_local_lift);
    if (usesStageEffectiveTime(config.tracking_backend, config.px4_local_lift) &&
        !trajectory.is_valid) {
        return config.tracking_backend == TrackingBackend::SMC &&
                       !smcWorldPvaAvailable(ingress.type_mask, ingress.coordinate_frame)
                   ? PlannerSetpointResult::kRejectedSmcFrame
                   : PlannerSetpointResult::kRejectedEffectiveTime;
    }
    cacheTrajectorySample(controller.mpcTrajectoryBuffer(), trajectory, ingress.receipt_time,
                          config);
    return PlannerSetpointResult::kCached;
}

// A hover thrust estimate. stamp_sec is the message stamp (zero when unstamped, then the receipt
// time stands in). Returns true when the estimate is usable: post the input event.
inline bool ingestHoverThrust(SensorData& sensor, double hover_thrust, uint32_t flags,
                              double stamp_sec, double receipt_sec) {
    if (!std::isfinite(hover_thrust) || hover_thrust <= 0.0 || hover_thrust >= 1.0) {
        sensor.hover_thrust_estimate_available = false;
        sensor.hover_thrust_estimate_flags = flags;
        return false;
    }
    sensor.hover_thrust_estimate = hover_thrust;
    sensor.hover_thrust_estimate_stamp = stamp_sec != 0.0 ? stamp_sec : receipt_sec;
    sensor.hover_thrust_estimate_available = true;
    sensor.hover_thrust_estimate_flags = flags;
    return true;
}

}  // namespace px4_multirotor_controller
