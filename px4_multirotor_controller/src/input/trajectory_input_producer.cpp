#include "px4_multirotor_controller/input/trajectory_input_producer.h"

#include <ros/ros.h>

#include <cmath>
#include <utility>

#include "px4_multirotor_controller/driver/trajectory_ingress.h"
#include "px4_multirotor_controller/ros_reference_conversion.h"
#include "px4_multirotor_controller/ros_time_conversion.h"

namespace px4_multirotor_controller {

TrajectoryInputProducer::TrajectoryInputProducer(ros::NodeHandle& nh, DroneController& controller,
                                                 SensorData& sensor_data, EventSink event_sink,
                                                 uint32_t queue_size)
    : controller_(controller),
      sensor_data_(sensor_data),
      event_sink_(std::move(event_sink)) {
    alg_setpoint_sub_ = nh.subscribe("alg/setpoint_raw/local", queue_size,
                                     &TrajectoryInputProducer::algSetpointCallback, this);
    active_analytic_sub_ =
        nh.subscribe("alg/multirotor_reference_trajectory/active/analytic", queue_size,
                     &TrajectoryInputProducer::activeAnalyticCallback, this);
    active_sampled_sub_ =
        nh.subscribe("alg/multirotor_reference_trajectory/active/sampled", queue_size,
                     &TrajectoryInputProducer::activeSampledCallback, this);
    hover_thrust_sub_ = nh.subscribe("hover_thrust/estimate_state", queue_size,
                                     &TrajectoryInputProducer::hoverThrustCallback, this);
}

void TrajectoryInputProducer::algSetpointCallback(
    const mavros_msgs::PositionTarget::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[TrajectoryInputProducer] Received null alg setpoint message");
        return;
    }
    PositionTargetIngress ingress;
    ingress.position << msg->position.x, msg->position.y, msg->position.z;
    ingress.velocity << msg->velocity.x, msg->velocity.y, msg->velocity.z;
    ingress.acceleration << msg->acceleration_or_force.x, msg->acceleration_or_force.y,
        msg->acceleration_or_force.z;
    ingress.yaw = msg->yaw;
    ingress.yaw_rate = msg->yaw_rate;
    ingress.type_mask = msg->type_mask;
    ingress.coordinate_frame = msg->coordinate_frame;
    ingress.header_stamp = msg->header.stamp.isZero() ? Time() : toCoreTime(msg->header.stamp);
    ingress.receipt_time = toCoreTime(ros::Time::now());
    switch (ingestPlannerSetpoint(controller_, ingress)) {
        case PlannerSetpointResult::kNotUsed:
            return;
        case PlannerSetpointResult::kRejectedSmcFrame:
            ROS_WARN_THROTTLE(1.0,
                              "[TrajectoryInputProducer] SMC rejected PositionTarget: need world "
                              "frame 1 and every P/V/A axis, with FORCE clear");
            return;
        case PlannerSetpointResult::kRejectedEffectiveTime:
            ROS_WARN_THROTTLE(1.0,
                              "[TrajectoryInputProducer] effective-time setpoint needs a finite "
                              "PVA and a non-zero header stamp");
            return;
        case PlannerSetpointResult::kCached:
            postInputEvent(event_type::INPUT_MPC_TRAJECTORY_UPDATED, "alg/setpoint_raw/local");
            return;
    }
}

void TrajectoryInputProducer::activeAnalyticCallback(
    const multirotor_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[TrajectoryInputProducer] Received null active analytic trajectory");
        return;
    }
    if (!controller_.activeTrajectoryCache().updateAnalytic(toCoreReference(*msg),
                                                 toCoreTime(ros::Time::now()))) {
        ROS_WARN_THROTTLE(1.0, "[TrajectoryInputProducer] Rejected active analytic trajectory");
        return;
    }
    postInputEvent(event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED,
                   "alg/multirotor_reference_trajectory/active/analytic");
}

void TrajectoryInputProducer::activeSampledCallback(
    const multirotor_reference_trajectory_msgs::SampledReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[TrajectoryInputProducer] Received null active sampled trajectory");
        return;
    }
    if (!controller_.activeTrajectoryCache().updateSampled(toCoreReference(*msg),
                                                toCoreTime(ros::Time::now()))) {
        ROS_WARN_THROTTLE(1.0, "[TrajectoryInputProducer] Rejected active sampled trajectory");
        return;
    }
    postInputEvent(event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED,
                   "alg/multirotor_reference_trajectory/active/sampled");
}

void TrajectoryInputProducer::hoverThrustCallback(
    const hover_thrust_estimator_msgs::HoverThrustEstimate::ConstPtr& msg) {
    // No message is an unusable estimate, like one that is out of range.
    const bool usable = ingestHoverThrust(
        sensor_data_, msg ? msg->hover_thrust : std::nan(""), msg ? msg->flags : 0U,
        msg ? msg->header.stamp.toSec() : 0.0, ros::Time::now().toSec());
    if (usable) {
        postInputEvent(event_type::INPUT_HOVER_THRUST_UPDATED, "hover_thrust/estimate_state");
    }
}

void TrajectoryInputProducer::postInputEvent(::state_machine::EventId event_id,
                                             const char* source) {
    if (!event_sink_) {
        ROS_ERROR("[TrajectoryInputProducer] Event sink is not configured");
        return;
    }

    ::state_machine::Event event(event_id,
                                 ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = source;
    const auto status = event_sink_(std::move(event));
    if (!status.ok()) {
        ROS_ERROR_THROTTLE(1.0,
                           "[TrajectoryInputProducer] Failed to post input event "
                           "%u from %s: %s",
                           event_id, source, status.message.c_str());
    }
}

}  // namespace px4_multirotor_controller
