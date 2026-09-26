#include "px4_multirotor_controller/input/trajectory_input_producer.h"
#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/ros_reference_conversion.h"
#include "px4_multirotor_controller/ros_time_conversion.h"

#include <ros/ros.h>

#include <cmath>
#include <utility>

#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"

namespace px4_multirotor_controller {

TrajectoryInputProducer::TrajectoryInputProducer(ros::NodeHandle& nh, SensorData& sensor_data,
                                                 ActiveTrajectoryCache& active_trajectory_cache,
                                                 ConfigProvider config_provider,
                                                 EventSink event_sink,
                                                 TrajectorySink trajectory_sink,
                                                 uint32_t queue_size)
    : sensor_data_(sensor_data),
      active_trajectory_cache_(active_trajectory_cache),
      config_provider_(std::move(config_provider)),
      event_sink_(std::move(event_sink)),
      trajectory_sink_(std::move(trajectory_sink)) {
    alg_setpoint_sub_ = nh.subscribe("alg/setpoint_raw/local", queue_size,
                                     &TrajectoryInputProducer::algSetpointCallback, this);
    active_analytic_sub_ =
        nh.subscribe("alg/multirotor_reference_trajectory/active/analytic", queue_size,
                     &TrajectoryInputProducer::activeAnalyticCallback, this);
    active_polynomial_sub_ =
        nh.subscribe("alg/multirotor_reference_trajectory/active/polynomial", queue_size,
                     &TrajectoryInputProducer::activePolynomialCallback, this);
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
    const ControllerConfig config = config_provider_ ? config_provider_() : ControllerConfig{};
    if (config.tracking_backend != TrackingBackend::PX4_LOCAL &&
        config.tracking_backend != TrackingBackend::SMC) {
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
    ingress.header_stamp =
        msg->header.stamp.isZero() ? Time() : toCoreTime(msg->header.stamp);
    ingress.receipt_time = toCoreTime(ros::Time::now());
    const MpcTrajectoryState traj =
        ingestPositionTarget(ingress, config.tracking_backend, config.px4_local_lift);
    if (usesStageEffectiveTime(config.tracking_backend, config.px4_local_lift) && !traj.is_valid) {
        if (config.tracking_backend == TrackingBackend::SMC &&
            (!smcCoordinateFrameIsWorld(ingress.coordinate_frame) ||
             (ingress.type_mask != 0U && !smcMaskSuppliesWorldPva(ingress.type_mask)))) {
            ROS_WARN_THROTTLE(1.0,
                              "[TrajectoryInputProducer] SMC rejected PositionTarget: need world "
                              "frame 1 and every P/V/A axis, with FORCE clear");
        } else {
            ROS_WARN_THROTTLE(1.0,
                              "[TrajectoryInputProducer] effective-time setpoint needs a finite "
                              "PVA and a non-zero header stamp");
        }
        return;
    }
    if (trajectory_sink_) {
        trajectory_sink_(traj);
    }
    postInputEvent(event_type::INPUT_MPC_TRAJECTORY_UPDATED, "alg/setpoint_raw/local");
}

void TrajectoryInputProducer::activeAnalyticCallback(
    const multirotor_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[TrajectoryInputProducer] Received null active analytic trajectory");
        return;
    }
    if (!active_trajectory_cache_.updateAnalytic(toCoreReference(*msg), toCoreTime(ros::Time::now()))) {
        ROS_WARN_THROTTLE(1.0, "[TrajectoryInputProducer] Rejected active analytic trajectory");
        return;
    }
    postInputEvent(event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED,
                   "alg/multirotor_reference_trajectory/active/analytic");
}

void TrajectoryInputProducer::activePolynomialCallback(
    const multirotor_reference_trajectory_msgs::ActivePolynomialReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[TrajectoryInputProducer] Received null active polynomial trajectory");
        return;
    }
    if (!active_trajectory_cache_.updatePolynomial(toCoreReference(*msg), toCoreTime(ros::Time::now()))) {
        ROS_WARN_THROTTLE(1.0, "[TrajectoryInputProducer] Rejected active polynomial trajectory");
        return;
    }
    postInputEvent(event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED,
                   "alg/multirotor_reference_trajectory/active/polynomial");
}

void TrajectoryInputProducer::activeSampledCallback(
    const multirotor_reference_trajectory_msgs::SampledReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[TrajectoryInputProducer] Received null active sampled trajectory");
        return;
    }
    if (!active_trajectory_cache_.updateSampled(toCoreReference(*msg), toCoreTime(ros::Time::now()))) {
        ROS_WARN_THROTTLE(1.0, "[TrajectoryInputProducer] Rejected active sampled trajectory");
        return;
    }
    postInputEvent(event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED,
                   "alg/multirotor_reference_trajectory/active/sampled");
}

void TrajectoryInputProducer::hoverThrustCallback(
    const hover_thrust_estimator_msgs::HoverThrustEstimate::ConstPtr& msg) {
    if (!msg || !std::isfinite(msg->hover_thrust) || msg->hover_thrust <= 0.0 ||
        msg->hover_thrust >= 1.0) {
        sensor_data_.hover_thrust_estimate_available = false;
        sensor_data_.hover_thrust_estimate_flags = msg ? msg->flags : 0U;
        return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    sensor_data_.hover_thrust_estimate = msg->hover_thrust;
    sensor_data_.hover_thrust_estimate_stamp = stamp.toSec();
    sensor_data_.hover_thrust_estimate_available = true;
    sensor_data_.hover_thrust_estimate_flags = msg->flags;
    postInputEvent(event_type::INPUT_HOVER_THRUST_UPDATED, "hover_thrust/estimate_state");
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
