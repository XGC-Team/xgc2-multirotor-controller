#include "px4_multirotor_controller/input/trajectory_input_producer.h"

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
    if (config.tracking_backend == TrackingBackend::NMPC_ATTITUDE_RATE) {
        return;
    }

    MpcTrajectoryState traj;
    traj.position_k.x() = msg->position.x;
    traj.position_k.y() = msg->position.y;
    traj.position_k.z() = msg->position.z;
    traj.velocity_k.x() = msg->velocity.x;
    traj.velocity_k.y() = msg->velocity.y;
    traj.velocity_k.z() = msg->velocity.z;
    traj.acceleration_k.x() = msg->acceleration_or_force.x;
    traj.acceleration_k.y() = msg->acceleration_or_force.y;
    traj.acceleration_k.z() = msg->acceleration_or_force.z;
    traj.planning_time = msg->header.stamp;
    const Eigen::Quaterniond yaw_quat = yawToQuaternion(msg->yaw);
    traj.qx = yaw_quat.x();
    traj.qy = yaw_quat.y();
    traj.qz = yaw_quat.z();
    traj.qw = yaw_quat.w();
    traj.yaw_rate = msg->yaw_rate;
    traj.type_mask = msg->type_mask;
    traj.coordinate_frame = msg->coordinate_frame;
    traj.is_valid = true;
    traj.new_data_received = false;
    if (trajectory_sink_) {
        trajectory_sink_(traj);
    }
    postInputEvent(event_type::INPUT_MPC_TRAJECTORY_UPDATED, "alg/setpoint_raw/local");
}

void TrajectoryInputProducer::activeAnalyticCallback(
    const multirotor_reference_trajectory::AnalyticReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[TrajectoryInputProducer] Received null active analytic trajectory");
        return;
    }
    if (!active_trajectory_cache_.updateAnalytic(*msg, ros::Time::now())) {
        ROS_WARN_THROTTLE(1.0, "[TrajectoryInputProducer] Rejected active analytic trajectory");
        return;
    }
    postInputEvent(event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED,
                   "alg/multirotor_reference_trajectory/active/analytic");
}

void TrajectoryInputProducer::activePolynomialCallback(
    const multirotor_reference_trajectory::ActivePolynomialReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[TrajectoryInputProducer] Received null active polynomial trajectory");
        return;
    }
    if (!active_trajectory_cache_.updatePolynomial(*msg, ros::Time::now())) {
        ROS_WARN_THROTTLE(1.0, "[TrajectoryInputProducer] Rejected active polynomial trajectory");
        return;
    }
    postInputEvent(event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED,
                   "alg/multirotor_reference_trajectory/active/polynomial");
}

void TrajectoryInputProducer::activeSampledCallback(
    const multirotor_reference_trajectory::SampledReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[TrajectoryInputProducer] Received null active sampled trajectory");
        return;
    }
    if (!active_trajectory_cache_.updateSampled(*msg, ros::Time::now())) {
        ROS_WARN_THROTTLE(1.0, "[TrajectoryInputProducer] Rejected active sampled trajectory");
        return;
    }
    postInputEvent(event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED,
                   "alg/multirotor_reference_trajectory/active/sampled");
}

void TrajectoryInputProducer::hoverThrustCallback(
    const hover_thrust_estimator::HoverThrustEstimate::ConstPtr& msg) {
    constexpr uint32_t kRejectFlags =
        hover_thrust_estimator::HoverThrustEstimate::FLAG_STATE_MACHINE_FAULT;
    if (!msg || (msg->flags & kRejectFlags) != 0U ||
        msg->state == hover_thrust_estimator::HoverThrustEstimate::STATE_FAULT ||
        !std::isfinite(msg->hover_thrust) || msg->hover_thrust <= 0.0 || msg->hover_thrust >= 1.0) {
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
