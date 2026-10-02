#include "px4_multirotor_controller/output/nmpc_output_consumer.h"
#include "px4_multirotor_controller/ros_time_conversion.h"

#include <px4_multirotor_controller_msgs/NmpcDebugSample.h>

#include <cmath>
#include <utility>

namespace px4_multirotor_controller {
namespace {

geometry_msgs::Vector3 vector3FromEigen(const Eigen::Vector3d& vector) {
    geometry_msgs::Vector3 value;
    value.x = vector.x();
    value.y = vector.y();
    value.z = vector.z();
    return value;
}

geometry_msgs::Pose poseFromState(const Se3StateVector& state) {
    geometry_msgs::Pose pose;
    pose.position.x = state(0);
    pose.position.y = state(1);
    pose.position.z = state(2);
    const double qw = state(6);
    const double qx = state(7);
    const double qy = state(8);
    const double qz = state(9);
    const double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (std::isfinite(q_norm) && q_norm > 1.0e-9) {
        pose.orientation.w = qw / q_norm;
        pose.orientation.x = qx / q_norm;
        pose.orientation.y = qy / q_norm;
        pose.orientation.z = qz / q_norm;
    } else {
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
    }
    return pose;
}

geometry_msgs::Twist twistFromState(const Se3StateVector& state) {
    geometry_msgs::Twist twist;
    twist.linear.x = state(3);
    twist.linear.y = state(4);
    twist.linear.z = state(5);
    twist.angular.x = state(10);
    twist.angular.y = state(11);
    twist.angular.z = state(12);
    return twist;
}

}  // namespace

NmpcOutputConsumer::NmpcOutputConsumer(ros::NodeHandle& nh, NmpcExecution& execution, uint32_t queue_size)
 : execution_(execution) {
 debug_pub_ = nh.advertise<px4_multirotor_controller_msgs::NmpcDebugSample>("alg/nmpc/debug_sample", queue_size);
 predicted_path_pub_ = nh.advertise<nav_msgs::Path>("alg/nmpc/predicted_path", queue_size);
 predicted_poses_pub_ = nh.advertise<geometry_msgs::PoseArray>("alg/nmpc/predicted_poses", queue_size);
 execution_.setDiagnosticSink([this](uint64_t sequence, const Time& stamp,
                                   const UavNmpcTrackingBackend& backend, bool success) {
   publishDebug(sequence, stamp, backend);
   if (success) publishPrediction(stamp, backend);
 });
}
NmpcOutputConsumer::~NmpcOutputConsumer() {
 execution_.stop();
 execution_.setDiagnosticSink({});
}
bool NmpcOutputConsumer::handle(const ::state_machine::Event& event) { return execution_.handle(event); }

void NmpcOutputConsumer::publishDebug(uint64_t sequence, const Time& stamp, const UavNmpcTrackingBackend& backend) {
    const NmpcDebugData& debug = backend.lastDebugData();
    if (!debug.valid) {
        return;
    }

    px4_multirotor_controller_msgs::NmpcDebugSample msg;
    msg.header.stamp = toRosTime(stamp);
    msg.header.frame_id = "world";
    msg.sequence = sequence;
    msg.success = debug.success;
    msg.solver_status = debug.solver_status;
    msg.solve_time_ms = debug.solve_time_ms;
    msg.state_estimate_stamp_sec = debug.state_estimate_stamp_sec;
    msg.filter_inertial_stamp_sec = debug.filter_inertial_stamp_sec;
    msg.filter_pose_stamp_sec = debug.filter_pose_stamp_sec;
    msg.last_vrpn_pose_stamp_sec = debug.last_vrpn_pose_stamp_sec;
    msg.state_pose = poseFromState(debug.state);
    msg.state_twist = twistFromState(debug.state);
    msg.reference_pose = poseFromState(debug.reference);
    msg.reference_twist = twistFromState(debug.reference);
    msg.horizon_pose = poseFromState(debug.horizon_reference);
    msg.reference_acceleration = vector3FromEigen(debug.reference_acceleration);
    msg.position_error = vector3FromEigen(debug.position_error);
    msg.velocity_error = vector3FromEigen(debug.velocity_error);
    msg.omega_error = vector3FromEigen(debug.omega_error);
    msg.body_rate_command = vector3FromEigen(debug.body_rate_command);
    msg.predicted_body_rate = vector3FromEigen(debug.predicted_body_rate);
    msg.alpha_command = vector3FromEigen(debug.angular_acceleration_command);
    msg.reference_alpha = vector3FromEigen(debug.reference_control.segment<3>(1));
    msg.specific_thrust_command = debug.optimal_control(0);
    msg.normalized_thrust_raw = debug.normalized_thrust_raw;
    msg.normalized_thrust_command = debug.normalized_thrust_command;
    msg.reference_specific_thrust = debug.reference_control(0);
    msg.hover_thrust = debug.hover_thrust;
    msg.initial_hover_thrust = debug.initial_hover_thrust;
    msg.thrust_actual_estimate = debug.thrust_actual_estimate;
    msg.last_commanded_specific_thrust = debug.last_commanded_specific_thrust;
    msg.effective_specific_thrust_min = debug.effective_specific_thrust_min;
    msg.effective_specific_thrust_max = debug.effective_specific_thrust_max;
    msg.normalized_thrust_min_saturated = debug.normalized_thrust_min_saturated;
    msg.normalized_thrust_max_saturated = debug.normalized_thrust_max_saturated;
    msg.roll_rate_saturated = debug.roll_rate_saturated;
    msg.pitch_rate_saturated = debug.pitch_rate_saturated;
    msg.yaw_rate_saturated = debug.yaw_rate_saturated;
    msg.roll_alpha_saturated = debug.roll_alpha_saturated;
    msg.pitch_alpha_saturated = debug.pitch_alpha_saturated;
    msg.yaw_alpha_saturated = debug.yaw_alpha_saturated;
    debug_pub_.publish(msg);
}

void NmpcOutputConsumer::publishPrediction(const Time& stamp, const UavNmpcTrackingBackend& backend) {
    nav_msgs::Path path;
    geometry_msgs::PoseArray poses;
    path.header.stamp = toRosTime(stamp);
    path.header.frame_id = "world";
    poses.header = path.header;

    const auto& predicted_states = backend.predictedStates();
    const size_t count = backend.predictedStateCount();
    path.poses.reserve(count);
    poses.poses.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const geometry_msgs::Pose pose = poseFromState(predicted_states[i]);
        geometry_msgs::PoseStamped stamped_pose;
        stamped_pose.header = path.header;
        stamped_pose.pose = pose;
        path.poses.push_back(stamped_pose);
        poses.poses.push_back(pose);
    }

    predicted_path_pub_.publish(path);
    predicted_poses_pub_.publish(poses);
}

}  // namespace px4_multirotor_controller
