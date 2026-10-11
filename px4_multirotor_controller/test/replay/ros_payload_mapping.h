#pragma once

// The mapping of the ROS messages the controller node subscribes to onto the payloads of the
// px4_multirotor_controller module, as the entity's ROS edge makes it: one function per input port.
// The module tests use it to feed the module the messages a recording or a scripted flight holds;
// the README of the package lists the same mapping for the edge module.
//
// Stamps are the message header stamps (sec, nsec) in the host clock domain; the sample's own
// stamp_ns, which the caller gives to push(), is the time the message was received.

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hover_thrust_estimator_msgs/HoverThrustEstimate.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "multirotor_reference_trajectory/payloads.h"
#include "px4_multirotor_controller/payloads.h"

namespace ros_payload_mapping {

inline void stamp(const ros::Time& t, uint32_t& sec, uint32_t& nsec) {
    sec = t.sec;
    nsec = t.nsec;
}

template <size_t N>
void copyText(const std::string& text, char (&field)[N], const char* what) {
    if (text.size() >= N)
        throw std::length_error(std::string(what) + " does not fit its payload");
    std::memset(field, 0, N);
    std::memcpy(field, text.data(), text.size());
}

inline xgc2_px4_state_estimate_v1 map(const rigid_state_estimator_msgs::RigidStateEstimate& m) {
    xgc2_px4_state_estimate_v1 p;
    std::memset(&p, 0, sizeof p);
    stamp(m.header.stamp, p.stamp_sec, p.stamp_nsec);
    p.position[0] = m.position.x;
    p.position[1] = m.position.y;
    p.position[2] = m.position.z;
    p.velocity[0] = m.velocity.x;
    p.velocity[1] = m.velocity.y;
    p.velocity[2] = m.velocity.z;
    p.orientation_xyzw[0] = m.orientation.x;
    p.orientation_xyzw[1] = m.orientation.y;
    p.orientation_xyzw[2] = m.orientation.z;
    p.orientation_xyzw[3] = m.orientation.w;
    p.angular_velocity[0] = m.angular_velocity.x;
    p.angular_velocity[1] = m.angular_velocity.y;
    p.angular_velocity[2] = m.angular_velocity.z;
    p.linear_acceleration[0] = m.linear_acceleration.x;
    p.linear_acceleration[1] = m.linear_acceleration.y;
    p.linear_acceleration[2] = m.linear_acceleration.z;
    p.gravity[0] = m.gravity.x;
    p.gravity[1] = m.gravity.y;
    p.gravity[2] = m.gravity.z;
    p.accel_bias[0] = m.accel_bias.x;
    p.accel_bias[1] = m.accel_bias.y;
    p.accel_bias[2] = m.accel_bias.z;
    p.filter_inertial_stamp_sec = m.filter_inertial_stamp_sec;
    p.filter_pose_stamp_sec = m.filter_pose_stamp_sec;
    p.last_vrpn_pose_stamp_sec = m.last_vrpn_pose_stamp_sec;
    p.flags = m.flags;
    p.estimator_state = m.estimator_state;
    return p;
}

inline xgc2_px4_pose_v1 map(const geometry_msgs::PoseStamped& m) {
    xgc2_px4_pose_v1 p;
    std::memset(&p, 0, sizeof p);
    stamp(m.header.stamp, p.stamp_sec, p.stamp_nsec);
    p.position[0] = m.pose.position.x;
    p.position[1] = m.pose.position.y;
    p.position[2] = m.pose.position.z;
    p.orientation_xyzw[0] = m.pose.orientation.x;
    p.orientation_xyzw[1] = m.pose.orientation.y;
    p.orientation_xyzw[2] = m.pose.orientation.z;
    p.orientation_xyzw[3] = m.pose.orientation.w;
    return p;
}

inline xgc2_px4_velocity_v1 map(const geometry_msgs::TwistStamped& m) {
    xgc2_px4_velocity_v1 p;
    std::memset(&p, 0, sizeof p);
    stamp(m.header.stamp, p.stamp_sec, p.stamp_nsec);
    p.linear[0] = m.twist.linear.x;
    p.linear[1] = m.twist.linear.y;
    p.linear[2] = m.twist.linear.z;
    return p;
}

// sensor_msgs/Imu: the controller needs only that a sample arrived.
inline xgc2_px4_imu_v1 imuSample(const ros::Time& header_stamp) {
    xgc2_px4_imu_v1 p;
    std::memset(&p, 0, sizeof p);
    stamp(header_stamp, p.stamp_sec, p.stamp_nsec);
    return p;
}

inline xgc2_px4_fcu_state_v1 map(const mavros_msgs::State& m) {
    xgc2_px4_fcu_state_v1 p;
    std::memset(&p, 0, sizeof p);
    stamp(m.header.stamp, p.stamp_sec, p.stamp_nsec);
    p.connected = m.connected ? 1 : 0;
    p.armed = m.armed ? 1 : 0;
    p.guided = m.guided ? 1 : 0;
    p.manual_input = m.manual_input ? 1 : 0;
    p.system_status = m.system_status;
    copyText(m.mode, p.mode, "mode");
    return p;
}

inline xgc2_px4_battery_v1 map(const sensor_msgs::BatteryState& m) {
    xgc2_px4_battery_v1 p;
    std::memset(&p, 0, sizeof p);
    stamp(m.header.stamp, p.stamp_sec, p.stamp_nsec);
    p.percentage = m.percentage;
    return p;
}

// std_msgs/String on /command. A command that does not fit is longer than the node accepts (64).
inline xgc2_px4_command_v1 map(const std_msgs::String& m) {
    xgc2_px4_command_v1 p;
    std::memset(&p, 0, sizeof p);
    copyText(m.data, p.text, "command");
    return p;
}

inline xgc2_px4_position_target_v1 map(const mavros_msgs::PositionTarget& m) {
    xgc2_px4_position_target_v1 p;
    std::memset(&p, 0, sizeof p);
    stamp(m.header.stamp, p.stamp_sec, p.stamp_nsec);
    p.position[0] = m.position.x;
    p.position[1] = m.position.y;
    p.position[2] = m.position.z;
    p.velocity[0] = m.velocity.x;
    p.velocity[1] = m.velocity.y;
    p.velocity[2] = m.velocity.z;
    p.acceleration[0] = m.acceleration_or_force.x;
    p.acceleration[1] = m.acceleration_or_force.y;
    p.acceleration[2] = m.acceleration_or_force.z;
    p.yaw = m.yaw;
    p.yaw_rate = m.yaw_rate;
    p.type_mask = m.type_mask;
    p.coordinate_frame = m.coordinate_frame;
    return p;
}

inline xgc2_px4_hover_thrust_v1 map(const hover_thrust_estimator_msgs::HoverThrustEstimate& m) {
    xgc2_px4_hover_thrust_v1 p;
    std::memset(&p, 0, sizeof p);
    stamp(m.header.stamp, p.stamp_sec, p.stamp_nsec);
    p.hover_thrust = m.hover_thrust;
    p.flags = m.flags;
    return p;
}

inline void header(const std_msgs::Header& h, xgc2_px4_reference_header_v1& p) {
    p.seq = h.seq;
    stamp(h.stamp, p.stamp_sec, p.stamp_nsec);
    p.reserved = 0;
    copyText(h.frame_id, p.frame_id, "frame_id");
}

// A request longer than the payload's capacity cannot be represented: the edge refuses it.
inline xgc2_px4_reference_analytic_v1 map(
    const multirotor_reference_trajectory_msgs::AnalyticReference& m) {
    if (m.params.size() > XGC2_PX4_REFERENCE_MAX_PARAMS) {
        throw std::length_error("analytic reference has more parameters than the payload holds");
    }
    xgc2_px4_reference_analytic_v1 p;
    std::memset(&p, 0, sizeof p);
    header(m.header, p.header);
    p.request_id = m.request_id;
    p.trajectory_id = m.trajectory_id;
    p.revision = m.revision;
    p.flags = m.flags;
    stamp(m.start_time, p.start_sec, p.start_nsec);
    p.analytic_type = m.analytic_type;
    p.params_len = static_cast<uint16_t>(m.params.size());
    p.duration = m.duration;
    p.origin_position[0] = m.origin.position.x;
    p.origin_position[1] = m.origin.position.y;
    p.origin_position[2] = m.origin.position.z;
    p.origin_q_xyzw[0] = m.origin.orientation.x;
    p.origin_q_xyzw[1] = m.origin.orientation.y;
    p.origin_q_xyzw[2] = m.origin.orientation.z;
    p.origin_q_xyzw[3] = m.origin.orientation.w;
    std::memcpy(p.params, m.params.data(), m.params.size() * sizeof(double));
    return p;
}

inline std::unique_ptr<xgc2_px4_reference_sampled_v1> map(
    const multirotor_reference_trajectory_msgs::SampledReference& m) {
    if (m.points.size() > XGC2_PX4_REFERENCE_MAX_POINTS) {
        throw std::length_error("sampled reference has more points than the payload holds");
    }
    auto p = std::make_unique<xgc2_px4_reference_sampled_v1>();
    std::memset(p.get(), 0, sizeof *p);
    header(m.header, p->header);
    p->trajectory_id = m.trajectory_id;
    p->revision = m.revision;
    p->flags = m.flags;
    p->points_len = static_cast<uint32_t>(m.points.size());
    stamp(m.start_time, p->start_sec, p->start_nsec);
    p->sample_dt = m.sample_dt;
    for (size_t i = 0; i < m.points.size(); ++i) {
        const auto& q = m.points[i];
        xgc2_px4_reference_point_v1& out = p->points[i];
        out.t_from_start = q.t_from_start;
        out.position[0] = q.position.x;
        out.position[1] = q.position.y;
        out.position[2] = q.position.z;
        out.velocity[0] = q.velocity.x;
        out.velocity[1] = q.velocity.y;
        out.velocity[2] = q.velocity.z;
        out.acceleration[0] = q.acceleration.x;
        out.acceleration[1] = q.acceleration.y;
        out.acceleration[2] = q.acceleration.z;
        out.jerk[0] = q.jerk.x;
        out.jerk[1] = q.jerk.y;
        out.jerk[2] = q.jerk.z;
        out.snap[0] = q.snap.x;
        out.snap[1] = q.snap.y;
        out.snap[2] = q.snap.z;
        out.yaw = q.yaw;
        out.yaw_rate = q.yaw_rate;
        out.yaw_accel = q.yaw_accel;
    }
    return p;
}

}  // namespace ros_payload_mapping
