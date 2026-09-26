#pragma once

// The ROS edge of the reference trajectory runtime: messages <-> the plain
// types in reference_types.h, field for field and bit-exact (times keep
// their sec/nsec). Only the node, its tests and the replay harness include
// this; the runtime never does.

#include <multirotor_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/ReferenceStatus.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <multirotor_reference_trajectory_msgs/WaypointReferenceRequest.h>

#include "multirotor_reference_trajectory/reference_types.h"

namespace multirotor_reference_trajectory {

namespace msgs = ::multirotor_reference_trajectory_msgs;

// The plain constants must stay the message constants.
static_assert(reference::AnalyticReference::ANALYTIC_HOLD == msgs::AnalyticReference::ANALYTIC_HOLD, "");
static_assert(reference::AnalyticReference::ANALYTIC_CIRCLE == msgs::AnalyticReference::ANALYTIC_CIRCLE, "");
static_assert(reference::AnalyticReference::ANALYTIC_HEIGHT_CIRCLE == msgs::AnalyticReference::ANALYTIC_HEIGHT_CIRCLE, "");
static_assert(reference::AnalyticReference::ANALYTIC_CIRCLE_ENTRY == msgs::AnalyticReference::ANALYTIC_CIRCLE_ENTRY, "");
static_assert(reference::AnalyticReference::ANALYTIC_FIGURE_EIGHT == msgs::AnalyticReference::ANALYTIC_FIGURE_EIGHT, "");
static_assert(reference::AnalyticReference::ANALYTIC_LINE == msgs::AnalyticReference::ANALYTIC_LINE, "");
static_assert(reference::AnalyticReference::ANALYTIC_LEMNISCATE == msgs::AnalyticReference::ANALYTIC_LEMNISCATE, "");
static_assert(reference::AnalyticReference::ANALYTIC_HELIX_YZ == msgs::AnalyticReference::ANALYTIC_HELIX_YZ, "");
static_assert(reference::AnalyticReference::ANALYTIC_HELIX_XY == msgs::AnalyticReference::ANALYTIC_HELIX_XY, "");
static_assert(reference::AnalyticReference::ANALYTIC_TORUS_KNOT == msgs::AnalyticReference::ANALYTIC_TORUS_KNOT, "");
static_assert(reference::WaypointReferenceRequest::OBJECTIVE_MINCO == msgs::WaypointReferenceRequest::OBJECTIVE_MINCO, "");
static_assert(reference::WaypointReferenceRequest::CONSTRAINT_POINT == msgs::WaypointReferenceRequest::CONSTRAINT_POINT, "");
static_assert(reference::WaypointReferenceRequest::CONSTRAINT_SPHERE == msgs::WaypointReferenceRequest::CONSTRAINT_SPHERE, "");
static_assert(reference::WaypointReferenceRequest::CONSTRAINT_BOX == msgs::WaypointReferenceRequest::CONSTRAINT_BOX, "");
static_assert(reference::WaypointReferenceRequest::CONSTRAINT_GATE == msgs::WaypointReferenceRequest::CONSTRAINT_GATE, "");
static_assert(reference::ReferenceStatus::STATE_SELF_CHECK == msgs::ReferenceStatus::STATE_SELF_CHECK, "");
static_assert(reference::ReferenceStatus::STATE_READY == msgs::ReferenceStatus::STATE_READY, "");
static_assert(reference::ReferenceStatus::STATE_PLANNING == msgs::ReferenceStatus::STATE_PLANNING, "");
static_assert(reference::ReferenceStatus::STATE_ACTIVE == msgs::ReferenceStatus::STATE_ACTIVE, "");
static_assert(reference::ReferenceStatus::STATE_FAULT == msgs::ReferenceStatus::STATE_FAULT, "");
static_assert(reference::ReferenceStatus::TYPE_NONE == msgs::ReferenceStatus::TYPE_NONE, "");
static_assert(reference::ReferenceStatus::TYPE_ANALYTIC == msgs::ReferenceStatus::TYPE_ANALYTIC, "");
static_assert(reference::ReferenceStatus::TYPE_POLYNOMIAL == msgs::ReferenceStatus::TYPE_POLYNOMIAL, "");
static_assert(reference::ReferenceStatus::TYPE_SAMPLED == msgs::ReferenceStatus::TYPE_SAMPLED, "");

inline Time toCore(const ros::Time& t) { return Time(t.sec, t.nsec); }
inline ros::Time toRos(const Time& t) { return ros::Time(t.sec, t.nsec); }

inline reference::Header toCore(const std_msgs::Header& h) { return {h.seq, toCore(h.stamp), h.frame_id}; }
inline std_msgs::Header toRos(const reference::Header& h) {
    std_msgs::Header out;
    out.seq = h.seq;
    out.stamp = toRos(h.stamp);
    out.frame_id = h.frame_id;
    return out;
}
inline reference::Point toCore(const geometry_msgs::Point& p) { return {p.x, p.y, p.z}; }
inline reference::Vector3 toCore(const geometry_msgs::Vector3& v) { return {v.x, v.y, v.z}; }
inline reference::Pose toCore(const geometry_msgs::Pose& p) {
    return {toCore(p.position), {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}};
}
inline geometry_msgs::Point toRos(const reference::Point& p) {
    geometry_msgs::Point out;
    out.x = p.x;
    out.y = p.y;
    out.z = p.z;
    return out;
}
inline geometry_msgs::Vector3 toRos(const reference::Vector3& v) {
    geometry_msgs::Vector3 out;
    out.x = v.x;
    out.y = v.y;
    out.z = v.z;
    return out;
}
inline geometry_msgs::Pose toRos(const reference::Pose& p) {
    geometry_msgs::Pose out;
    out.position = toRos(p.position);
    out.orientation.x = p.orientation.x;
    out.orientation.y = p.orientation.y;
    out.orientation.z = p.orientation.z;
    out.orientation.w = p.orientation.w;
    return out;
}

inline reference::AnalyticReference toCore(const msgs::AnalyticReference& m) {
    reference::AnalyticReference out;
    out.header = toCore(m.header);
    out.request_id = m.request_id;
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.analytic_type = m.analytic_type;
    out.flags = m.flags;
    out.start_time = toCore(m.start_time);
    out.duration = m.duration;
    out.origin = toCore(m.origin);
    out.params = m.params;
    return out;
}
inline msgs::AnalyticReference toRos(const reference::AnalyticReference& m) {
    msgs::AnalyticReference out;
    out.header = toRos(m.header);
    out.request_id = m.request_id;
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.analytic_type = m.analytic_type;
    out.flags = m.flags;
    out.start_time = toRos(m.start_time);
    out.duration = m.duration;
    out.origin = toRos(m.origin);
    out.params = m.params;
    return out;
}

inline reference::SampledReference toCore(const msgs::SampledReference& m) {
    reference::SampledReference out;
    out.header = toCore(m.header);
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.flags = m.flags;
    out.start_time = toCore(m.start_time);
    out.sample_dt = m.sample_dt;
    out.points.reserve(m.points.size());
    for (const auto& p : m.points) {
        out.points.push_back({p.t_from_start, toCore(p.position), toCore(p.velocity), toCore(p.acceleration),
                              toCore(p.jerk), toCore(p.snap), p.yaw, p.yaw_rate, p.yaw_accel});
    }
    return out;
}
inline msgs::SampledReference toRos(const reference::SampledReference& m) {
    msgs::SampledReference out;
    out.header = toRos(m.header);
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.flags = m.flags;
    out.start_time = toRos(m.start_time);
    out.sample_dt = m.sample_dt;
    out.points.reserve(m.points.size());
    for (const auto& p : m.points) {
        msgs::FlatReferencePoint q;
        q.t_from_start = p.t_from_start;
        q.position = toRos(p.position);
        q.velocity = toRos(p.velocity);
        q.acceleration = toRos(p.acceleration);
        q.jerk = toRos(p.jerk);
        q.snap = toRos(p.snap);
        q.yaw = p.yaw;
        q.yaw_rate = p.yaw_rate;
        q.yaw_accel = p.yaw_accel;
        out.points.push_back(q);
    }
    return out;
}

inline reference::WaypointReferenceRequest toCore(const msgs::WaypointReferenceRequest& m) {
    reference::WaypointReferenceRequest out;
    out.header = toCore(m.header);
    out.request_id = m.request_id;
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.flags = m.flags;
    for (const auto& p : m.waypoints) out.waypoints.push_back(toCore(p));
    out.constraint_types = m.constraint_types;
    for (const auto& v : m.region_size) out.region_size.push_back(toCore(v));
    out.segment_times = m.segment_times;
    out.start_velocity = toCore(m.start_velocity);
    out.start_acceleration = toCore(m.start_acceleration);
    out.end_velocity = toCore(m.end_velocity);
    out.end_acceleration = toCore(m.end_acceleration);
    out.desired_speed = m.desired_speed;
    out.time_weight = m.time_weight;
    out.max_body_rate = m.max_body_rate;
    out.max_tilt = m.max_tilt;
    out.min_thrust = m.min_thrust;
    out.max_thrust = m.max_thrust;
    out.max_iterations = m.max_iterations;
    out.rel_cost_tol = m.rel_cost_tol;
    out.max_velocity = m.max_velocity;
    out.max_acceleration = m.max_acceleration;
    out.max_jerk = m.max_jerk;
    out.max_snap = m.max_snap;
    out.objective = m.objective;
    return out;
}

inline msgs::ActivePolynomialReference toRos(const reference::ActivePolynomialReference& m) {
    msgs::ActivePolynomialReference out;
    out.header = toRos(m.header);
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.flags = m.flags;
    out.start_time = toRos(m.start_time);
    out.duration = m.duration;
    out.order = m.order;
    out.segment_durations = m.segment_durations;
    out.coeff_x = m.coeff_x;
    out.coeff_y = m.coeff_y;
    out.coeff_z = m.coeff_z;
    out.coeff_yaw = m.coeff_yaw;
    return out;
}

inline msgs::ReferenceStatus toRos(const reference::ReferenceStatus& m) {
    msgs::ReferenceStatus out;
    out.header = toRos(m.header);
    out.state = m.state;
    out.flags = m.flags;
    out.active_trajectory_id = m.active_trajectory_id;
    out.active_revision = m.active_revision;
    out.active_type = m.active_type;
    return out;
}

}  // namespace multirotor_reference_trajectory
