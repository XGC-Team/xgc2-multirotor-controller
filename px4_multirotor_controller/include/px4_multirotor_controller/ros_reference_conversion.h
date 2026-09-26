#pragma once

#include <multirotor_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>

#include "px4_multirotor_controller/ros_time_conversion.h"
#include "px4_multirotor_controller/uav/reference_types.h"

namespace px4_multirotor_controller {

// ROS edge only: reference messages -> the cache's plain inputs, field for
// field.
namespace detail {
using RosAnalytic = multirotor_reference_trajectory_msgs::AnalyticReference;
using CoreAnalytic = reference::AnalyticReference;
static_assert(CoreAnalytic::ANALYTIC_HOLD == RosAnalytic::ANALYTIC_HOLD, "analytic type");
static_assert(CoreAnalytic::ANALYTIC_CIRCLE == RosAnalytic::ANALYTIC_CIRCLE, "analytic type");
static_assert(CoreAnalytic::ANALYTIC_HEIGHT_CIRCLE == RosAnalytic::ANALYTIC_HEIGHT_CIRCLE, "analytic type");
static_assert(CoreAnalytic::ANALYTIC_CIRCLE_ENTRY == RosAnalytic::ANALYTIC_CIRCLE_ENTRY, "analytic type");
static_assert(CoreAnalytic::ANALYTIC_FIGURE_EIGHT == RosAnalytic::ANALYTIC_FIGURE_EIGHT, "analytic type");
static_assert(CoreAnalytic::ANALYTIC_LINE == RosAnalytic::ANALYTIC_LINE, "analytic type");
static_assert(CoreAnalytic::ANALYTIC_LEMNISCATE == RosAnalytic::ANALYTIC_LEMNISCATE, "analytic type");
static_assert(CoreAnalytic::ANALYTIC_HELIX_YZ == RosAnalytic::ANALYTIC_HELIX_YZ, "analytic type");
static_assert(CoreAnalytic::ANALYTIC_HELIX_XY == RosAnalytic::ANALYTIC_HELIX_XY, "analytic type");
static_assert(CoreAnalytic::ANALYTIC_TORUS_KNOT == RosAnalytic::ANALYTIC_TORUS_KNOT, "analytic type");

template <typename P>
reference::Point point(const P& p) {
    return {p.x, p.y, p.z};
}
template <typename V>
reference::Vector3 vec(const V& v) {
    return {v.x, v.y, v.z};
}
}  // namespace detail

inline reference::AnalyticReference toCoreReference(const multirotor_reference_trajectory_msgs::AnalyticReference& m) {
    reference::AnalyticReference r;
    r.header.stamp = toCoreTime(m.header.stamp);
    r.request_id = m.request_id;
    r.trajectory_id = m.trajectory_id;
    r.revision = m.revision;
    r.analytic_type = m.analytic_type;
    r.flags = m.flags;
    r.start_time = toCoreTime(m.start_time);
    r.duration = m.duration;
    r.origin.position = detail::point(m.origin.position);
    r.origin.orientation = {m.origin.orientation.x, m.origin.orientation.y, m.origin.orientation.z, m.origin.orientation.w};
    r.params = m.params;
    return r;
}

// The reverse, for the request the controller itself sends (reference
// activation). frame_id is the edge's.
inline multirotor_reference_trajectory_msgs::AnalyticReference toRosReference(const reference::AnalyticReference& r) {
    multirotor_reference_trajectory_msgs::AnalyticReference m;
    m.header.stamp = toRosTime(r.header.stamp);
    m.request_id = r.request_id;
    m.trajectory_id = r.trajectory_id;
    m.revision = r.revision;
    m.analytic_type = r.analytic_type;
    m.flags = r.flags;
    m.start_time = toRosTime(r.start_time);
    m.duration = r.duration;
    m.origin.position.x = r.origin.position.x;
    m.origin.position.y = r.origin.position.y;
    m.origin.position.z = r.origin.position.z;
    m.origin.orientation.x = r.origin.orientation.x;
    m.origin.orientation.y = r.origin.orientation.y;
    m.origin.orientation.z = r.origin.orientation.z;
    m.origin.orientation.w = r.origin.orientation.w;
    m.params = r.params;
    return m;
}

inline reference::ActivePolynomialReference toCoreReference(
    const multirotor_reference_trajectory_msgs::ActivePolynomialReference& m) {
    reference::ActivePolynomialReference r;
    r.header.stamp = toCoreTime(m.header.stamp);
    r.trajectory_id = m.trajectory_id;
    r.revision = m.revision;
    r.flags = m.flags;
    r.start_time = toCoreTime(m.start_time);
    r.duration = m.duration;
    r.order = m.order;
    r.segment_durations = m.segment_durations;
    r.coeff_x = m.coeff_x;
    r.coeff_y = m.coeff_y;
    r.coeff_z = m.coeff_z;
    r.coeff_yaw = m.coeff_yaw;
    return r;
}

inline reference::SampledReference toCoreReference(const multirotor_reference_trajectory_msgs::SampledReference& m) {
    reference::SampledReference r;
    r.header.stamp = toCoreTime(m.header.stamp);
    r.trajectory_id = m.trajectory_id;
    r.revision = m.revision;
    r.flags = m.flags;
    r.start_time = toCoreTime(m.start_time);
    r.sample_dt = m.sample_dt;
    r.points.reserve(m.points.size());
    for (const auto& p : m.points) {
        reference::FlatReferencePoint q;
        q.t_from_start = p.t_from_start;
        q.position = detail::point(p.position);
        q.velocity = detail::vec(p.velocity);
        q.acceleration = detail::vec(p.acceleration);
        q.jerk = detail::vec(p.jerk);
        q.snap = detail::vec(p.snap);
        q.yaw = p.yaw;
        q.yaw_rate = p.yaw_rate;
        q.yaw_accel = p.yaw_accel;
        r.points.push_back(q);
    }
    return r;
}

}  // namespace px4_multirotor_controller
