#pragma once

// Conversions between the reference payloads (multirotor_reference_trajectory/payloads.h) and the
// plain reference types of the controller's trajectory cache (uav/reference_types.h). The module and
// its tests include this; nothing else does. Times keep their sec/nsec and doubles are copied, as
// ros_reference_conversion.h does for the ROS messages.

#include <algorithm>
#include <cstring>
#include <string>

#include "multirotor_reference_trajectory/payloads.h"
#include "px4_multirotor_controller/uav/reference_types.h"

namespace px4_multirotor_controller {
namespace module {

// False when the payload's length exceeds its capacity. Throws std::runtime_error when a time is
// out of the range of ROS time.
inline bool toCore(const xgc2_px4_reference_analytic_v1& in, reference::AnalyticReference& out) {
    if (in.params_len > XGC2_PX4_REFERENCE_MAX_PARAMS) return false;
    out.header.stamp = Time(in.header.stamp_sec, in.header.stamp_nsec);
    out.request_id = in.request_id;
    out.trajectory_id = in.trajectory_id;
    out.revision = in.revision;
    out.analytic_type = in.analytic_type;
    out.flags = in.flags;
    out.start_time = Time(in.start_sec, in.start_nsec);
    out.duration = in.duration;
    out.origin.position = {in.origin_position[0], in.origin_position[1], in.origin_position[2]};
    out.origin.orientation = {in.origin_q_xyzw[0], in.origin_q_xyzw[1], in.origin_q_xyzw[2],
                              in.origin_q_xyzw[3]};
    out.params.assign(in.params, in.params + in.params_len);
    return true;
}

// False when the payload's length exceeds its capacity. Throws std::runtime_error when a time is
// out of the range of ROS time.
inline bool toCore(const xgc2_px4_reference_sampled_v1& in, reference::SampledReference& out) {
    if (in.points_len > XGC2_PX4_REFERENCE_MAX_POINTS) return false;
    out.header.stamp = Time(in.header.stamp_sec, in.header.stamp_nsec);
    out.trajectory_id = in.trajectory_id;
    out.revision = in.revision;
    out.flags = in.flags;
    out.start_time = Time(in.start_sec, in.start_nsec);
    out.sample_dt = in.sample_dt;
    out.points.resize(in.points_len);
    for (uint32_t i = 0; i < in.points_len; ++i) {
        const xgc2_px4_reference_point_v1& p = in.points[i];
        reference::FlatReferencePoint& q = out.points[i];
        q.t_from_start = p.t_from_start;
        q.position = {p.position[0], p.position[1], p.position[2]};
        q.velocity = {p.velocity[0], p.velocity[1], p.velocity[2]};
        q.acceleration = {p.acceleration[0], p.acceleration[1], p.acceleration[2]};
        q.jerk = {p.jerk[0], p.jerk[1], p.jerk[2]};
        q.snap = {p.snap[0], p.snap[1], p.snap[2]};
        q.yaw = p.yaw;
        q.yaw_rate = p.yaw_rate;
        q.yaw_accel = p.yaw_accel;
    }
    return true;
}

// The activation request the controller sends to the reference generator, with the frame the
// ROS consumer gives it. False when the message does not fit the payload.
inline bool toPayload(const reference::AnalyticReference& in, const char* frame_id,
                      xgc2_px4_reference_analytic_v1& out) {
    const size_t frame_length = std::strlen(frame_id);
    if (in.params.size() > XGC2_PX4_REFERENCE_MAX_PARAMS ||
        frame_length >= sizeof out.header.frame_id) {
        return false;
    }
    out.header.seq = 0;
    out.header.stamp_sec = in.header.stamp.sec;
    out.header.stamp_nsec = in.header.stamp.nsec;
    out.header.reserved = 0;
    std::memset(out.header.frame_id, 0, sizeof out.header.frame_id);
    std::memcpy(out.header.frame_id, frame_id, frame_length);
    out.request_id = in.request_id;
    out.trajectory_id = in.trajectory_id;
    out.revision = in.revision;
    out.flags = in.flags;
    out.start_sec = in.start_time.sec;
    out.start_nsec = in.start_time.nsec;
    out.analytic_type = in.analytic_type;
    out.params_len = static_cast<uint16_t>(in.params.size());
    out.reserved = 0;
    out.duration = in.duration;
    out.origin_position[0] = in.origin.position.x;
    out.origin_position[1] = in.origin.position.y;
    out.origin_position[2] = in.origin.position.z;
    out.origin_q_xyzw[0] = in.origin.orientation.x;
    out.origin_q_xyzw[1] = in.origin.orientation.y;
    out.origin_q_xyzw[2] = in.origin.orientation.z;
    out.origin_q_xyzw[3] = in.origin.orientation.w;
    std::memset(out.params, 0, sizeof out.params);
    std::copy(in.params.begin(), in.params.end(), out.params);
    return true;
}

}  // namespace module
}  // namespace px4_multirotor_controller
