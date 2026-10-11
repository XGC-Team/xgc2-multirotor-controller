#pragma once

// Conversions between the module payloads (payloads.h) and the plain types of the reference
// runtime (reference_types.h). The reference module and its tests include this; nothing else does.
// The conversions are field for field and bit exact: times keep their sec/nsec, doubles are copied.

#include <algorithm>
#include <cstring>

#include "multirotor_reference_trajectory/payloads.h"
#include "multirotor_reference_trajectory/reference_types.h"

namespace multirotor_reference_trajectory {
namespace module {

inline void toCore(const xgc2_px4_reference_header_v1& in, reference::Header& out) {
    out.seq = in.seq;
    out.stamp.sec = in.stamp_sec;
    out.stamp.nsec = in.stamp_nsec;
    out.frame_id.assign(in.frame_id, strnlen(in.frame_id, sizeof in.frame_id));
}

// False when the frame id does not fit.
inline bool toPayload(const reference::Header& in, xgc2_px4_reference_header_v1& out) {
    if (in.frame_id.size() >= sizeof out.frame_id)
        return false;
    out.seq = in.seq;
    out.stamp_sec = in.stamp.sec;
    out.stamp_nsec = in.stamp.nsec;
    out.reserved = 0;
    std::memset(out.frame_id, 0, sizeof out.frame_id);
    std::memcpy(out.frame_id, in.frame_id.data(), in.frame_id.size());
    return true;
}

// False when the payload's length exceeds its capacity.
inline bool toCore(const xgc2_px4_reference_analytic_v1& in, reference::AnalyticReference& out) {
    if (in.params_len > XGC2_PX4_REFERENCE_MAX_PARAMS)
        return false;
    toCore(in.header, out.header);
    out.request_id = in.request_id;
    out.trajectory_id = in.trajectory_id;
    out.revision = in.revision;
    out.analytic_type = in.analytic_type;
    out.flags = in.flags;
    out.start_time.sec = in.start_sec;
    out.start_time.nsec = in.start_nsec;
    out.duration = in.duration;
    out.origin.position = {in.origin_position[0], in.origin_position[1], in.origin_position[2]};
    out.origin.orientation = {in.origin_q_xyzw[0], in.origin_q_xyzw[1], in.origin_q_xyzw[2],
                              in.origin_q_xyzw[3]};
    out.params.assign(in.params, in.params + in.params_len);
    return true;
}

// False when the message does not fit the payload.
inline bool toPayload(const reference::AnalyticReference& in, xgc2_px4_reference_analytic_v1& out) {
    if (in.params.size() > XGC2_PX4_REFERENCE_MAX_PARAMS || !toPayload(in.header, out.header)) {
        return false;
    }
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

// False when the payload's length exceeds its capacity.
inline bool toCore(const xgc2_px4_reference_sampled_v1& in, reference::SampledReference& out) {
    if (in.points_len > XGC2_PX4_REFERENCE_MAX_POINTS)
        return false;
    toCore(in.header, out.header);
    out.trajectory_id = in.trajectory_id;
    out.revision = in.revision;
    out.flags = in.flags;
    out.start_time.sec = in.start_sec;
    out.start_time.nsec = in.start_nsec;
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

// Writes the header and the used prefix of points; the rest of the slot is not touched.
// False when the message does not fit the payload.
inline bool toPayload(const reference::SampledReference& in, xgc2_px4_reference_sampled_v1& out) {
    if (in.points.size() > XGC2_PX4_REFERENCE_MAX_POINTS || !toPayload(in.header, out.header)) {
        return false;
    }
    out.trajectory_id = in.trajectory_id;
    out.revision = in.revision;
    out.flags = in.flags;
    out.points_len = static_cast<uint32_t>(in.points.size());
    out.start_sec = in.start_time.sec;
    out.start_nsec = in.start_time.nsec;
    out.sample_dt = in.sample_dt;
    for (size_t i = 0; i < in.points.size(); ++i) {
        const reference::FlatReferencePoint& q = in.points[i];
        xgc2_px4_reference_point_v1& p = out.points[i];
        p.t_from_start = q.t_from_start;
        p.position[0] = q.position.x;
        p.position[1] = q.position.y;
        p.position[2] = q.position.z;
        p.velocity[0] = q.velocity.x;
        p.velocity[1] = q.velocity.y;
        p.velocity[2] = q.velocity.z;
        p.acceleration[0] = q.acceleration.x;
        p.acceleration[1] = q.acceleration.y;
        p.acceleration[2] = q.acceleration.z;
        p.jerk[0] = q.jerk.x;
        p.jerk[1] = q.jerk.y;
        p.jerk[2] = q.jerk.z;
        p.snap[0] = q.snap.x;
        p.snap[1] = q.snap.y;
        p.snap[2] = q.snap.z;
        p.yaw = q.yaw;
        p.yaw_rate = q.yaw_rate;
        p.yaw_accel = q.yaw_accel;
    }
    return true;
}

inline void toCore(const xgc2_px4_reference_status_v1& in, reference::ReferenceStatus& out) {
    toCore(in.header, out.header);
    out.state = in.state;
    out.flags = in.flags;
    out.active_trajectory_id = in.active_trajectory_id;
    out.active_revision = in.active_revision;
    out.active_type = in.active_type;
}

// False when the status header does not fit the payload.
inline bool toPayload(const reference::ReferenceStatus& in, xgc2_px4_reference_status_v1& out) {
    if (!toPayload(in.header, out.header))
        return false;
    out.state = in.state;
    out.active_type = in.active_type;
    out.reserved[0] = 0;
    out.reserved[1] = 0;
    out.flags = in.flags;
    out.active_trajectory_id = in.active_trajectory_id;
    out.active_revision = in.active_revision;
    return true;
}

}  // namespace module
}  // namespace multirotor_reference_trajectory
