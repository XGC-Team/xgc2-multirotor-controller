/*
 * Payloads of the multirotor reference trajectory chain (xgc2-module module ABI).
 *
 * Fixed-size POD structs, C11 and C++ compatible, shared by the
 * px4_multirotor_reference module, the px4_multirotor_controller module that
 * consumes its output, and the ROS edge of the entity. They never leave the
 * process: the host hands them from the producer's slot to the readers without
 * copying or serializing. Layout, field order and widths of a schema never
 * change; a different layout gets a new schema id.
 *
 * Times are ROS times (seconds and nanoseconds, as in the ROS messages the
 * ROS node publishes); the layouts mirror multirotor_reference_trajectory_msgs
 * field for field. Variable-length ROS fields have a capacity and a length:
 * a request that exceeds the capacity cannot be represented and is refused by
 * the producing edge.
 */
#ifndef MULTIROTOR_REFERENCE_TRAJECTORY_PAYLOADS_H
#define MULTIROTOR_REFERENCE_TRAJECTORY_PAYLOADS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XGC2_PX4_REFERENCE_ANALYTIC_SCHEMA "xgc2.px4.reference_analytic.v1"
#define XGC2_PX4_REFERENCE_SAMPLED_SCHEMA "xgc2.px4.reference_sampled.v1"
#define XGC2_PX4_REFERENCE_STATUS_SCHEMA "xgc2.px4.reference_status.v1"
#define XGC2_PX4_REFERENCE_RESET_SCHEMA "xgc2.px4.reference_reset.v1"

#define XGC2_PX4_REFERENCE_FRAME_ID_SIZE 32u /* including the terminating NUL */
#define XGC2_PX4_REFERENCE_MAX_PARAMS 16u    /* analytic curve parameters */
#define XGC2_PX4_REFERENCE_MAX_POINTS 1024u  /* sampled reference points */

/* std_msgs/Header. Only the stamp is read by the runtime; seq and frame_id pass through. */
typedef struct xgc2_px4_reference_header_v1 {
    uint32_t seq;
    uint32_t stamp_sec;
    uint32_t stamp_nsec;
    uint32_t reserved;                               /* zero */
    char frame_id[XGC2_PX4_REFERENCE_FRAME_ID_SIZE]; /* NUL-terminated */
} xgc2_px4_reference_header_v1;

/*
 * "xgc2.px4.reference_analytic.v1": multirotor_reference_trajectory_msgs/AnalyticReference.
 * A request into the reference module (port `analytic`) and the active reference it publishes
 * (port `active_analytic`). params[0 .. params_len) are the curve parameters of analytic_type;
 * the constants of analytic_type are those of the ROS message.
 */
typedef struct xgc2_px4_reference_analytic_v1 {
    xgc2_px4_reference_header_v1 header;
    uint32_t request_id;
    uint32_t trajectory_id;
    uint32_t revision;
    uint32_t flags;
    uint32_t start_sec; /* zero: start as soon as the lead time allows */
    uint32_t start_nsec;
    uint16_t analytic_type;
    uint16_t params_len; /* <= XGC2_PX4_REFERENCE_MAX_PARAMS */
    uint32_t reserved;   /* zero */
    double duration;     /* s; not positive: the curve's default duration */
    double origin_position[3];
    double origin_q_xyzw[4];
    double params[XGC2_PX4_REFERENCE_MAX_PARAMS];
} xgc2_px4_reference_analytic_v1;

/* multirotor_reference_trajectory_msgs/FlatReferencePoint. */
typedef struct xgc2_px4_reference_point_v1 {
    double t_from_start;
    double position[3];
    double velocity[3];
    double acceleration[3];
    double jerk[3];
    double snap[3];
    double yaw;
    double yaw_rate;
    double yaw_accel;
} xgc2_px4_reference_point_v1;

/*
 * "xgc2.px4.reference_sampled.v1": multirotor_reference_trajectory_msgs/SampledReference.
 * Only points[0 .. points_len) are meaningful; producers write just that prefix.
 */
typedef struct xgc2_px4_reference_sampled_v1 {
    xgc2_px4_reference_header_v1 header;
    uint32_t trajectory_id;
    uint32_t revision;
    uint32_t flags;
    uint32_t points_len; /* <= XGC2_PX4_REFERENCE_MAX_POINTS */
    uint32_t start_sec;  /* zero: start as soon as the lead time allows */
    uint32_t start_nsec;
    double sample_dt;
    xgc2_px4_reference_point_v1 points[XGC2_PX4_REFERENCE_MAX_POINTS];
} xgc2_px4_reference_sampled_v1;

/*
 * "xgc2.px4.reference_status.v1": multirotor_reference_trajectory_msgs/ReferenceStatus.
 * state is ReferenceStatus::STATE_*, active_type is ReferenceStatus::TYPE_*.
 */
typedef struct xgc2_px4_reference_status_v1 {
    xgc2_px4_reference_header_v1 header;
    uint8_t state;
    uint8_t active_type;
    uint8_t reserved[2]; /* zero */
    uint32_t flags;
    uint32_t active_trajectory_id;
    uint32_t active_revision;
} xgc2_px4_reference_status_v1;

/* "xgc2.px4.reference_reset.v1": std_msgs/Empty. The event itself is the request. */
typedef struct xgc2_px4_reference_reset_v1 {
    uint64_t reserved; /* zero */
} xgc2_px4_reference_reset_v1;

#ifdef __cplusplus
#define XGC2_PX4_REFERENCE_ASSERT(condition, message) static_assert(condition, message)
#else
#define XGC2_PX4_REFERENCE_ASSERT(condition, message) _Static_assert(condition, message)
#endif

XGC2_PX4_REFERENCE_ASSERT(sizeof(xgc2_px4_reference_header_v1) == 48, "reference header size");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_header_v1, seq) == 0, "header.seq");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_header_v1, stamp_sec) == 4,
                          "header.stamp_sec");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_header_v1, stamp_nsec) == 8,
                          "header.stamp_nsec");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_header_v1, reserved) == 12,
                          "header.reserved");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_header_v1, frame_id) == 16,
                          "header.frame_id");

XGC2_PX4_REFERENCE_ASSERT(sizeof(xgc2_px4_reference_analytic_v1) == 272, "analytic size");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, request_id) == 48,
                          "analytic.request_id");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, trajectory_id) == 52,
                          "analytic.trajectory_id");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, revision) == 56,
                          "analytic.revision");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, flags) == 60, "analytic.flags");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, start_sec) == 64,
                          "analytic.start_sec");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, start_nsec) == 68,
                          "analytic.start_nsec");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, analytic_type) == 72,
                          "analytic.analytic_type");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, params_len) == 74,
                          "analytic.params_len");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, duration) == 80,
                          "analytic.duration");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, origin_position) == 88,
                          "analytic.origin_position");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, origin_q_xyzw) == 112,
                          "analytic.origin_q_xyzw");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_analytic_v1, params) == 144,
                          "analytic.params");

XGC2_PX4_REFERENCE_ASSERT(sizeof(xgc2_px4_reference_point_v1) == 152, "point size");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_point_v1, position) == 8, "point.position");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_point_v1, velocity) == 32, "point.velocity");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_point_v1, acceleration) == 56,
                          "point.acceleration");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_point_v1, jerk) == 80, "point.jerk");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_point_v1, snap) == 104, "point.snap");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_point_v1, yaw) == 128, "point.yaw");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_point_v1, yaw_rate) == 136, "point.yaw_rate");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_point_v1, yaw_accel) == 144,
                          "point.yaw_accel");

XGC2_PX4_REFERENCE_ASSERT(sizeof(xgc2_px4_reference_sampled_v1) == 80 + 152 * 1024, "sampled size");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_sampled_v1, trajectory_id) == 48,
                          "sampled.trajectory_id");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_sampled_v1, revision) == 52,
                          "sampled.revision");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_sampled_v1, flags) == 56, "sampled.flags");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_sampled_v1, points_len) == 60,
                          "sampled.points_len");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_sampled_v1, start_sec) == 64,
                          "sampled.start_sec");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_sampled_v1, start_nsec) == 68,
                          "sampled.start_nsec");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_sampled_v1, sample_dt) == 72,
                          "sampled.sample_dt");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_sampled_v1, points) == 80, "sampled.points");

XGC2_PX4_REFERENCE_ASSERT(sizeof(xgc2_px4_reference_status_v1) == 64, "status size");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_status_v1, state) == 48, "status.state");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_status_v1, active_type) == 49,
                          "status.active_type");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_status_v1, flags) == 52, "status.flags");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_status_v1, active_trajectory_id) == 56,
                          "status.active_trajectory_id");
XGC2_PX4_REFERENCE_ASSERT(offsetof(xgc2_px4_reference_status_v1, active_revision) == 60,
                          "status.active_revision");

XGC2_PX4_REFERENCE_ASSERT(sizeof(xgc2_px4_reference_reset_v1) == 8, "reset size");

#undef XGC2_PX4_REFERENCE_ASSERT

#ifdef __cplusplus
}
#endif

#endif /* MULTIROTOR_REFERENCE_TRAJECTORY_PAYLOADS_H */
