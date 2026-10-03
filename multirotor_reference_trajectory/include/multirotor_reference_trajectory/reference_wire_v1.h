#ifndef MULTIROTOR_REFERENCE_TRAJECTORY_REFERENCE_WIRE_V1_H
#define MULTIROTOR_REFERENCE_TRAJECTORY_REFERENCE_WIRE_V1_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Owned xgc.ref.* v1 wire layouts. Keep field order and widths stable. */
typedef struct xgc_ref_header_v1 {
  uint32_t seq;
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  uint32_t reserved;
  char frame_id[32];
} xgc_ref_header_v1;

typedef struct xgc_ref_analytic_v1 {
  xgc_ref_header_v1 header;
  uint32_t request_id;
  uint32_t trajectory_id;
  uint32_t revision;
  uint32_t flags;
  uint32_t start_sec;
  uint32_t start_nsec;
  uint16_t analytic_type;
  uint16_t reserved;
  uint32_t params_len;
  double duration;
  double origin_position[3];
  double origin_q_xyzw[4];
} xgc_ref_analytic_v1;

typedef struct xgc_ref_flat_point_v1 {
  double t_from_start;
  double position[3];
  double velocity[3];
  double acceleration[3];
  double jerk[3];
  double snap[3];
  double yaw;
  double yaw_rate;
  double yaw_accel;
} xgc_ref_flat_point_v1;

typedef struct xgc_ref_sampled_v1 {
  xgc_ref_header_v1 header;
  uint32_t trajectory_id;
  uint32_t revision;
  uint32_t flags;
  uint32_t points_len;
  uint32_t start_sec;
  uint32_t start_nsec;
  double sample_dt;
} xgc_ref_sampled_v1;

typedef struct xgc_ref_status_v1 {
  xgc_ref_header_v1 header;
  uint8_t state;
  uint8_t active_type;
  uint8_t reserved[2];
  uint32_t flags;
  uint32_t active_trajectory_id;
  uint32_t active_revision;
} xgc_ref_status_v1;

typedef struct xgc_ref_reset_v1 {
  uint64_t reserved;
} xgc_ref_reset_v1;

/* Canonical flat reference payload, also owned by this product's wire API. */
typedef struct xgc_flat_ref_v1 {
  double stamp;
  double position[3];
  double velocity[3];
  double acceleration[3];
  double jerk[3];
  double snap[3];
  double yaw;
  double yaw_rate;
  double yaw_accel;
  uint32_t flags;
  uint32_t reserved;
} xgc_flat_ref_v1;

#ifdef __cplusplus
#define MRT_WIRE_ASSERT static_assert
#else
#define MRT_WIRE_ASSERT _Static_assert
#endif

MRT_WIRE_ASSERT(sizeof(xgc_ref_header_v1) == 48, "xgc_ref_header_v1 size");
MRT_WIRE_ASSERT(offsetof(xgc_ref_header_v1, seq) == 0, "header.seq offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_header_v1, stamp_sec) == 4, "header.stamp_sec offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_header_v1, stamp_nsec) == 8, "header.stamp_nsec offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_header_v1, reserved) == 12, "header.reserved offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_header_v1, frame_id) == 16, "header.frame_id offset");

MRT_WIRE_ASSERT(sizeof(xgc_ref_analytic_v1) == 144, "xgc_ref_analytic_v1 size");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, request_id) == 48, "analytic.request_id offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, trajectory_id) == 52, "analytic.trajectory_id offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, revision) == 56, "analytic.revision offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, flags) == 60, "analytic.flags offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, start_sec) == 64, "analytic.start_sec offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, start_nsec) == 68, "analytic.start_nsec offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, analytic_type) == 72, "analytic.analytic_type offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, params_len) == 76, "analytic.params_len offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, duration) == 80, "analytic.duration offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, origin_position) == 88, "analytic.origin_position offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_analytic_v1, origin_q_xyzw) == 112, "analytic.origin_q_xyzw offset");

MRT_WIRE_ASSERT(sizeof(xgc_ref_flat_point_v1) == 152, "xgc_ref_flat_point_v1 size");
MRT_WIRE_ASSERT(offsetof(xgc_ref_flat_point_v1, t_from_start) == 0, "flat_point.t_from_start offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_flat_point_v1, position) == 8, "flat_point.position offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_flat_point_v1, velocity) == 32, "flat_point.velocity offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_flat_point_v1, acceleration) == 56, "flat_point.acceleration offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_flat_point_v1, jerk) == 80, "flat_point.jerk offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_flat_point_v1, snap) == 104, "flat_point.snap offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_flat_point_v1, yaw) == 128, "flat_point.yaw offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_flat_point_v1, yaw_rate) == 136, "flat_point.yaw_rate offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_flat_point_v1, yaw_accel) == 144, "flat_point.yaw_accel offset");

MRT_WIRE_ASSERT(sizeof(xgc_ref_sampled_v1) == 80, "xgc_ref_sampled_v1 size");
MRT_WIRE_ASSERT(offsetof(xgc_ref_sampled_v1, trajectory_id) == 48, "sampled.trajectory_id offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_sampled_v1, revision) == 52, "sampled.revision offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_sampled_v1, flags) == 56, "sampled.flags offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_sampled_v1, points_len) == 60, "sampled.points_len offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_sampled_v1, start_sec) == 64, "sampled.start_sec offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_sampled_v1, start_nsec) == 68, "sampled.start_nsec offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_sampled_v1, sample_dt) == 72, "sampled.sample_dt offset");

MRT_WIRE_ASSERT(sizeof(xgc_ref_status_v1) == 64, "xgc_ref_status_v1 size");
MRT_WIRE_ASSERT(offsetof(xgc_ref_status_v1, state) == 48, "status.state offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_status_v1, active_type) == 49, "status.active_type offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_status_v1, reserved) == 50, "status.reserved offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_status_v1, flags) == 52, "status.flags offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_status_v1, active_trajectory_id) == 56, "status.active_trajectory_id offset");
MRT_WIRE_ASSERT(offsetof(xgc_ref_status_v1, active_revision) == 60, "status.active_revision offset");
MRT_WIRE_ASSERT(sizeof(xgc_ref_reset_v1) == 8, "xgc_ref_reset_v1 size");
MRT_WIRE_ASSERT(offsetof(xgc_ref_reset_v1, reserved) == 0, "reset.reserved offset");

MRT_WIRE_ASSERT(sizeof(xgc_flat_ref_v1) == 160, "xgc_flat_ref_v1 size");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, stamp) == 0, "flat_ref.stamp offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, position) == 8, "flat_ref.position offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, velocity) == 32, "flat_ref.velocity offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, acceleration) == 56, "flat_ref.acceleration offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, jerk) == 80, "flat_ref.jerk offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, snap) == 104, "flat_ref.snap offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, yaw) == 128, "flat_ref.yaw offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, yaw_rate) == 136, "flat_ref.yaw_rate offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, yaw_accel) == 144, "flat_ref.yaw_accel offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, flags) == 152, "flat_ref.flags offset");
MRT_WIRE_ASSERT(offsetof(xgc_flat_ref_v1, reserved) == 156, "flat_ref.reserved offset");

#undef MRT_WIRE_ASSERT

#ifdef __cplusplus
}
#endif

#endif
