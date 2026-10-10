/*
 * Payloads of the px4_multirotor_controller module (xgc2-module module ABI).
 *
 * Fixed-size POD structs, C11 and C++ compatible. They are the controller's own input and output
 * vocabulary: what the entity's ROS edge (MAVROS, the state estimator, the hover thrust estimator,
 * the operator's /command) hands to the controller, and what the controller hands back. They never
 * leave the process: the host passes them from the producer's slot to the readers without copying
 * or serializing. Layout, field order and widths of a schema never change; a different layout gets
 * a new schema id.
 *
 * The reference trajectory payloads (the controller consumes the active reference and requests
 * one) are in multirotor_reference_trajectory/payloads.h.
 *
 * Time. A message stamp is the ROS header stamp of the message the payload was made from, in the
 * host clock domain, as sec/nsec (zero when the message had no stamp). The sample's own stamp_ns,
 * given by the producer when it commits, is the time the sample was received; the controller uses
 * that one for its receive statistics and for the time of its input events. The estimator and hover
 * thrust stamps are compared with the controller's clock, so they must be in the host clock domain.
 * Orientations are quaternions in x, y, z, w order, as in the ROS messages.
 */
#ifndef PX4_MULTIROTOR_CONTROLLER_PAYLOADS_H
#define PX4_MULTIROTOR_CONTROLLER_PAYLOADS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XGC2_PX4_STATE_ESTIMATE_SCHEMA "xgc2.px4.state_estimate.v1"
#define XGC2_PX4_POSE_SCHEMA "xgc2.px4.pose.v1"
#define XGC2_PX4_VELOCITY_SCHEMA "xgc2.px4.velocity.v1"
#define XGC2_PX4_IMU_SCHEMA "xgc2.px4.imu.v1"
#define XGC2_PX4_FCU_STATE_SCHEMA "xgc2.px4.fcu_state.v1"
#define XGC2_PX4_BATTERY_SCHEMA "xgc2.px4.battery.v1"
#define XGC2_PX4_COMMAND_SCHEMA "xgc2.px4.command.v1"
#define XGC2_PX4_POSITION_TARGET_SCHEMA "xgc2.px4.position_target.v1"
#define XGC2_PX4_HOVER_THRUST_SCHEMA "xgc2.px4.hover_thrust.v1"
#define XGC2_PX4_ATTITUDE_RATE_TARGET_SCHEMA "xgc2.px4.attitude_rate_target.v1"
#define XGC2_PX4_FCU_REQUEST_SCHEMA "xgc2.px4.fcu_request.v1"
#define XGC2_PX4_CONTROLLER_STATUS_SCHEMA "xgc2.px4.controller_status.v1"

/* ---------------------------------------------------------------- inputs */

/*
 * "xgc2.px4.state_estimate.v1": the fused state of rigid_state_estimator_msgs/RigidStateEstimate
 * that the controller reads (DFBC and NMPC feedback; SMC and PX4_LOCAL do not use it).
 * estimator_state and flags are the estimator's STATE_* and FLAG_* values
 * (common/state_estimate_status.h). The three *_stamp_sec fields are the message's float64 stamps.
 */
typedef struct xgc2_px4_state_estimate_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  double position[3];
  double velocity[3];
  double orientation_xyzw[4];
  double angular_velocity[3];
  double linear_acceleration[3];
  double gravity[3];
  double accel_bias[3];
  double filter_inertial_stamp_sec;
  double filter_pose_stamp_sec;
  double last_vrpn_pose_stamp_sec;
  uint32_t flags;
  uint8_t estimator_state;
  uint8_t reserved[3]; /* zero */
} xgc2_px4_state_estimate_v1;

/*
 * "xgc2.px4.pose.v1": geometry_msgs/PoseStamped. Ports local_pose (mavros/local_position/pose) and
 * vrpn_pose (the canonical pose of the entity, topic `pose`).
 */
typedef struct xgc2_px4_pose_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  double position[3];
  double orientation_xyzw[4];
} xgc2_px4_pose_v1;

/* "xgc2.px4.velocity.v1": the linear velocity of mavros/local_position/velocity_local. */
typedef struct xgc2_px4_velocity_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  double linear[3];
} xgc2_px4_velocity_v1;

/*
 * "xgc2.px4.imu.v1": a sample of mavros/imu/data. The controller reads no IMU value; its health
 * checks use that samples arrive and how often.
 */
typedef struct xgc2_px4_imu_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
} xgc2_px4_imu_v1;

/* "xgc2.px4.fcu_state.v1": mavros_msgs/State. mode is NUL-terminated, e.g. "OFFBOARD". */
typedef struct xgc2_px4_fcu_state_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  uint8_t connected;
  uint8_t armed;
  uint8_t guided;
  uint8_t manual_input;
  uint8_t system_status; /* MAV_STATE */
  uint8_t reserved[3];   /* zero */
  char mode[32];
} xgc2_px4_fcu_state_v1;

/* "xgc2.px4.battery.v1": sensor_msgs/BatteryState.percentage, 0..1 (telemetry only). */
typedef struct xgc2_px4_battery_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  double percentage;
} xgc2_px4_battery_v1;

/*
 * "xgc2.px4.command.v1": the operator's /command string ("takeoff", "land", "hover", "custom1" and
 * their aliases, see driver/command_input.h), NUL-terminated.
 */
typedef struct xgc2_px4_command_v1 {
  char text[64];
} xgc2_px4_command_v1;

/*
 * "xgc2.px4.hover_thrust.v1": the hover thrust estimate of
 * hover_thrust_estimator_msgs/HoverThrustEstimate. A zero stamp means unstamped: the controller then
 * uses the time it received the sample.
 */
typedef struct xgc2_px4_hover_thrust_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  double hover_thrust;
  uint32_t flags;
  uint32_t reserved; /* zero */
} xgc2_px4_hover_thrust_v1;

/* ---------------------------------------------------------------- inputs and outputs */

/*
 * "xgc2.px4.position_target.v1": mavros_msgs/PositionTarget. As an input (port alg_setpoint) it is a
 * planner's setpoint, from alg/setpoint_raw/local; as an output (port setpoint) it is the
 * controller's setpoint for mavros/setpoint_raw/local, stamped with the time of the control step.
 * acceleration is acceleration_or_force. type_mask and coordinate_frame are the MAVROS values.
 */
typedef struct xgc2_px4_position_target_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  double position[3];
  double velocity[3];
  double acceleration[3];
  double yaw;
  double yaw_rate;
  uint16_t type_mask;
  uint8_t coordinate_frame;
  uint8_t reserved[5]; /* zero */
} xgc2_px4_position_target_v1;

/* ---------------------------------------------------------------- outputs */

/*
 * "xgc2.px4.attitude_rate_target.v1": the body-rate and thrust command of the DFBC and NMPC
 * backends (mavros/setpoint_raw/attitude with IGNORE_ATTITUDE). thrust is normalized and already
 * limited to [0, 1], as the ROS node publishes it.
 */
typedef struct xgc2_px4_attitude_rate_target_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  double body_rate[3];
  double thrust;
} xgc2_px4_attitude_rate_target_v1;

#define XGC2_PX4_FCU_REQUEST_ARM 1u  /* mavros/cmd/command, MAV_CMD_COMPONENT_ARM_DISARM (400) */
#define XGC2_PX4_FCU_REQUEST_MODE 2u /* mavros/set_mode */

/*
 * "xgc2.px4.fcu_request.v1": a flight controller service request the edge makes for the
 * controller. kind XGC2_PX4_FCU_REQUEST_ARM: arm is 1 to arm, 0 to disarm. kind
 * XGC2_PX4_FCU_REQUEST_MODE: mode is the custom mode, NUL-terminated.
 */
typedef struct xgc2_px4_fcu_request_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  uint32_t kind;
  uint32_t arm;
  char mode[32];
} xgc2_px4_fcu_request_v1;

/* "xgc2.px4.controller_status.v1": the name of the controller's flight state, NUL-terminated. */
typedef struct xgc2_px4_controller_status_v1 {
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  char state[48];
} xgc2_px4_controller_status_v1;

#ifdef __cplusplus
#define XGC2_PX4_ASSERT(condition, message) static_assert(condition, message)
#else
#define XGC2_PX4_ASSERT(condition, message) _Static_assert(condition, message)
#endif

XGC2_PX4_ASSERT(sizeof(xgc2_px4_state_estimate_v1) == 216, "state_estimate size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, position) == 8, "state_estimate.position");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, velocity) == 32, "state_estimate.velocity");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, orientation_xyzw) == 56,
                "state_estimate.orientation_xyzw");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, angular_velocity) == 88,
                "state_estimate.angular_velocity");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, linear_acceleration) == 112,
                "state_estimate.linear_acceleration");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, gravity) == 136, "state_estimate.gravity");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, accel_bias) == 160,
                "state_estimate.accel_bias");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, filter_inertial_stamp_sec) == 184,
                "state_estimate.filter_inertial_stamp_sec");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, filter_pose_stamp_sec) == 192,
                "state_estimate.filter_pose_stamp_sec");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, last_vrpn_pose_stamp_sec) == 200,
                "state_estimate.last_vrpn_pose_stamp_sec");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, flags) == 208, "state_estimate.flags");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_state_estimate_v1, estimator_state) == 212,
                "state_estimate.estimator_state");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_pose_v1) == 64, "pose size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_pose_v1, position) == 8, "pose.position");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_pose_v1, orientation_xyzw) == 32, "pose.orientation_xyzw");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_velocity_v1) == 32, "velocity size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_velocity_v1, linear) == 8, "velocity.linear");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_imu_v1) == 8, "imu size");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_fcu_state_v1) == 48, "fcu_state size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_fcu_state_v1, connected) == 8, "fcu_state.connected");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_fcu_state_v1, system_status) == 12, "fcu_state.system_status");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_fcu_state_v1, mode) == 16, "fcu_state.mode");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_battery_v1) == 16, "battery size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_battery_v1, percentage) == 8, "battery.percentage");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_command_v1) == 64, "command size");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_hover_thrust_v1) == 24, "hover_thrust size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_hover_thrust_v1, hover_thrust) == 8, "hover_thrust.hover_thrust");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_hover_thrust_v1, flags) == 16, "hover_thrust.flags");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_position_target_v1) == 104, "position_target size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_position_target_v1, position) == 8, "position_target.position");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_position_target_v1, velocity) == 32, "position_target.velocity");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_position_target_v1, acceleration) == 56,
                "position_target.acceleration");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_position_target_v1, yaw) == 80, "position_target.yaw");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_position_target_v1, yaw_rate) == 88, "position_target.yaw_rate");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_position_target_v1, type_mask) == 96, "position_target.type_mask");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_position_target_v1, coordinate_frame) == 98,
                "position_target.coordinate_frame");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_attitude_rate_target_v1) == 40, "attitude_rate_target size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_attitude_rate_target_v1, body_rate) == 8,
                "attitude_rate_target.body_rate");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_attitude_rate_target_v1, thrust) == 32,
                "attitude_rate_target.thrust");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_fcu_request_v1) == 48, "fcu_request size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_fcu_request_v1, kind) == 8, "fcu_request.kind");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_fcu_request_v1, arm) == 12, "fcu_request.arm");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_fcu_request_v1, mode) == 16, "fcu_request.mode");

XGC2_PX4_ASSERT(sizeof(xgc2_px4_controller_status_v1) == 56, "controller_status size");
XGC2_PX4_ASSERT(offsetof(xgc2_px4_controller_status_v1, state) == 8, "controller_status.state");

#undef XGC2_PX4_ASSERT

#ifdef __cplusplus
}
#endif

#endif /* PX4_MULTIROTOR_CONTROLLER_PAYLOADS_H */
