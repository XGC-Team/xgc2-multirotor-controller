#pragma once

#include <cstdint>

namespace px4_multirotor_controller {
namespace state_estimate {

// Estimator state and flag values of rigid_state_estimator_msgs/
// RigidStateEstimate, for the core without the message type. The ROS edge
// (sensor_input_producer.cpp) static_asserts that they match the message.
constexpr uint8_t STATE_SELF_CHECK = 1;
constexpr uint8_t STATE_INITIALIZING = 2;
constexpr uint8_t STATE_RUNNING = 3;
constexpr uint8_t STATE_COASTING = 4;
constexpr uint8_t STATE_FAULT = 5;

constexpr uint32_t FLAG_IMU_MISSING = 1u << 0;
constexpr uint32_t FLAG_VRPN_MISSING = 1u << 1;
constexpr uint32_t FLAG_IMU_STALE = 1u << 2;
constexpr uint32_t FLAG_VRPN_STALE = 1u << 3;
constexpr uint32_t FLAG_IMU_RATE_LOW = 1u << 4;
constexpr uint32_t FLAG_VRPN_RATE_LOW = 1u << 5;
constexpr uint32_t FLAG_TIME_JUMP = 1u << 6;
constexpr uint32_t FLAG_COASTING = 1u << 7;
constexpr uint32_t FLAG_FAULT = 1u << 8;
constexpr uint32_t FLAG_INNOVATION_REJECTED = 1u << 9;
constexpr uint32_t FLAG_EXTRINSIC_UNVERIFIED = 1u << 10;
constexpr uint32_t FLAG_COVARIANCE_HIGH = 1u << 11;
constexpr uint32_t FLAG_INVALID_IMU = 1u << 12;
constexpr uint32_t FLAG_INVALID_VRPN = 1u << 13;
constexpr uint32_t FLAG_POSE_TIME_ALIGNMENT_REJECTED = 1u << 14;
constexpr uint32_t FLAG_VRPN_SUSPECTED = 1u << 15;
constexpr uint32_t FLAG_VRPN_FAULT = 1u << 16;
constexpr uint32_t FLAG_VRPN_RECOVERY = 1u << 17;
constexpr uint32_t FLAG_FILTER_DEGRADED = 1u << 18;
constexpr uint32_t FLAG_FILTER_IMU_ONLY = 1u << 19;

}  // namespace state_estimate
}  // namespace px4_multirotor_controller
