#pragma once

#include <rigid_state_estimator_msgs/RigidStateEstimate.h>

#include <cmath>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {
namespace sensor_checks {

constexpr double kAirborneAltitudeThreshold = 0.3;

inline bool isControlStateActive(const SensorData& sensor) {
    return sensor.uav_state_estimate_stats.is_active;
}

inline bool isControlStateNew(const SensorData& sensor) {
    return sensor.uav_state_estimate_stats.is_new;
}

inline bool areBaseSensorsActive(const SensorData& sensor) {
    return isControlStateActive(sensor) && sensor.state_stats.is_active &&
           sensor.battery_stats.is_active;
}

inline double vrpnLocalPositionDiff(const SensorData& sensor) {
    const double dx = sensor.vrpn_x - sensor.local_x;
    const double dy = sensor.vrpn_y - sensor.local_y;
    const double dz = sensor.vrpn_z - sensor.local_z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline bool isVrpnPoseConsistent(const SensorData& sensor, double position_tolerance = 0.05) {
    return vrpnLocalPositionDiff(sensor) < position_tolerance;
}

inline bool isStateEstimateUsableForControl(const SensorData& sensor) {
    constexpr uint32_t kBlockingFlags =
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_IMU_MISSING |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_VRPN_MISSING |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_IMU_STALE |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_VRPN_STALE |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_IMU_RATE_LOW |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_VRPN_RATE_LOW |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_TIME_JUMP |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_FAULT |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_EXTRINSIC_UNVERIFIED |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_INVALID_IMU |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_INVALID_VRPN |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_VRPN_FAULT |
        rigid_state_estimator_msgs::RigidStateEstimate::FLAG_FILTER_IMU_ONLY;
    const bool state_ok = sensor.uav_state_estimator_state ==
                              rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING ||
                          sensor.uav_state_estimator_state ==
                              rigid_state_estimator_msgs::RigidStateEstimate::STATE_COASTING;
    return sensor.uav_state_estimate_stats.is_active && state_ok &&
           (sensor.uav_state_estimator_flags & kBlockingFlags) == 0u;
}

inline bool isControlStateUsableForControl(const SensorData& sensor) {
    return isStateEstimateUsableForControl(sensor);
}

inline bool areSensorsAllActive(const SensorData& sensor) {
    return areBaseSensorsActive(sensor) && isControlStateUsableForControl(sensor);
}

inline bool isFcuConnected(const SensorData& sensor) {
    return sensor.fcu_connected;
}

inline bool isFcuArmed(const SensorData& sensor) {
    return sensor.fcu_armed;
}

inline bool hasManualInput(const SensorData& sensor) {
    return sensor.fcu_manual_input;
}

inline bool isOffboardMode(const SensorData& sensor) {
    return sensor.fcu_mode == "OFFBOARD";
}

inline bool isAirborne(const SensorData& sensor) {
    return sensor.fcu_armed && sensor.z > kAirborneAltitudeThreshold;
}

}  // namespace sensor_checks
}  // namespace px4_multirotor_controller
