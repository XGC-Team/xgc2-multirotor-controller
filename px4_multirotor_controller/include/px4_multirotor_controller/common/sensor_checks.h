#pragma once

#include <estimator_vrpn_px4_rotor_state/RigidStateEstimate.h>

#include <cmath>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {
namespace sensor_checks {

constexpr double kAirborneAltitudeThreshold = 0.3;

inline bool areBaseSensorsActive(const SensorData& sensor) {
    return sensor.uav_state_estimate_stats.is_active && sensor.state_stats.is_active &&
           sensor.battery_stats.is_active;
}

inline bool areVrpnTopicsActive(const SensorData& sensor) {
    return sensor.vrpn_pose_stats.is_active && sensor.vrpn_twist_stats.is_active;
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
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_IMU_MISSING |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_IMU_STALE |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_IMU_RATE_LOW |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_TIME_JUMP |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_FAULT |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_EXTRINSIC_UNVERIFIED |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_COVARIANCE_HIGH |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_INVALID_IMU;
    const bool state_ok = sensor.uav_state_estimator_state ==
                              estimator_vrpn_px4_rotor_state::RigidStateEstimate::STATE_RUNNING ||
                          sensor.uav_state_estimator_state ==
                              estimator_vrpn_px4_rotor_state::RigidStateEstimate::STATE_COASTING;
    return sensor.uav_state_estimate_stats.is_active && state_ok &&
           (sensor.uav_state_estimator_flags & kBlockingFlags) == 0u;
}

inline bool areSensorsAllActive(const SensorData& sensor) {
    return areBaseSensorsActive(sensor) && isStateEstimateUsableForControl(sensor);
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
