#pragma once

#include <cmath>

#include "px4_multirotor_controller/common/state_estimate_status.h"
#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {
namespace sensor_checks {

constexpr double kAirborneAltitudeThreshold = 0.3;

inline bool isControlStateActive(const SensorData& sensor) {
    return sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.uav_state_estimate_stats.is_active
               : sensor.local_pos_stats.is_active && sensor.local_velocity_stats.is_active &&
                     sensor.imu_stats.is_active;
}

inline bool isControlStateNew(const SensorData& sensor) {
    return sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.uav_state_estimate_stats.is_new
               : sensor.local_pos_stats.is_new || sensor.local_velocity_stats.is_new ||
                     sensor.imu_stats.is_new;
}

inline bool isWorldPoseNew(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) ? isControlStateNew(sensor)
                                              : sensor.local_pos_stats.is_new;
}

inline bool areBaseSensorsActive(const SensorData& sensor) {
    // Battery is Adapter/instrument/rosbag telemetry (W15 / T7). It is not readiness
    // and must not force Landing.
    return isControlStateActive(sensor) && sensor.state_stats.is_active;
}

inline bool arePassThroughSensorsActive(const SensorData& sensor) {
    return sensor.local_pos_stats.is_active && sensor.local_velocity_stats.is_active &&
           sensor.imu_stats.is_active && sensor.state_stats.is_active;
}

inline bool isStateEstimateUsableForControl(const SensorData& sensor) {
    constexpr uint32_t kBlockingFlags =
        state_estimate::FLAG_IMU_MISSING | state_estimate::FLAG_VRPN_MISSING |
        state_estimate::FLAG_IMU_STALE | state_estimate::FLAG_VRPN_STALE |
        state_estimate::FLAG_IMU_RATE_LOW | state_estimate::FLAG_VRPN_RATE_LOW |
        state_estimate::FLAG_TIME_JUMP | state_estimate::FLAG_FAULT |
        state_estimate::FLAG_EXTRINSIC_UNVERIFIED | state_estimate::FLAG_INVALID_IMU |
        state_estimate::FLAG_INVALID_VRPN | state_estimate::FLAG_VRPN_FAULT |
        state_estimate::FLAG_FILTER_IMU_ONLY;
    const bool state_ok = sensor.uav_state_estimator_state == state_estimate::STATE_RUNNING ||
                          sensor.uav_state_estimator_state == state_estimate::STATE_COASTING;
    return sensor.uav_state_estimate_stats.is_active && state_ok &&
           (sensor.uav_state_estimator_flags & kBlockingFlags) == 0u;
}

inline bool isControlStateUsableForControl(const SensorData& sensor) {
    if (sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE)
        return isStateEstimateUsableForControl(sensor);
    const double norm2 = sensor.local_qx * sensor.local_qx + sensor.local_qy * sensor.local_qy +
                         sensor.local_qz * sensor.local_qz + sensor.local_qw * sensor.local_qw;
    return isControlStateActive(sensor) && sensor.mavros_inertial_valid &&
           std::isfinite(sensor.local_x) && std::isfinite(sensor.local_y) &&
           std::isfinite(sensor.local_z) && std::isfinite(sensor.local_vx) &&
           std::isfinite(sensor.local_vy) && std::isfinite(sensor.local_vz) &&
           std::isfinite(norm2) && norm2 > 1.0e-18;
}

inline bool areSensorsAllActive(const SensorData& sensor) {
    return areBaseSensorsActive(sensor) && isControlStateUsableForControl(sensor);
}

inline bool areSensorsReady(const SensorData& sensor, TrackingBackend backend) {
    if (!trackingUsesFusedEstimate(backend)) {
        return arePassThroughSensorsActive(sensor);
    }
    return areSensorsAllActive(sensor);
}

inline double worldX(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.x
               : sensor.local_x;
}

inline double worldY(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.y
               : sensor.local_y;
}

inline double worldZ(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.z
               : sensor.local_z;
}

inline double worldVx(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.vx
               : sensor.local_vx;
}

inline double worldVy(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.vy
               : sensor.local_vy;
}

inline double worldVz(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.vz
               : sensor.local_vz;
}

// Same declared control frame and feedback source as world position/velocity.
inline double worldQx(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.qx
               : sensor.local_qx;
}
inline double worldQy(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.qy
               : sensor.local_qy;
}
inline double worldQz(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.qz
               : sensor.local_qz;
}
inline double worldQw(const SensorData& sensor, TrackingBackend backend) {
    return trackingUsesFusedEstimate(backend) &&
                   sensor.feedback_source == FeedbackSource::RIGID_ESTIMATE
               ? sensor.qw
               : sensor.local_qw;
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

inline bool isAirborne(const SensorData& sensor, TrackingBackend backend) {
    return sensor.fcu_armed && worldZ(sensor, backend) > kAirborneAltitudeThreshold;
}

}  // namespace sensor_checks
}  // namespace px4_multirotor_controller
