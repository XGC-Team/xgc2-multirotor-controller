#pragma once

#include <cmath>
#include <cstdint>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

enum class PoseIngressResult {
    RejectedInvalid = 0,
    AcceptedNonFreshStamp = 1,
    AcceptedFresh = 2,
};

inline bool isValidSe3Pose(double x, double y, double z, double qx, double qy, double qz,
                           double qw) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(qx) ||
        !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw)) {
        return false;
    }
    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    return std::isfinite(norm) && std::abs(norm - 1.0) <= 0.05;
}

inline void pushPoseSample(SensorData::PoseRing& ring, const SensorData::PoseSample& sample) {
    ring.samples[ring.head] = sample;
    ring.head = (ring.head + 1U) % SensorData::PoseRing::kCapacity;
    if (ring.count < SensorData::PoseRing::kCapacity) {
        ++ring.count;
    }
}

inline bool isFreshPoseTimestamp(bool has_header_stamp, uint64_t stamp_ns, double rx_sec,
                                 const std::optional<uint64_t>& last_stamp_ns,
                                 const std::optional<double>& last_rx_s) {
    constexpr uint64_t kClockBackstepResetNs = 5000000000ULL;
    if (has_header_stamp) {
        if (!last_stamp_ns) {
            return true;
        }
        if (stamp_ns > *last_stamp_ns) {
            return true;
        }
        return (*last_stamp_ns - stamp_ns) > kClockBackstepResetNs &&
               (!last_rx_s || rx_sec >= *last_rx_s);
    }
    if (!last_rx_s) {
        return true;
    }
    return rx_sec > *last_rx_s;
}

inline PoseIngressResult ingestLocalPoseSample(SensorData& sensor, uint32_t stamp_sec,
                                               uint32_t stamp_nsec, double rx_sec, double x,
                                               double y, double z, double qx, double qy,
                                               double qz, double qw) {
    if (!isValidSe3Pose(x, y, z, qx, qy, qz, qw)) {
        sensor.local_pose_invalid = true;
        ++sensor.local_pose_invalid_count;
        return PoseIngressResult::RejectedInvalid;
    }
    const bool has_header_stamp = (stamp_sec != 0U || stamp_nsec != 0U);
    const uint64_t stamp_ns =
        static_cast<uint64_t>(stamp_sec) * 1000000000ULL + static_cast<uint64_t>(stamp_nsec);
    const bool fresh = isFreshPoseTimestamp(has_header_stamp, stamp_ns, rx_sec,
                                            sensor.last_valid_local_stamp_ns,
                                            sensor.last_valid_local_rx_s);

    SensorData::PoseSample sample;
    sample.seq = ++sensor.local_pose_seq;
    sample.stamp_ns = stamp_ns;
    sample.stamp_s = static_cast<double>(stamp_sec) + static_cast<double>(stamp_nsec) * 1e-9;
    sample.rx_s = rx_sec;
    sample.has_header_stamp = has_header_stamp;
    sample.x = x;
    sample.y = y;
    sample.z = z;
    sample.qx = qx;
    sample.qy = qy;
    sample.qz = qz;
    sample.qw = qw;
    pushPoseSample(sensor.local_pose_ring, sample);

    if (!fresh) {
        return PoseIngressResult::AcceptedNonFreshStamp;
    }

    sensor.local_pose_invalid = false;

    sensor.local_x = x;
    sensor.local_y = y;
    sensor.local_z = z;
    sensor.local_qx = qx;
    sensor.local_qy = qy;
    sensor.local_qz = qz;
    sensor.local_qw = qw;
    sensor.last_valid_local_rx_s = rx_sec;
    if (has_header_stamp) {
        sensor.last_valid_local_stamp_ns = stamp_ns;
    }
    return PoseIngressResult::AcceptedFresh;
}

inline PoseIngressResult ingestVrpnPoseSample(SensorData& sensor, uint32_t stamp_sec,
                                              uint32_t stamp_nsec, double rx_sec, double x,
                                              double y, double z, double qx, double qy,
                                              double qz, double qw) {
    if (!isValidSe3Pose(x, y, z, qx, qy, qz, qw)) {
        sensor.vrpn_pose_invalid = true;
        ++sensor.vrpn_pose_invalid_count;
        return PoseIngressResult::RejectedInvalid;
    }
    const bool has_header_stamp = (stamp_sec != 0U || stamp_nsec != 0U);
    const uint64_t stamp_ns =
        static_cast<uint64_t>(stamp_sec) * 1000000000ULL + static_cast<uint64_t>(stamp_nsec);
    const bool fresh = isFreshPoseTimestamp(has_header_stamp, stamp_ns, rx_sec,
                                            sensor.last_valid_vrpn_stamp_ns,
                                            sensor.last_valid_vrpn_rx_s);

    SensorData::PoseSample sample;
    sample.seq = ++sensor.vrpn_pose_seq;
    sample.stamp_ns = stamp_ns;
    sample.stamp_s = static_cast<double>(stamp_sec) + static_cast<double>(stamp_nsec) * 1e-9;
    sample.rx_s = rx_sec;
    sample.has_header_stamp = has_header_stamp;
    sample.x = x;
    sample.y = y;
    sample.z = z;
    sample.qx = qx;
    sample.qy = qy;
    sample.qz = qz;
    sample.qw = qw;
    pushPoseSample(sensor.vrpn_pose_ring, sample);

    if (!fresh) {
        return PoseIngressResult::AcceptedNonFreshStamp;
    }

    sensor.vrpn_pose_invalid = false;

    sensor.vrpn_x = x;
    sensor.vrpn_y = y;
    sensor.vrpn_z = z;
    sensor.vrpn_qx = qx;
    sensor.vrpn_qy = qy;
    sensor.vrpn_qz = qz;
    sensor.vrpn_qw = qw;
    sensor.last_valid_vrpn_rx_s = rx_sec;
    if (has_header_stamp) {
        sensor.last_valid_vrpn_stamp_ns = stamp_ns;
    }
    return PoseIngressResult::AcceptedFresh;
}

}  // namespace px4_multirotor_controller
