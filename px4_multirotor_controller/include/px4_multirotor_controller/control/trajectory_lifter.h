#pragma once

#include <ros/ros.h>

#include <cmath>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

struct TrajectoryLifterConfig {
    double planning_period{0.1};
};

inline bool positionTargetIgnored(uint16_t mask, uint16_t bit) {
    return (mask & bit) != 0U;
}

/// World-frame PV/PVA lift. τ = now − stamp. Ignored axes are not filled and
/// stay ignored in the output mask. Incoming type_mask is kept when nonzero.
inline Setpoint liftWorldLocal(const MpcTrajectoryState& sample, const ros::Time& now,
                               uint16_t default_mask, bool enable_yaw) {
    Setpoint sp;
    if (!sample.is_valid) {
        return sp;
    }

    uint16_t mask = sample.type_mask != 0U ? sample.type_mask : default_mask;
    if (!enable_yaw) {
        mask |= kIgnoreYawBit | kIgnoreYawRateBit;
    } else {
        mask &= static_cast<uint16_t>(~kIgnoreYawBit);
    }

    double tau = 0.0;
    if (!sample.planning_time.isZero() && now.isValid()) {
        tau = (now - sample.planning_time).toSec();
    }
    if (tau < 0.0) {
        tau = 0.0;
    }

    const double ax = positionTargetIgnored(mask, kIgnoreAfxBit) ? 0.0 : sample.acceleration_k.x();
    const double ay = positionTargetIgnored(mask, kIgnoreAfyBit) ? 0.0 : sample.acceleration_k.y();
    const double az = positionTargetIgnored(mask, kIgnoreAfzBit) ? 0.0 : sample.acceleration_k.z();

    if (!positionTargetIgnored(mask, kIgnorePxBit)) {
        sp.x = sample.position_k.x() + sample.velocity_k.x() * tau + 0.5 * ax * tau * tau;
    }
    if (!positionTargetIgnored(mask, kIgnorePyBit)) {
        sp.y = sample.position_k.y() + sample.velocity_k.y() * tau + 0.5 * ay * tau * tau;
    }
    if (!positionTargetIgnored(mask, kIgnorePzBit)) {
        sp.z = sample.position_k.z() + sample.velocity_k.z() * tau + 0.5 * az * tau * tau;
    }
    if (!positionTargetIgnored(mask, kIgnoreVxBit)) {
        sp.vx = sample.velocity_k.x() + ax * tau;
    }
    if (!positionTargetIgnored(mask, kIgnoreVyBit)) {
        sp.vy = sample.velocity_k.y() + ay * tau;
    }
    if (!positionTargetIgnored(mask, kIgnoreVzBit)) {
        sp.vz = sample.velocity_k.z() + az * tau;
    }
    if (!positionTargetIgnored(mask, kIgnoreAfxBit)) {
        sp.ax = ax;
    }
    if (!positionTargetIgnored(mask, kIgnoreAfyBit)) {
        sp.ay = ay;
    }
    if (!positionTargetIgnored(mask, kIgnoreAfzBit)) {
        sp.az = az;
    }

    sp.qx = sample.qx;
    sp.qy = sample.qy;
    sp.qz = sample.qz;
    sp.qw = sample.qw;
    sp.yaw_rate = sample.yaw_rate;
    sp.type_mask = mask;
    sp.coordinate_frame = sample.coordinate_frame == 0U ? 1U : sample.coordinate_frame;
    return sp;
}

inline bool passThroughReferenceReady(const MpcTrajectoryState& sample, double command_time,
                                      double now, double timeout) {
    (void)command_time;
    if (!sample.is_valid) {
        return false;
    }
    if (!sample.position_k.allFinite() || !sample.velocity_k.allFinite() ||
        !sample.acceleration_k.allFinite()) {
        return false;
    }
    const double stamp = sample.planning_time.isZero() ? now : sample.planning_time.toSec();
    if (timeout > 0.0 && now - stamp > timeout) {
        return false;
    }
    return true;
}

/// Initial-ready target is aligned to the hover pose. Ignored position axes
/// are not part of the check (SCE 3523 ignores \(p_x,p_y\)).
inline bool passThroughPlanMatchesHover(const MpcTrajectoryState& sample, double hover_x,
                                        double hover_y, double hover_z, double xy_tol,
                                        double z_tol) {
    if (!sample.is_valid) {
        return false;
    }
    const uint16_t mask = sample.type_mask;
    const bool use_x = !positionTargetIgnored(mask, kIgnorePxBit);
    const bool use_y = !positionTargetIgnored(mask, kIgnorePyBit);
    const bool use_z = !positionTargetIgnored(mask, kIgnorePzBit);
    if (!use_x && !use_y && !use_z) {
        return true;
    }
    if (use_x && use_y) {
        const double dx = sample.position_k.x() - hover_x;
        const double dy = sample.position_k.y() - hover_y;
        if ((dx * dx + dy * dy) > xy_tol * xy_tol) {
            return false;
        }
    } else if (use_x && std::abs(sample.position_k.x() - hover_x) > xy_tol) {
        return false;
    } else if (use_y && std::abs(sample.position_k.y() - hover_y) > xy_tol) {
        return false;
    }
    if (use_z && std::abs(sample.position_k.z() - hover_z) > z_tol) {
        return false;
    }
    return true;
}

/// Hover→Custom1 needs a fresh initial target near hover. After takeover the
/// plan may recede; only freshness remains.
inline bool passThroughMayTakeSetpoint(const MpcTrajectoryState& sample, double command_time,
                                       double now, double timeout, double hover_x, double hover_y,
                                       double hover_z, double xy_tol, double z_tol,
                                       bool tracking_armed) {
    if (!passThroughReferenceReady(sample, command_time, now, timeout)) {
        return false;
    }
    if (tracking_armed) {
        return true;
    }
    return passThroughPlanMatchesHover(sample, hover_x, hover_y, hover_z, xy_tol, z_tol);
}

class TrajectoryLifter {
   public:
    explicit inline TrajectoryLifter(
        const TrajectoryLifterConfig& config = TrajectoryLifterConfig{})
        : config_(config) {}

    inline Setpoint lift(const MpcTrajectoryState& mpc_traj, const ros::Time& current_time,
                         uint16_t default_mask = kDefaultPvaLocalTypeMask,
                         bool enable_yaw = false) const {
        return liftWorldLocal(mpc_traj, current_time, default_mask, enable_yaw);
    }

    inline const TrajectoryLifterConfig& config() const {
        return config_;
    }

   private:
    TrajectoryLifterConfig config_;
};

}  // namespace px4_multirotor_controller
