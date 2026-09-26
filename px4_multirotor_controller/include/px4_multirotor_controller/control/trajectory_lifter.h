#pragma once


#include <cmath>
#include <cstdint>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/common/time.h"
#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"
#include "px4_multirotor_controller/uav/mpc_trajectory_buffer.h"

namespace px4_multirotor_controller {

struct TrajectoryLifterConfig {
    double planning_period{0.1};
};

inline bool positionTargetIgnored(uint16_t mask, uint16_t bit) {
    return (mask & bit) != 0U;
}

/// World-frame PV/PVA lift. The current ingress uses local receipt time as
/// planning_time, not the planner's effective time. This is a segment lift,
/// not a cross-plan continuity, freshness, or delay-compensation guarantee.
/// Ignored derivatives do not contribute to active fields; ignored output
/// axes stay ignored. Incoming type_mask is kept when nonzero.
inline Setpoint liftWorldLocal(const MpcTrajectoryState& sample, const Time& now,
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
    if (!sample.planning_time.isZero() && !now.isZero()) {
        tau = (now - sample.planning_time).toSec();
    }
    if (tau < 0.0) {
        tau = 0.0;
    }

    const double vx = positionTargetIgnored(mask, kIgnoreVxBit) ? 0.0 : sample.velocity_k.x();
    const double vy = positionTargetIgnored(mask, kIgnoreVyBit) ? 0.0 : sample.velocity_k.y();
    const double vz = positionTargetIgnored(mask, kIgnoreVzBit) ? 0.0 : sample.velocity_k.z();
    const double ax = positionTargetIgnored(mask, kIgnoreAfxBit) ? 0.0 : sample.acceleration_k.x();
    const double ay = positionTargetIgnored(mask, kIgnoreAfyBit) ? 0.0 : sample.acceleration_k.y();
    const double az = positionTargetIgnored(mask, kIgnoreAfzBit) ? 0.0 : sample.acceleration_k.z();

    if (!positionTargetIgnored(mask, kIgnorePxBit)) {
        sp.x = sample.position_k.x() + vx * tau + 0.5 * ax * tau * tau;
    }
    if (!positionTargetIgnored(mask, kIgnorePyBit)) {
        sp.y = sample.position_k.y() + vy * tau + 0.5 * ay * tau * tau;
    }
    if (!positionTargetIgnored(mask, kIgnorePzBit)) {
        sp.z = sample.position_k.z() + vz * tau + 0.5 * az * tau * tau;
    }
    if (!positionTargetIgnored(mask, kIgnoreVxBit)) {
        sp.vx = vx + ax * tau;
    }
    if (!positionTargetIgnored(mask, kIgnoreVyBit)) {
        sp.vy = vy + ay * tau;
    }
    if (!positionTargetIgnored(mask, kIgnoreVzBit)) {
        sp.vz = vz + az * tau;
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

inline bool passThroughReferenceReady(const MpcTrajectoryState& sample) {
    if (!sample.is_valid) {
        return false;
    }
    if (!sample.position_k.allFinite() || !sample.velocity_k.allFinite() ||
        !sample.acceleration_k.allFinite()) {
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

/// Hover→Custom1 needs an initial target near hover. After takeover the plan
/// may recede; reference timestamps never gate tracking.
inline bool passThroughMayTakeSetpoint(const MpcTrajectoryState& sample, double hover_x,
                                       double hover_y, double hover_z, double xy_tol, double z_tol,
                                       bool tracking_armed) {
    if (!passThroughReferenceReady(sample)) {
        return false;
    }
    if (tracking_armed) {
        return true;
    }
    return passThroughPlanMatchesHover(sample, hover_x, hover_y, hover_z, xy_tol, z_tol);
}

// Absolute world frame used by this stack (MAVROS FRAME_LOCAL_NED = 1, ENU after
// the MAVROS conversion). Frame 0 is the historical "unset, treat as local" input.
inline bool smcCoordinateFrameIsWorld(uint8_t coordinate_frame) {
    const uint8_t frame = coordinate_frame == 0U ? 1U : coordinate_frame;
    return frame == 1U;
}

// SMC needs every world position, velocity and acceleration component.
// An ignored axis is unavailable; it is not the number zero. FORCE means the
// acceleration field is a force. Yaw bits are irrelevant because SMC yaw is off.
inline bool smcMaskSuppliesWorldPva(uint16_t type_mask) {
    constexpr uint16_t kPvaIgnoreBits = kIgnorePxBit | kIgnorePyBit | kIgnorePzBit | kIgnoreVxBit |
                                        kIgnoreVyBit | kIgnoreVzBit | kIgnoreAfxBit |
                                        kIgnoreAfyBit | kIgnoreAfzBit;
    return (type_mask & kForceBit) == 0U && (type_mask & kPvaIgnoreBits) == 0U;
}

// Nonzero wire masks are the message itself. A zero mask selects local_type_mask,
// the same substitution liftWorldLocal already makes.
inline uint16_t effectivePositionTargetMask(uint16_t type_mask, uint16_t default_mask) {
    return type_mask != 0U ? type_mask : default_mask;
}

inline bool smcWorldPvaAvailable(uint16_t type_mask, uint8_t coordinate_frame,
                                 uint16_t default_mask) {
    return smcCoordinateFrameIsWorld(coordinate_frame) &&
           smcMaskSuppliesWorldPva(effectivePositionTargetMask(type_mask, default_mask));
}

// Header effective time for SMC and for PX4_LOCAL zero-order hold.
// Legacy PX4_LOCAL keeps the arrival stamp.
inline bool usesStageEffectiveTime(TrackingBackend backend, Px4LocalLiftMode lift_mode) {
    return backend == TrackingBackend::SMC ||
           (backend == TrackingBackend::PX4_LOCAL &&
            lift_mode == Px4LocalLiftMode::ZeroOrderHold);
}

// One ingress for the ROS producer and ctl-px4.
struct PositionTargetIngress {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double yaw_rate{0.0};
    uint16_t type_mask{0};
    uint8_t coordinate_frame{1};
    Time header_stamp{};
    Time receipt_time{};
};

inline MpcTrajectoryState ingestPositionTarget(const PositionTargetIngress& in,
                                               TrackingBackend backend,
                                               Px4LocalLiftMode lift_mode) {
    MpcTrajectoryState traj;
    traj.position_k = in.position;
    traj.velocity_k = in.velocity;
    traj.acceleration_k = in.acceleration;
    const Eigen::Quaterniond yaw_quat = yawToQuaternion(in.yaw);
    traj.qx = yaw_quat.x();
    traj.qy = yaw_quat.y();
    traj.qz = yaw_quat.z();
    traj.qw = yaw_quat.w();
    traj.yaw_rate = in.yaw_rate;
    traj.type_mask = in.type_mask;
    traj.coordinate_frame = in.coordinate_frame == 0U ? 1U : in.coordinate_frame;
    traj.new_data_received = false;
    if (usesStageEffectiveTime(backend, lift_mode)) {
        const bool finite = in.position.allFinite() && in.velocity.allFinite() &&
                            in.acceleration.allFinite() && std::isfinite(in.yaw) &&
                            std::isfinite(in.yaw_rate);
        traj.planning_time = in.header_stamp;
        traj.is_valid = finite && !in.header_stamp.isZero();
        if (backend == TrackingBackend::SMC) {
            // A zero wire mask still has to pass liftForBackend against local_type_mask.
            // Any explicit mask or frame that is not world PVA is rejected here so the
            // producer does not replace a live segment with an unusable one.
            const bool wire_is_world_pva = smcCoordinateFrameIsWorld(in.coordinate_frame) &&
                                           (in.type_mask == 0U || smcMaskSuppliesWorldPva(in.type_mask));
            traj.is_valid = traj.is_valid && wire_is_world_pva;
        }
        return traj;
    }
    traj.planning_time = in.receipt_time;
    traj.is_valid = true;
    return traj;
}

// Legacy promotes on arrival. SMC and zero-order hold wait until the header.
inline bool promoteTrajectorySample(MpcTrajectoryBuffer& buffer, const Time& now,
                                    TrackingBackend backend, Px4LocalLiftMode lift_mode) {
    if (!buffer.hasPending()) {
        return false;
    }
    if (usesStageEffectiveTime(backend, lift_mode)) {
        const MpcTrajectoryState& pending = buffer.pending();
        if (!pending.is_valid || pending.planning_time.isZero() || now < pending.planning_time) {
            return false;
        }
    }
    return buffer.promotePending(buffer.pending().planning_time);
}

struct EffectiveSegmentLift {
    bool success{false};
    Setpoint setpoint{};
};

inline int64_t segmentPeriodNs(double planning_period) {
    if (!std::isfinite(planning_period) || planning_period <= 0.0) {
        return -1;
    }
    return std::llround(planning_period * 1e9);
}

// True on [header, header + planning_period]. Shared by SMC and zero-order hold.
inline bool stageWindowOpen(const MpcTrajectoryState& sample, const Time& now,
                            double planning_period) {
    if (!passThroughReferenceReady(sample) || sample.planning_time.isZero() || now.isZero() ||
        now < sample.planning_time) {
        return false;
    }
    const int64_t period_ns = segmentPeriodNs(planning_period);
    const int64_t tau_ns = (now - sample.planning_time).toNSec();
    return period_ns >= 0 && tau_ns >= 0 && tau_ns <= period_ns;
}

// Legacy PX4_LOCAL is liftWorldLocal from the receipt stamp, with no window.
// SMC integrates inside the window and fails outside it. Zero-order hold uses
// the same window and the same segment, and outputs the stage-1 sample.
inline EffectiveSegmentLift liftForBackend(const MpcTrajectoryState& sample, const Time& now,
                                           double planning_period, uint16_t default_mask,
                                           bool enable_yaw, TrackingBackend backend,
                                           Px4LocalLiftMode px4_local_lift) {
    EffectiveSegmentLift out;
    if (backend == TrackingBackend::SMC) {
        if (!stageWindowOpen(sample, now, planning_period) ||
            !smcWorldPvaAvailable(sample.type_mask, sample.coordinate_frame, default_mask)) {
            return out;
        }
        out.setpoint = liftWorldLocal(sample, now, default_mask, false);
        out.success = true;
        return out;
    }
    if (px4_local_lift == Px4LocalLiftMode::ZeroOrderHold) {
        if (!stageWindowOpen(sample, now, planning_period)) {
            return out;
        }
        out.setpoint = liftWorldLocal(sample, sample.planning_time, default_mask, enable_yaw);
        out.success = true;
        return out;
    }
    if (!sample.is_valid) {
        return out;
    }
    out.setpoint = liftWorldLocal(sample, now, default_mask, enable_yaw);
    out.success = true;
    return out;
}

class TrajectoryLifter {
   public:
    explicit inline TrajectoryLifter(
        const TrajectoryLifterConfig& config = TrajectoryLifterConfig{})
        : config_(config) {}

    inline Setpoint lift(const MpcTrajectoryState& mpc_traj, const Time& current_time,
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
