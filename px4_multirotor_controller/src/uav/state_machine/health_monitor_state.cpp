#include "px4_multirotor_controller/uav/state_machine/health_monitor_state.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "px4_multirotor_controller/common/controller_clock.h"
#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {
namespace {

constexpr uint64_t kClockBackstepResetNs = 5000000000ULL;
constexpr uint64_t kClockForwardResetNs = 60000000000ULL;
constexpr double kMaxRealTimeForClockResetS = 5.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxMocapAngularVelRadPerSec = 250.0 * kPi / 180.0;
constexpr double kMocapAttitudeMarginRad = 25.0 * kPi / 180.0;
constexpr double kErrorLogMinIntervalS = 1.0;

void collectNewSamples(const SensorData::PoseRing& ring, uint64_t& last_seq,
                       std::vector<SensorData::PoseSample>& out) {
    out.clear();
    if (ring.count == 0) {
        return;
    }
    const size_t cap = SensorData::PoseRing::kCapacity;
    const size_t start = (ring.head + cap - ring.count) % cap;
    for (size_t i = 0; i < ring.count; ++i) {
        const auto& sample = ring.samples[(start + i) % cap];
        if (sample.seq > last_seq) {
            out.push_back(sample);
        }
    }
    if (!out.empty()) {
        last_seq = out.back().seq;
    }
}

double worldBoundsOvershoot(const WorldControlBounds& bounds, double x, double y, double z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return std::numeric_limits<double>::infinity();
    }
    double max_over = 0.0;
    if (x < bounds.x_min) max_over = std::max(max_over, bounds.x_min - x);
    if (x > bounds.x_max) max_over = std::max(max_over, x - bounds.x_max);
    if (y < bounds.y_min) max_over = std::max(max_over, bounds.y_min - y);
    if (y > bounds.y_max) max_over = std::max(max_over, y - bounds.y_max);
    if (z < bounds.z_min) max_over = std::max(max_over, bounds.z_min - z);
    if (z > bounds.z_max) max_over = std::max(max_over, z - bounds.z_max);
    return max_over;
}

}  // namespace

HealthMonitorState::HealthMonitorState(DroneController& controller) : controller_(controller) {}

bool HealthMonitorState::isPoseTimedOut() const {
    const auto& sd = controller_.getSensorData();
    const auto& cfg = controller_.getConfig().safety;
    const double now = controller_.getCurrentTime();
    if (sd.last_valid_local_rx_s && (now - *sd.last_valid_local_rx_s) >= cfg.pose_stale_timeout_s) {
        return true;
    }
    if (sd.last_valid_vrpn_rx_s && (now - *sd.last_valid_vrpn_rx_s) >= cfg.pose_stale_timeout_s) {
        return true;
    }
    return false;
}

bool HealthMonitorState::isLocalPositionFaulted() const {
    const auto& sd = controller_.getSensorData();
    const auto& cfg = controller_.getConfig().safety;
    const double now = controller_.getCurrentTime();
    if (safety_state_.local_pose_faulted || safety_state_.local_pose_timed_out ||
        sd.local_pose_invalid || !sd.local_pos_stats.is_active) {
        return true;
    }
    if (sd.last_valid_local_rx_s && (now - *sd.last_valid_local_rx_s) >= cfg.pose_stale_timeout_s) {
        return true;
    }
    return false;
}

void HealthMonitorState::resetSafetyLatches() {
    const auto& sd = controller_.getSensorData();
    safety_state_.geofence_violated = false;
    safety_state_.mocap_gross_bounds_violated = false;
    safety_state_.local_gross_bounds_violated = false;
    safety_state_.local_pose_faulted = false;
    safety_state_.velocity_xy_exceeded = false;
    safety_state_.velocity_z_exceeded = false;
    safety_state_.was_vrpn_pose_invalid = sd.vrpn_pose_invalid;
    safety_state_.was_local_pose_invalid = sd.local_pose_invalid;
    safety_state_.last_seen_vrpn_invalid_count = sd.vrpn_pose_invalid_count;
    safety_state_.last_seen_local_invalid_count = sd.local_pose_invalid_count;
    last_error_event_id_ = 0;
    last_error_log_s_ = -1.0e9;
    last_safety_trigger_ = SafetyTriggerEvidence{};
}

std::optional<double> HealthMonitorState::nextWake() const {
    const auto& sd = controller_.getSensorData();
    const auto& cfg = controller_.getConfig().safety;
    std::optional<double> due;
    const auto include = [&](std::optional<double> candidate) {
        if (!candidate) {
            return;
        }
        if (!due || *candidate < *due) {
            due = *candidate;
        }
    };
    if (!safety_state_.state_estimate_unusable &&
        safety_state_.state_estimate_unusable_since >= 0.0) {
        include(controllerDueAt(safety_state_.state_estimate_unusable_since,
                                cfg.state_estimate_unusable_trip_delay));
    }
    if (sd.last_valid_local_rx_s && safety_state_.was_local_pos_active &&
        !safety_state_.local_pose_timed_out) {
        include(controllerDueAt(*sd.last_valid_local_rx_s, cfg.pose_stale_timeout_s));
    }
    if (sd.last_valid_vrpn_rx_s && safety_state_.was_vrpn_pose_active &&
        !safety_state_.vrpn_pose_timed_out) {
        include(controllerDueAt(*sd.last_valid_vrpn_rx_s, cfg.pose_stale_timeout_s));
    }
    return due;
}

void HealthMonitorState::emitSafetySignal(FlightSignal signal, const char* reason, double measured,
                                          double threshold, double dt_s, bool mark_local_fault) {
    const uint32_t event_id = static_cast<uint32_t>(signal);
    const double now = controller_.getCurrentTime();
    if (mark_local_fault) {
        safety_state_.local_pose_faulted = true;
    }

    const bool same_recent_landing_event =
        (controller_.flightState() == FlightState::Landing) &&
        (last_error_event_id_ == event_id) &&
        ((now - last_error_log_s_) < kErrorLogMinIntervalS);
    if (same_recent_landing_event) {
        return;
    }

    const bool first_or_changed_error = !last_safety_trigger_.triggered ||
                                        last_error_event_id_ != event_id ||
                                        ((now - last_error_log_s_) >= kErrorLogMinIntervalS);
    last_safety_trigger_.triggered = true;
    last_safety_trigger_.event_id = event_id;
    last_safety_trigger_.signal = signal;
    last_safety_trigger_.reason = reason ? reason : "unknown";
    last_safety_trigger_.measured = measured;
    last_safety_trigger_.threshold = threshold;
    last_safety_trigger_.dt_s = dt_s;
    last_safety_trigger_.timestamp_s = now;

    if (first_or_changed_error) {
        controller_.logError(
            "[HealthMonitor] safety_trigger reason=%s event_id=%u measured=%.4f "
            "threshold=%.4f dt_s=%.4f",
            last_safety_trigger_.reason.c_str(), event_id, measured, threshold, dt_s);
        last_error_event_id_ = event_id;
        last_error_log_s_ = now;
    }
    controller_.emitFlightSignal(signal);
}

bool HealthMonitorState::evaluatePoseSampleSafety(
    const SensorData::PoseSample& sample, PoseStreamTracker& tracker, bool& gross_violated_latch,
    FlightSignal gross_signal, const char* gross_reason, FlightSignal jump_signal,
    const char* pos_jump_reason, const char* att_jump_reason, bool clear_pairing_on_reset,
    bool mark_local_fault) {
    const auto& cfg = controller_.getConfig().safety;

    if (cfg.mocap_gross_bounds.outside(sample.x, sample.y, sample.z)) {
        const double violation =
            worldBoundsOvershoot(cfg.mocap_gross_bounds, sample.x, sample.y, sample.z);
        if (!gross_violated_latch) {
            emitSafetySignal(gross_signal, gross_reason, violation, 0.0, 0.0, mark_local_fault);
        }
        gross_violated_latch = true;
    } else {
        gross_violated_latch = false;
    }

    auto evaluate_jump = [&](double dt_s) {
        if (dt_s <= 0.0 || dt_s > cfg.pose_stale_timeout_s) {
            return;
        }
        const auto& prev = *tracker.prev_valid;
        const double max_axis = std::max(
            {std::abs(sample.x - prev.x), std::abs(sample.y - prev.y), std::abs(sample.z - prev.z)});
        const double pos_limit = 1.0 + 6.0 * dt_s;
        if (max_axis > pos_limit) {
            emitSafetySignal(jump_signal, pos_jump_reason, max_axis, pos_limit, dt_s,
                             mark_local_fault);
            return;
        }
        const double norm1 = std::sqrt(prev.qx * prev.qx + prev.qy * prev.qy + prev.qz * prev.qz +
                                       prev.qw * prev.qw);
        const double norm2 =
            std::sqrt(sample.qx * sample.qx + sample.qy * sample.qy + sample.qz * sample.qz +
                      sample.qw * sample.qw);
        if (norm1 > 1.0e-12 && norm2 > 1.0e-12) {
            const double dot = (prev.qx * sample.qx + prev.qy * sample.qy + prev.qz * sample.qz +
                                prev.qw * sample.qw) /
                               (norm1 * norm2);
            const double dtheta = 2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
            const double att_limit = std::min(
                kPi, std::max(kPi / 3.0,
                              kMocapAttitudeMarginRad + kMaxMocapAngularVelRadPerSec * dt_s));
            if (dtheta > att_limit) {
                emitSafetySignal(jump_signal, att_jump_reason, dtheta, att_limit, dt_s,
                                 mark_local_fault);
            }
        }
    };

    bool stamp_regressed = false;
    if (!tracker.prev_valid) {
        tracker.prev_valid = sample;
    } else if (tracker.prev_valid->has_header_stamp != sample.has_header_stamp) {
        // Clock domain mismatch (stamped vs unstamped): never mix stamp_s and rx_s.
        ++tracker.stamp_anomaly_count;
        tracker.prev_valid = sample;
        if (clear_pairing_on_reset) {
            pending_vrpn_.clear();
            tracker.last_pairing_stamp_ns.reset();
            position_distance_ = PositionDistance{};
        } else {
            local_history_.clear();
            pending_vrpn_.clear();
            position_distance_ = PositionDistance{};
        }
    } else if (sample.has_header_stamp) {
        const auto& prev = *tracker.prev_valid;
        if (sample.stamp_ns <= prev.stamp_ns) {
            const uint64_t backstep_ns = prev.stamp_ns - sample.stamp_ns;
            if (backstep_ns > kClockBackstepResetNs && (sample.rx_s - prev.rx_s) >= 0.0) {
                tracker.prev_valid = sample;
                if (clear_pairing_on_reset) {
                    pending_vrpn_.clear();
                    tracker.last_pairing_stamp_ns.reset();
                    position_distance_ = PositionDistance{};
                } else {
                    local_history_.clear();
                    pending_vrpn_.clear();
                    position_distance_ = PositionDistance{};
                }
            } else {
                ++tracker.stamp_anomaly_count;
                stamp_regressed = true;
            }
        } else {
            const uint64_t forward_ns = sample.stamp_ns - prev.stamp_ns;
            const double dt = static_cast<double>(forward_ns) * 1e-9;
            if (forward_ns > kClockForwardResetNs &&
                (sample.rx_s - prev.rx_s) < kMaxRealTimeForClockResetS) {
                tracker.prev_valid = sample;
                if (clear_pairing_on_reset) {
                    pending_vrpn_.clear();
                    tracker.last_pairing_stamp_ns.reset();
                    position_distance_ = PositionDistance{};
                } else {
                    local_history_.clear();
                    pending_vrpn_.clear();
                    position_distance_ = PositionDistance{};
                }
            } else {
                evaluate_jump(dt);
                tracker.prev_valid = sample;
            }
        }
    } else {
        const double dt = sample.rx_s - tracker.prev_valid->rx_s;
        if (dt <= 0.0) {
            ++tracker.stamp_anomaly_count;
        } else {
            evaluate_jump(dt);
            tracker.prev_valid = sample;
        }
    }
    return stamp_regressed;
}

void HealthMonitorState::onTick() {
    const auto& sd = controller_.getSensorData();
    const auto& controller_config = controller_.getConfig();
    const auto& cfg = controller_config.safety;
    auto& ss = safety_state_;
    const double now = controller_.getCurrentTime();

    std::vector<SensorData::PoseSample> new_local;
    collectNewSamples(sd.local_pose_ring, last_processed_local_seq_, new_local);
    for (const auto& s : new_local) {
        const bool regressed = evaluatePoseSampleSafety(
            s, local_tracker_, ss.local_gross_bounds_violated, FlightSignal::LocalGrossBounds,
            "local_gross_bounds", FlightSignal::LocalJump, "local_position_jump",
            "local_attitude_jump", false, true);
        if (!s.has_header_stamp || regressed) {
            if (!s.has_header_stamp) {
                ++local_tracker_.stamp_anomaly_count;
            }
            continue;
        }
        if (!local_history_.empty() && s.stamp_ns <= local_history_.back().stamp_ns) {
            local_history_.clear();
            pending_vrpn_.clear();
            position_distance_ = PositionDistance{};
        }
        local_history_.push_back(s);
        while (local_history_.size() > SensorData::PoseRing::kCapacity) {
            local_history_.pop_front();
        }
    }

    std::vector<SensorData::PoseSample> new_vrpn;
    collectNewSamples(sd.vrpn_pose_ring, last_processed_vrpn_seq_, new_vrpn);
    for (const auto& v : new_vrpn) {
        const bool regressed = evaluatePoseSampleSafety(
            v, vrpn_tracker_, ss.mocap_gross_bounds_violated, FlightSignal::MocapGrossBounds,
            "mocap_gross_bounds", FlightSignal::VrpnJump, "vrpn_position_jump",
            "vrpn_attitude_jump", true, false);
        if (!v.has_header_stamp) {
            position_distance_.metres = std::numeric_limits<double>::quiet_NaN();
            position_distance_.available = false;
            position_distance_.exceeded = false;
            continue;
        }
        if (regressed ||
            (vrpn_tracker_.last_pairing_stamp_ns &&
             v.stamp_ns <= *vrpn_tracker_.last_pairing_stamp_ns)) {
            position_distance_.metres = std::numeric_limits<double>::quiet_NaN();
            position_distance_.available = false;
            position_distance_.exceeded = false;
            continue;
        }
        vrpn_tracker_.last_pairing_stamp_ns = v.stamp_ns;
        const int64_t delay_ns =
            static_cast<int64_t>(std::llround(std::max(0.0, cfg.localization_delay_s) * 1e9));
        const int64_t t_target_ns = static_cast<int64_t>(v.stamp_ns) - delay_ns;
        pending_vrpn_.push_back(PendingVrpnSample{v, t_target_ns});
        while (pending_vrpn_.size() > SensorData::PoseRing::kCapacity) {
            pending_vrpn_.pop_front();
        }
    }

    const int64_t max_gap_ns =
        static_cast<int64_t>(std::llround(std::max(0.0, cfg.max_pairing_gap_s) * 1e9));
    bool aligned_error_tripped = false;
    double aligned_error_measured = 0.0;

    while (!pending_vrpn_.empty()) {
        const auto& item = pending_vrpn_.front();
        if (item.t_target_ns < 0) {
            position_distance_.metres = std::numeric_limits<double>::quiet_NaN();
            position_distance_.available = false;
            position_distance_.exceeded = false;
            pending_vrpn_.pop_front();
            continue;
        }
        if (local_history_.empty()) {
            if ((now - item.sample.rx_s) > cfg.pose_stale_timeout_s || pending_vrpn_.size() > 32) {
                position_distance_.metres = std::numeric_limits<double>::quiet_NaN();
                position_distance_.available = false;
                position_distance_.exceeded = false;
                pending_vrpn_.pop_front();
                continue;
            }
            break;
        }
        const int64_t oldest_ns = static_cast<int64_t>(local_history_.front().stamp_ns);
        const int64_t newest_ns = static_cast<int64_t>(local_history_.back().stamp_ns);
        if (item.t_target_ns < oldest_ns) {
            position_distance_.metres = std::numeric_limits<double>::quiet_NaN();
            position_distance_.available = false;
            position_distance_.exceeded = false;
            pending_vrpn_.pop_front();
            continue;
        }
        if (item.t_target_ns > newest_ns) {
            if ((now - item.sample.rx_s) > cfg.pose_stale_timeout_s || pending_vrpn_.size() > 32) {
                position_distance_.metres = std::numeric_limits<double>::quiet_NaN();
                position_distance_.available = false;
                position_distance_.exceeded = false;
                pending_vrpn_.pop_front();
                continue;
            }
            break;
        }
        size_t idx = 0;
        while (idx + 1 < local_history_.size() &&
               static_cast<int64_t>(local_history_[idx + 1].stamp_ns) <= item.t_target_ns) {
            ++idx;
        }
        const auto& left = local_history_[idx];
        double lx = left.x;
        double ly = left.y;
        double lz = left.z;
        bool paired = false;
        if (static_cast<int64_t>(left.stamp_ns) == item.t_target_ns) {
            paired = true;
        } else if (idx + 1 < local_history_.size()) {
            const auto& right = local_history_[idx + 1];
            const int64_t gap_ns =
                static_cast<int64_t>(right.stamp_ns) - static_cast<int64_t>(left.stamp_ns);
            if (gap_ns > 0 && gap_ns <= max_gap_ns) {
                const double alpha =
                    static_cast<double>(item.t_target_ns - static_cast<int64_t>(left.stamp_ns)) /
                    static_cast<double>(gap_ns);
                lx = left.x + alpha * (right.x - left.x);
                ly = left.y + alpha * (right.y - left.y);
                lz = left.z + alpha * (right.z - left.z);
                paired = true;
            }
        }
        if (!paired) {
            position_distance_.metres = std::numeric_limits<double>::quiet_NaN();
            position_distance_.available = false;
            position_distance_.exceeded = false;
        } else {
            const double err =
                std::hypot(item.sample.x - lx, item.sample.y - ly, item.sample.z - lz);
            position_distance_.metres = err;
            position_distance_.available = std::isfinite(err);
            position_distance_.exceeded =
                position_distance_.available && err > cfg.max_aligned_position_error_m;
            if (position_distance_.exceeded) {
                aligned_error_tripped = true;
                aligned_error_measured = std::max(aligned_error_measured, err);
            }
        }
        pending_vrpn_.pop_front();
    }
    if (aligned_error_tripped) {
        emitSafetySignal(FlightSignal::AlignedPoseError, "aligned_pose_error",
                         aligned_error_measured, cfg.max_aligned_position_error_m, 0.0);
    }

    // Immediate 1-sample invalid pose arrival checks (captures unconsumed invalid
    // increments even when a subsequent valid frame coalesced into the same turn).
    if (sd.vrpn_pose_invalid_count > ss.last_seen_vrpn_invalid_count ||
        (sd.vrpn_pose_invalid && !ss.was_vrpn_pose_invalid)) {
        ss.last_seen_vrpn_invalid_count = sd.vrpn_pose_invalid_count;
        ss.was_vrpn_pose_invalid = sd.vrpn_pose_invalid;
        emitSafetySignal(
            FlightSignal::VrpnPoseInvalid, "vrpn_pose_invalid",
            static_cast<double>(std::max<uint64_t>(1U, sd.vrpn_pose_invalid_count)), 1.0, 0.0);
    } else if (!sd.vrpn_pose_invalid) {
        ss.was_vrpn_pose_invalid = false;
    }

    if (sd.local_pose_invalid_count > ss.last_seen_local_invalid_count ||
        (sd.local_pose_invalid && !ss.was_local_pose_invalid)) {
        ss.last_seen_local_invalid_count = sd.local_pose_invalid_count;
        ss.was_local_pose_invalid = sd.local_pose_invalid;
        emitSafetySignal(
            FlightSignal::LocalPoseInvalid, "local_pose_invalid",
            static_cast<double>(std::max<uint64_t>(1U, sd.local_pose_invalid_count)), 1.0, 0.0,
            true);
    } else if (!sd.local_pose_invalid) {
        ss.was_local_pose_invalid = false;
    }

    // Pose stream staleness checks across all backends (T_stale configurable, default 2.0 s).
    const double local_age = sd.last_valid_local_rx_s
                                 ? std::max(0.0, now - *sd.last_valid_local_rx_s)
                                 : sd.local_pos_stats.time_since_last_msg;
    const bool local_timed_out_by_rx =
        sd.last_valid_local_rx_s && (now - *sd.last_valid_local_rx_s) >= cfg.pose_stale_timeout_s;
    const bool local_currently_ok = sd.local_pos_stats.is_active && !local_timed_out_by_rx;
    if (ss.was_local_pos_active && !local_currently_ok) {
        emitSafetySignal(FlightSignal::TimeoutLocalPosition, "timeout_local_pose", local_age,
                         cfg.pose_stale_timeout_s, local_age, true);
    }
    ss.was_local_pos_active = local_currently_ok;
    ss.local_pose_timed_out = !local_currently_ok;

    const double vrpn_age = sd.last_valid_vrpn_rx_s
                                ? std::max(0.0, now - *sd.last_valid_vrpn_rx_s)
                                : sd.vrpn_pose_stats.time_since_last_msg;
    const bool vrpn_timed_out_by_rx =
        sd.last_valid_vrpn_rx_s && (now - *sd.last_valid_vrpn_rx_s) >= cfg.pose_stale_timeout_s;
    const bool vrpn_currently_ok = sd.vrpn_pose_stats.is_active && !vrpn_timed_out_by_rx;
    if (ss.was_vrpn_pose_active && !vrpn_currently_ok) {
        emitSafetySignal(FlightSignal::TimeoutVrpnPose, "timeout_vrpn_pose", vrpn_age,
                         cfg.pose_stale_timeout_s, vrpn_age);
    }
    ss.was_vrpn_pose_active = vrpn_currently_ok;
    ss.vrpn_pose_timed_out = !vrpn_currently_ok;

    const bool fused = trackingUsesFusedEstimate(controller_config.tracking_backend);
    if (fused && sd.feedback_source == FeedbackSource::RIGID_ESTIMATE) {
        checkSensorActiveEdge(sd.uav_state_estimate_stats, ss.was_uav_state_estimate_active,
                              FlightSignal::TimeoutEstimate, "timeout_state_estimate", 2.5);
    }
    if (fused && sd.feedback_source == FeedbackSource::MAVROS_LOCAL) {
        checkSensorActiveEdge(sd.local_velocity_stats, ss.was_local_velocity_active,
                              FlightSignal::TimeoutLocalVelocity, "timeout_local_velocity", 2.5);
        checkSensorActiveEdge(sd.imu_stats, ss.was_imu_active, FlightSignal::TimeoutImu,
                              "timeout_imu", 2.5);
    }
    checkSensorActiveEdge(sd.state_stats, ss.was_state_active, FlightSignal::TimeoutFcuState,
                          "timeout_fcu_state", 2.5);

    const bool control_state_unusable = fused && sensor_checks::isControlStateActive(sd) &&
                                        !sensor_checks::isControlStateUsableForControl(sd);
    if (control_state_unusable && ss.state_estimate_unusable_since < 0.0) {
        ss.state_estimate_unusable_since = now;
    } else if (!control_state_unusable) {
        ss.state_estimate_unusable_since = -1.0;
    }

    const double unusable_duration =
        ss.state_estimate_unusable_since >= 0.0 ? now - ss.state_estimate_unusable_since : 0.0;
    const bool estimate_unusable_trip =
        control_state_unusable && unusable_duration >= cfg.state_estimate_unusable_trip_delay;
    if (!ss.state_estimate_unusable && estimate_unusable_trip) {
        emitSafetySignal(FlightSignal::EstimateUnusable, "state_estimate_unusable",
                         unusable_duration, cfg.state_estimate_unusable_trip_delay,
                         unusable_duration);
    }
    ss.state_estimate_unusable = estimate_unusable_trip;

    if (sensor_checks::isWorldPoseNew(sd, controller_config.tracking_backend)) {
        const double x = sensor_checks::worldX(sd, controller_config.tracking_backend);
        const double y = sensor_checks::worldY(sd, controller_config.tracking_backend);
        const double z = sensor_checks::worldZ(sd, controller_config.tracking_backend);
        const bool currently_violated = cfg.world_boundary && cfg.world_boundary->control_bounds &&
                                        cfg.world_boundary->control_bounds->outside(x, y, z);
        if (!ss.geofence_violated && currently_violated) {
            const double overshoot =
                worldBoundsOvershoot(*cfg.world_boundary->control_bounds, x, y, z);
            emitSafetySignal(FlightSignal::Geofence, "control_bounds_geofence", overshoot, 0.0,
                             0.0);
        }
        ss.geofence_violated = currently_violated;
    }

    const bool world_vel_new =
        sensor_checks::isWorldPoseNew(sd, controller_config.tracking_backend) ||
        ((sd.feedback_source == FeedbackSource::MAVROS_LOCAL ||
          !trackingUsesFusedEstimate(controller_config.tracking_backend)) &&
         sd.local_velocity_stats.is_new);
    if (world_vel_new) {
        const double vx = sensor_checks::worldVx(sd, controller_config.tracking_backend);
        const double vy = sensor_checks::worldVy(sd, controller_config.tracking_backend);
        const double vz = sensor_checks::worldVz(sd, controller_config.tracking_backend);

        bool xy_exceeded = false;
        double xy_measured = 0.0;
        double xy_threshold = cfg.max_velocity_xy;
        bool z_exceeded = false;
        double z_measured = std::abs(vz);
        double z_threshold = cfg.max_velocity_z;

        if (cfg.world_boundary && cfg.world_boundary->control_velocity_bounds) {
            const auto& vb = *cfg.world_boundary->control_velocity_bounds;
            const bool vx_bad = !std::isfinite(vx) || vx < vb.vx_min || vx > vb.vx_max;
            const bool vy_bad = !std::isfinite(vy) || vy < vb.vy_min || vy > vb.vy_max;
            xy_exceeded = vx_bad || vy_bad;
            if (vx_bad) {
                xy_measured = vx;
                xy_threshold = (!std::isfinite(vx) || vx > vb.vx_max) ? vb.vx_max : vb.vx_min;
            } else if (vy_bad) {
                xy_measured = vy;
                xy_threshold = (!std::isfinite(vy) || vy > vb.vy_max) ? vb.vy_max : vb.vy_min;
            } else {
                xy_measured = std::max(std::abs(vx), std::abs(vy));
                xy_threshold = std::max({std::abs(vb.vx_min), std::abs(vb.vx_max),
                                         std::abs(vb.vy_min), std::abs(vb.vy_max)});
            }

            z_exceeded = !std::isfinite(vz) || vz < vb.vz_min || vz > vb.vz_max;
            z_measured = vz;
            z_threshold = (!std::isfinite(vz) || vz > vb.vz_max) ? vb.vz_max : vb.vz_min;
        } else if (!cfg.require_control_velocity_bounds) {
            const double velocity_xy = std::sqrt(vx * vx + vy * vy);
            xy_measured = velocity_xy;
            xy_threshold = cfg.max_velocity_xy;
            xy_exceeded = !std::isfinite(velocity_xy) || velocity_xy > cfg.max_velocity_xy;

            z_measured = std::abs(vz);
            z_threshold = cfg.max_velocity_z;
            z_exceeded = !std::isfinite(vz) || z_measured > cfg.max_velocity_z;
        } else {
            xy_exceeded = true;
            z_exceeded = true;
            xy_measured = std::sqrt(vx * vx + vy * vy);
            xy_threshold = 0.0;
            z_measured = std::abs(vz);
            z_threshold = 0.0;
        }

        if (!ss.velocity_xy_exceeded && xy_exceeded) {
            emitSafetySignal(FlightSignal::VelocityXY, "velocity_xy_exceeded", xy_measured,
                             xy_threshold, 0.0);
        }
        ss.velocity_xy_exceeded = xy_exceeded;

        if (!ss.velocity_z_exceeded && z_exceeded) {
            emitSafetySignal(FlightSignal::VelocityZ, "velocity_z_exceeded", z_measured,
                             z_threshold, 0.0);
        }
        ss.velocity_z_exceeded = z_exceeded;
    }
}

void HealthMonitorState::onFreshImu(std::vector<FlightSignal>& signals) {
    const auto config = controller_.getConfig();
    if (config.tracking_backend != TrackingBackend::PX4_LOCAL)
        return;
    const auto& setpoint = controller_.getSetpoint();
    const double acc_xy = std::sqrt(setpoint.ax * setpoint.ax + setpoint.ay * setpoint.ay);
    if (acc_xy > config.safety.acc_saturation_xy) {
        last_safety_trigger_.triggered = true;
        last_safety_trigger_.event_id = static_cast<uint32_t>(FlightSignal::SaturationXY);
        last_safety_trigger_.signal = FlightSignal::SaturationXY;
        last_safety_trigger_.reason = "acc_saturation_xy";
        last_safety_trigger_.measured = acc_xy;
        last_safety_trigger_.threshold = config.safety.acc_saturation_xy;
        last_safety_trigger_.dt_s = 0.0;
        last_safety_trigger_.timestamp_s = controller_.getCurrentTime();
        controller_.logError("[HealthMonitor] safety_trigger reason=acc_saturation_xy event_id=43");
        signals.push_back(FlightSignal::SaturationXY);
    }
    const double acc_z = std::abs(setpoint.az);
    if (acc_z > config.safety.acc_saturation_z) {
        last_safety_trigger_.triggered = true;
        last_safety_trigger_.event_id = static_cast<uint32_t>(FlightSignal::SaturationZ);
        last_safety_trigger_.signal = FlightSignal::SaturationZ;
        last_safety_trigger_.reason = "acc_saturation_z";
        last_safety_trigger_.measured = acc_z;
        last_safety_trigger_.threshold = config.safety.acc_saturation_z;
        last_safety_trigger_.dt_s = 0.0;
        last_safety_trigger_.timestamp_s = controller_.getCurrentTime();
        controller_.logError("[HealthMonitor] safety_trigger reason=acc_saturation_z event_id=44");
        signals.push_back(FlightSignal::SaturationZ);
    }
}

void HealthMonitorState::checkSensorActiveEdge(const SensorData::TopicStats& stats,
                                               bool& was_active, FlightSignal signal,
                                               const char* reason, double threshold) {
    const bool currently_active = stats.is_active;
    if (was_active && !currently_active) {
        emitSafetySignal(signal, reason, stats.time_since_last_msg, threshold,
                         stats.time_since_last_msg);
    }
    was_active = currently_active;
}

}  // namespace px4_multirotor_controller
