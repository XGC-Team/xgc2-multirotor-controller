#include "px4_multirotor_controller/driver/sensor_statistics.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace px4_multirotor_controller {
namespace {
// Missed periods do not replay measurements. Keep the original deadline grid
// while skipping directly to its next tick, including the first /clock jump.
Time nextDeadline(const Time& deadline, const Time& now, uint64_t period_ns) {
    const uint64_t now_ns = now.toNSec();
    const uint64_t remaining_ns = period_ns - (now_ns - deadline.toNSec()) % period_ns;
    return Time().fromNSec(now_ns + remaining_ns);
}

bool isPoseStreamIndex(std::size_t index) {
    return index == static_cast<std::size_t>(SensorStream::LocalPose) ||
           index == static_cast<std::size_t>(SensorStream::Pose);
}

constexpr std::array<std::size_t, 2> kPoseStreamIndices{
    static_cast<std::size_t>(SensorStream::LocalPose),
    static_cast<std::size_t>(SensorStream::Pose),
};
}  // namespace

SensorStatistics::SensorStatistics(SensorData& s)
 : tracks_{{{&s.uav_state_estimate_stats, {}, {}, {}}, {&s.local_pos_stats, {}, {}, {}},
             {&s.local_velocity_stats, {}, {}, {}}, {&s.imu_stats, {}, {}, {}},
             {&s.state_stats, {}, {}, {}}, {&s.battery_stats, {}, {}, {}},
             {&s.vrpn_pose_stats, {}, {}, {}}}} {}

void SensorStatistics::setPoseStaleTimeout(double timeout_s) {
    if (std::isfinite(timeout_s) && timeout_s > 0.0) {
        pose_stale_timeout_s_ = timeout_s;
    }
}

void SensorStatistics::start(const Time& now) {
    if (started_) return;
    next_stats_ = now + Duration(0.1);
    next_heartbeat_ = now + Duration(1.0);
    started_ = true;
}
void SensorStatistics::observe(SensorStream stream, const Time& receipt) {
    auto& track = tracks_.at(static_cast<unsigned>(stream));
    const double dt = track.last.isZero() ? 0.0 : (receipt - track.last).toSec();
    track.times.push_back(receipt);
    track.dt.push_back(dt);
    if (track.times.size() > 10) { track.times.pop_front(); track.dt.pop_front(); }
    track.last = receipt;
    track.stats->last_message_time = receipt;
    track.stats->is_active = true;
    track.stats->is_new = true;
}
void SensorStatistics::updateStatistics() {
    for (auto& track : tracks_) {
        if (track.times.size() < 2) continue;
        const double duration = (track.times.back() - track.times.front()).toSec();
        if (duration > 0.0) track.stats->frequency_hz = static_cast<double>(track.times.size() - 1) / duration;
        double dt_max = 0.0;
        for (double dt : track.dt) dt_max = std::max(dt_max, dt);
        track.stats->dt_max = dt_max;
        double mean = 0.0;
        for (double dt : track.dt) mean += dt;
        mean /= track.dt.size();
        double variance = 0.0;
        for (double dt : track.dt) { const double diff = dt - mean; variance += diff * diff; }
        track.stats->jitter = track.dt.size() < 2 ? 0.0 : std::sqrt(variance / track.dt.size());
        if (track.effective_frequency && duration > 0.0) {
            *track.effective_frequency = static_cast<double>(track.valid_frames) / duration;
            track.valid_frames = 0;
        }
    }
}
void SensorStatistics::updateHeartbeats(const Time& now) {
    const Duration pose_timeout(pose_stale_timeout_s_);
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        auto& track = tracks_[i];
        if (track.times.empty()) continue;
        const double elapsed = (now - track.times.back()).toSec();
        track.stats->time_since_last_msg = elapsed;
        if (isPoseStreamIndex(i)) {
            track.stats->is_active =
                now < (track.times.back() + pose_timeout) && elapsed < pose_stale_timeout_s_;
        } else {
            track.stats->is_active = elapsed <= 2.5;
        }
    }
}
void SensorStatistics::updatePoseTimeouts(const Time& now) {
    const Duration pose_timeout(pose_stale_timeout_s_);
    for (const std::size_t i : kPoseStreamIndices) {
        auto& track = tracks_[i];
        if (track.times.empty() || !track.stats->is_active) continue;
        const double elapsed = (now - track.times.back()).toSec();
        if (now >= (track.times.back() + pose_timeout) || elapsed >= pose_stale_timeout_s_) {
            track.stats->time_since_last_msg = elapsed;
            track.stats->is_active = false;
        }
    }
}
void SensorStatistics::advance(const Time& now) {
    if (!started_) start(now);
    if (now >= next_stats_) {
        updateStatistics();
        next_stats_ = nextDeadline(next_stats_, now, 100000000ULL);
    }
    if (now >= next_heartbeat_) {
        updateHeartbeats(now);
        next_heartbeat_ = nextDeadline(next_heartbeat_, now, 1000000000ULL);
    } else {
        updatePoseTimeouts(now);
    }
}
std::optional<Time> SensorStatistics::nextWake() const {
    if (!started_) return std::nullopt;
    Time due = std::min(next_stats_, next_heartbeat_);
    const Duration pose_timeout(pose_stale_timeout_s_);
    for (const std::size_t i : kPoseStreamIndices) {
        const auto& track = tracks_[i];
        if (track.stats->is_active && !track.times.empty()) {
            due = std::min(due, track.times.back() + pose_timeout);
        }
    }
    return due;
}
void SensorStatistics::resetNewFlags() { for (auto& track : tracks_) track.stats->is_new = false; }
}
