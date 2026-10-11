#pragma once

#include <array>
#include <deque>
#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

enum class SensorStream : unsigned { Estimate, LocalPose, LocalVelocity, Imu, State, Battery, Pose, Count };

// The controller's original TopicStatsManager policy, without ROS subscriptions
// or timers. The ROS node and the xgc2-module module both hand it receipt times
// and advance its same 10 Hz statistics / 1 Hz heartbeat clock. The 10-sample
// window and 2.5 s timeout deliberately retain the ROS policy, including a
// single-message heartbeat.
class SensorStatistics {
public:
    explicit SensorStatistics(SensorData& sensor);
    void start(const Time& now);
    void observe(SensorStream stream, const Time& receipt);
    void advance(const Time& now);
    void resetNewFlags();
    void setQualityOutput(SensorStream stream, double* effective_frequency) {
        tracks_.at(static_cast<unsigned>(stream)).effective_frequency = effective_frequency;
    }
    void markValidFrame(SensorStream stream) { ++tracks_.at(static_cast<unsigned>(stream)).valid_frames; }
private:
    struct Track {
        SensorData::TopicStats* stats;
        std::deque<Time> times;
        std::deque<double> dt;
        Time last;
        double* effective_frequency{nullptr};
        unsigned valid_frames{0};
    };
    void updateStatistics();
    void updateHeartbeats(const Time& now);
    std::array<Track, static_cast<unsigned>(SensorStream::Count)> tracks_;
    Time next_stats_, next_heartbeat_;
    bool started_{false};
};
}
