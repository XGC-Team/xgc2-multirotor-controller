#include "px4_multirotor_controller/driver/sensor_statistics.h"

#include <gtest/gtest.h>

using namespace px4_multirotor_controller;

namespace {
constexpr uint32_t kEpoch = 1792400000;
Time epoch(uint32_t nsec = 0) {
    return Time(kEpoch, nsec);
}
}  // namespace

TEST(SensorStatisticsClock, OriginalWindowAndHeartbeatCadence) {
    SensorData data;
    SensorStatistics statistics(data);
    statistics.start(Time(100.0));
    statistics.observe(SensorStream::State, Time(100.01));
    statistics.advance(Time(101.0));
    statistics.advance(Time(102.0));
    statistics.advance(Time(102.6));
    EXPECT_TRUE(data.state_stats.is_active);
    statistics.advance(Time(103.0));
    EXPECT_FALSE(data.state_stats.is_active);
    EXPECT_DOUBLE_EQ(data.state_stats.time_since_last_msg, 2.99);
    statistics.observe(SensorStream::State, Time(103.1));
    EXPECT_TRUE(data.state_stats.is_active);
    statistics.resetNewFlags();
    EXPECT_FALSE(data.state_stats.is_new);
    for (int i = 1; i <= 10; ++i)
        statistics.observe(SensorStream::LocalPose, Time(103.0 + i * 0.02));
    statistics.advance(Time(103.3));
    EXPECT_NEAR(data.local_pos_stats.frequency_hz, 50.0, 1e-9);
    EXPECT_NEAR(data.local_pos_stats.dt_max, 0.02, 1e-9);
    EXPECT_GT(data.local_pos_stats.jitter, 0.0);
}

TEST(SensorStatisticsClock, ZeroToEpochSkipsMissedPeriodsAndKeepsOriginalPhase) {
    SensorData data;
    SensorStatistics statistics(data);
    statistics.start(Time());
    statistics.observe(SensorStream::State, epoch(20000000));
    statistics.observe(SensorStream::LocalPose, epoch(10000000));
    statistics.observe(SensorStream::LocalPose, epoch(20000000));
    statistics.advance(epoch(25000000));
    EXPECT_DOUBLE_EQ(data.local_pos_stats.frequency_hz, 100.0);
    EXPECT_TRUE(data.state_stats.is_active);
    EXPECT_DOUBLE_EQ(data.state_stats.time_since_last_msg, .005);
    statistics.observe(SensorStream::LocalPose, epoch(40000000));
    statistics.advance(epoch(99999999));
    EXPECT_DOUBLE_EQ(data.local_pos_stats.frequency_hz, 100.0);
    statistics.advance(epoch(100000000));
    EXPECT_NEAR(data.local_pos_stats.frequency_hz, 2.0 / .03, 1e-12);
    statistics.advance(Time(kEpoch + 2, 0));
    EXPECT_TRUE(data.state_stats.is_active);
    statistics.advance(Time(kEpoch + 2, 600000000));
    EXPECT_TRUE(
        data.state_stats.is_active);  // heartbeat is still due at the original integer phase
    statistics.advance(Time(kEpoch + 3, 0));
    EXPECT_FALSE(data.state_stats.is_active);
}

TEST(SensorStatisticsClock, LargeForwardJumpKeepsFractionalPhaseAndSummarizesOnce) {
    SensorData data;
    const double initial_rate = data.local_pos_stats.frequency_hz;
    SensorStatistics statistics(data);
    statistics.start(Time(100, 25000000));
    double effective = -1;
    statistics.setQualityOutput(SensorStream::Pose, &effective);
    statistics.observe(SensorStream::Pose, Time(100, 30000000));
    statistics.observe(SensorStream::Pose, Time(100, 40000000));
    statistics.markValidFrame(SensorStream::Pose);
    statistics.markValidFrame(SensorStream::Pose);
    statistics.advance(epoch(50000000));
    EXPECT_DOUBLE_EQ(effective, 200.0);  // no extra empty statistics passes replayed
    statistics.observe(SensorStream::LocalPose, epoch(60000000));
    statistics.observe(SensorStream::LocalPose, epoch(70000000));
    statistics.advance(epoch(124999999));
    EXPECT_DOUBLE_EQ(data.local_pos_stats.frequency_hz, initial_rate);
    statistics.advance(epoch(125000000));
    EXPECT_DOUBLE_EQ(data.local_pos_stats.frequency_hz, 100.0);
    EXPECT_FALSE(data.vrpn_pose_stats.is_active);
}

TEST(SensorStatisticsClock, ImplicitEpochStartStillWaitsOneNormalWindow) {
    SensorData data;
    const double initial_rate = data.local_pos_stats.frequency_hz;
    SensorStatistics statistics(data);
    statistics.advance(epoch());
    statistics.observe(SensorStream::LocalPose, epoch(10000000));
    statistics.observe(SensorStream::LocalPose, epoch(20000000));
    statistics.advance(epoch(99999999));
    EXPECT_DOUBLE_EQ(data.local_pos_stats.frequency_hz, initial_rate);
    statistics.advance(epoch(100000000));
    EXPECT_DOUBLE_EQ(data.local_pos_stats.frequency_hz, 100.0);
}
