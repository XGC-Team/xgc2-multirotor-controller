#include <gtest/gtest.h>

#include <chrono>

#include "px4_multirotor_controller/driver/sensor_statistics.h"

using namespace px4_multirotor_controller;

TEST(ControllerStatistics, FirstEpochClockDoesNotReplayMissedDeadlines) {
    SensorData data;
    SensorStatistics statistics(data);
    statistics.start(Time());
    const Time epoch(1792520000U, 350000000U);
    statistics.observe(SensorStream::State, epoch);
    statistics.observe(SensorStream::LocalPose, epoch - Duration(0.02));
    statistics.observe(SensorStream::LocalPose, epoch);

    const auto started = std::chrono::steady_clock::now();
    statistics.advance(epoch);
    EXPECT_LT(std::chrono::steady_clock::now() - started, std::chrono::seconds(1));
    EXPECT_NEAR(data.local_pos_stats.frequency_hz, 50.0, 1e-9);
    EXPECT_TRUE(data.state_stats.is_active);
    EXPECT_DOUBLE_EQ(data.state_stats.time_since_last_msg, 0.0);

    // Keep the original 100 ms grid, rather than shifting it to epoch + 100 ms.
    statistics.observe(SensorStream::LocalPose, Time(1792520000U, 390000000U));
    statistics.advance(Time(1792520000U, 399999999U));
    EXPECT_NEAR(data.local_pos_stats.frequency_hz, 50.0, 1e-9);
    statistics.advance(Time(1792520000U, 400000000U));
    EXPECT_NEAR(data.local_pos_stats.frequency_hz, 2.0 / 0.06, 1e-9);

    // A paused clock does not repeat statistics or heartbeat work.
    data.local_pos_stats.frequency_hz = -1.0;
    statistics.advance(Time(1792520000U, 400000000U));
    EXPECT_DOUBLE_EQ(data.local_pos_stats.frequency_hz, -1.0);
    statistics.advance(Time(1792520000U, 999999999U));
    EXPECT_DOUBLE_EQ(data.state_stats.time_since_last_msg, 0.0);
    statistics.advance(Time(1792520001U, 0U));
    EXPECT_DOUBLE_EQ(data.state_stats.time_since_last_msg, 0.65);
    statistics.advance(Time(1792520002U, 0U));
    EXPECT_TRUE(data.state_stats.is_active);
    statistics.advance(Time(1792520003U, 0U));
    EXPECT_FALSE(data.state_stats.is_active);
    EXPECT_DOUBLE_EQ(data.state_stats.time_since_last_msg, 2.65);
}

TEST(ControllerStatistics, SkippedPeriodsKeepTheStartupPhaseAndQualityWindow) {
    SensorData data;
    SensorStatistics statistics(data);
    statistics.start(Time(10U, 23000000U));
    double quality_frequency = -1.0;
    statistics.setQualityOutput(SensorStream::LocalPose, &quality_frequency);
    statistics.observe(SensorStream::LocalPose, Time(100U, 0U));
    statistics.markValidFrame(SensorStream::LocalPose);
    statistics.observe(SensorStream::LocalPose, Time(100U, 20000000U));
    statistics.markValidFrame(SensorStream::LocalPose);
    statistics.advance(Time(100U, 50000000U));
    EXPECT_DOUBLE_EQ(quality_frequency, 100.0);

    statistics.markValidFrame(SensorStream::LocalPose);
    statistics.advance(Time(100U, 122999999U));
    EXPECT_DOUBLE_EQ(quality_frequency, 100.0);
    statistics.advance(Time(100U, 123000000U));
    EXPECT_DOUBLE_EQ(quality_frequency, 50.0);
    statistics.advance(Time(100U, 123000000U));
    EXPECT_DOUBLE_EQ(quality_frequency, 50.0);
}
