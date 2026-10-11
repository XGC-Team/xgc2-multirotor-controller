#include <gtest/gtest.h>
#include <atomic>
#include <condition_variable>
#include <future>
#include <limits>
#include "px4_multirotor_controller/driver/controller_driver.h"
#include "px4_multirotor_controller/driver/controller_config.h"
using namespace px4_multirotor_controller;
TEST(ControllerStatistics, OriginalWindowJitterAndSingleMessageHeartbeat) {
    SensorData data; SensorStatistics statistics(data); statistics.start(Time(100.0));
    statistics.observe(SensorStream::State, Time(100.01));
    statistics.advance(Time(101.0)); statistics.advance(Time(102.0));
    statistics.advance(Time(102.6)); EXPECT_TRUE(data.state_stats.is_active);
    statistics.advance(Time(103.0)); EXPECT_FALSE(data.state_stats.is_active);
    EXPECT_DOUBLE_EQ(data.state_stats.time_since_last_msg, 2.99);
    statistics.observe(SensorStream::State, Time(103.1)); EXPECT_TRUE(data.state_stats.is_active);
    statistics.resetNewFlags(); EXPECT_FALSE(data.state_stats.is_new);
    for (int i = 1; i <= 10; ++i) statistics.observe(SensorStream::LocalPose, Time(103.0 + i * 0.02));
    statistics.advance(Time(103.3));
    EXPECT_NEAR(data.local_pos_stats.frequency_hz, 50.0, 1e-9);
    EXPECT_NEAR(data.local_pos_stats.dt_max, 0.02, 1e-9);
    EXPECT_GT(data.local_pos_stats.jitter, 0.0); // original first dt is zero
}
TEST(ControllerConfiguration, OwningProfileValuesAndValidation) {
    auto getter = [](const std::string& key, ControllerParameterValue& value) {
        if (key == "world_boundary_json") { value = std::string("null"); return true; }
        if (key == "tracking_backend") { value = std::string("dfbc"); return true; }
        if (key == "nmpc/control_period") { value = -1.0; return true; }
        return false;
    };
    auto config = readControllerConfig(ControllerParameters(getter));
    EXPECT_EQ(config.tracking_backend, TrackingBackend::DFBC);
    EXPECT_DOUBLE_EQ(config.takeoff_altitude, 2.3); // owning YAML, not C++ default
    EXPECT_DOUBLE_EQ(config.nmpc.control_period, 0.01); // unchanged validation
    EXPECT_TRUE(config.dfbc.acceleration_correction_enabled);
}
TEST(NmpcGeneration, InvalidatedAndUnissuedResultsCannotPoisonNextScope) {
    NmpcResultBuffer buffer;
    const auto first = buffer.beginGeneration(); const auto id1 = buffer.reserveRequestSequence();
    NmpcSolveResult result; result.control_generation = first; result.sequence = id1; result.success = true;
    ASSERT_TRUE(buffer.store(result));
    buffer.invalidateGeneration(first);
    const auto second = buffer.beginGeneration(); const auto id2 = buffer.reserveRequestSequence();
    ASSERT_GT(id2, id1); EXPECT_FALSE(buffer.store(result));
    NmpcSolveResult output; EXPECT_FALSE(buffer.consumeNewerThan(0, output));
    result.control_generation = second; result.sequence = id2 + 100;
    EXPECT_FALSE(buffer.store(result));
    result.sequence = id2; EXPECT_TRUE(buffer.store(result));
    ASSERT_TRUE(buffer.consumeNewerThan(id1, output)); EXPECT_EQ(output.sequence, id2);
}
TEST(NmpcExecution, ConfigSnapshotBusyLateCompletionStopAndRestart) {
    SensorData data; DroneController controller(data); ControllerConfig config; controller.setConfig(config);
    reference::AnalyticReference ref; ref.start_time = ref.header.stamp = Time(100.0);
    ref.duration = 60.0; ref.analytic_type = reference::AnalyticReference::ANALYTIC_HOLD;
    ref.origin.position.z = 1.0; ref.origin.orientation.w = 1.0;
    ASSERT_TRUE(controller.activeTrajectoryCache().updateAnalytic(ref, Time(100.0)));
    std::mutex mutex; std::condition_variable condition; bool entered = false, release = false;
    std::atomic<unsigned> completions{0}; std::atomic<double> captured{0.0};
    NmpcExecution execution(controller, [] { return Time(100.01); },
        [&](::state_machine::Event) { ++completions; return ::state_machine::Status{}; },
        [&](const NmpcExecution::Request& request) {
            captured.store(request.config.nmpc.gravity);
            { std::unique_lock<std::mutex> lock(mutex); entered = true; condition.notify_all();
              condition.wait(lock, [&] { return release; }); }
            NmpcSolveResult result; result.success = true; result.target.thrust = 0.61; return result;
        });
    execution.start();
    auto make_request = [&] {
        auto& buffer = controller.nmpcResultBuffer();
        ::state_machine::Event event(output_event_type::REQUEST_NMPC_SOLVE, ::state_machine::EventTimestamp{100.01});
        event.correlation_id = buffer.reserveRequestSequence();
        event.payload["control_generation"] = static_cast<int64_t>(buffer.activeGeneration());
        return event;
    };
    controller.nmpcResultBuffer().beginGeneration();
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
        auto event = make_request(); event.timestamp = invalid;
        EXPECT_NO_THROW(EXPECT_TRUE(execution.handle(event)));
    }
    EXPECT_EQ(completions.load(), 0U);
    NmpcSolveResult no_result;
    EXPECT_FALSE(controller.nmpcResultBuffer().consumeNewerThan(0, no_result));
    ASSERT_TRUE(execution.handle(make_request()));
    { std::unique_lock<std::mutex> lock(mutex); ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&] { return entered; })); }
    config.nmpc.gravity = 12.0; controller.setConfig(config);
    ASSERT_TRUE(execution.handle(make_request())); // deterministic busy rejection
    NmpcSolveResult busy; ASSERT_TRUE(controller.nmpcResultBuffer().consumeNewerThan(0, busy));
    EXPECT_EQ(busy.solver_status, nmpc_solver_status::kDispatcherBusy);
    auto stopped = std::async(std::launch::async, [&] { execution.stop(); });
    // Wait on the actual generation invalidation, not a timing sleep.
    const auto invalidation_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (controller.nmpcResultBuffer().activeGeneration() != 0 && std::chrono::steady_clock::now() < invalidation_deadline) std::this_thread::yield();
    EXPECT_EQ(controller.nmpcResultBuffer().activeGeneration(), 0U);
    { std::lock_guard<std::mutex> lock(mutex); release = true; } condition.notify_all();
    ASSERT_EQ(stopped.wait_for(std::chrono::seconds(2)), std::future_status::ready); stopped.get();
    EXPECT_DOUBLE_EQ(captured.load(), 9.8066); // accepted config, not mutable 12.0
    EXPECT_EQ(completions.load(), 1U); // busy failure only, late success invalidated
    EXPECT_FALSE(controller.nmpcResultBuffer().consumeNewerThan(0, busy));
    execution.start(); controller.nmpcResultBuffer().beginGeneration();
    ASSERT_TRUE(execution.handle(make_request()));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (completions.load() < 2 && std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
    EXPECT_EQ(completions.load(), 2U);
    ASSERT_TRUE(controller.nmpcResultBuffer().consumeNewerThan(0, busy)); EXPECT_TRUE(busy.success);
    EXPECT_EQ(busy.stamp, Time(100.01)); // original accepted request time, not completion time
    EXPECT_DOUBLE_EQ(captured.load(), 12.0);
    execution.stop();
}

TEST(NmpcResultFreshness, FutureDuplicateMalformedAndDisabledTimeoutPolicy) {
    NmpcResultBuffer buffer; const auto generation = buffer.beginGeneration();
    NmpcSolveResult result; result.control_generation = generation;
    result.sequence = buffer.reserveRequestSequence(); result.success = true;
    result.stamp = Time(0.0);
    ASSERT_TRUE(buffer.store(result));
    EXPECT_TRUE(buffer.hasFreshSuccess(Time(0.0), 0.1));
    EXPECT_TRUE(buffer.hasFreshSuccess(Time(1.0), 0.0));
    EXPECT_TRUE(buffer.hasFreshSuccess(Time(1.0), -1.0));
    result.sequence = buffer.reserveRequestSequence(); result.stamp = Time(100.0);
    ASSERT_TRUE(buffer.store(result));
    EXPECT_FALSE(buffer.hasFreshSuccess(Time(99.0), 0.1));
    EXPECT_FALSE(buffer.hasFreshSuccess(Time(99.0), 0.0));
    EXPECT_FALSE(buffer.hasFreshSuccess(Time(99.0), -1.0));
    EXPECT_TRUE(buffer.hasFreshSuccess(Time(100.1), 0.1));
    EXPECT_FALSE(buffer.hasFreshSuccess(Time(100, 100000001), 0.1));
    result.stamp = Time(100.1); result.target.thrust = 0.91;
    EXPECT_FALSE(buffer.store(result)); // same request cannot rewrite stamp or target
    NmpcSolveResult stored; ASSERT_TRUE(buffer.consumeNewerThan(0, stored));
    EXPECT_EQ(stored.stamp, Time(100.0)); EXPECT_NE(stored.target.thrust, 0.91);
    EXPECT_FALSE(buffer.hasFreshSuccess(Time(100.2), 0.1)); // duplicate did not refresh success
    result.sequence = buffer.reserveRequestSequence(); result.stamp.nsec = 1000000000U;
    EXPECT_FALSE(buffer.store(result));
}
