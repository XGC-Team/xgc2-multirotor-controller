#pragma once

#include <atomic>
#include "px4_multirotor_controller/driver/nmpc_execution.h"
#include "px4_multirotor_controller/driver/sensor_statistics.h"

namespace px4_multirotor_controller {
// Owns domain scheduling and async work. Transport adapters only marshal inputs,
// supply the domain clock, and consume outputs; no transport-specific solver or
// second health policy lives beside this driver.
class ControllerDriver {
public:
    explicit ControllerDriver(SensorData& sensor)
     : controller_(sensor), statistics_(sensor),
       nmpc_(controller_, [this] { return Time().fromNSec(clock_ns_.load()); },
             [this](::state_machine::Event event) { return controller_.getStateMachine().postEvent(std::move(event)); }) {}
    DroneController& controller() { return controller_; }
    SensorStatistics& statistics() { return statistics_; }
    NmpcExecution& nmpc() { return nmpc_; }
    void start(const Time& now) { clock_ns_.store(now.toNSec()); statistics_.start(now); nmpc_.start(); active_ = true; }
    void update(const Time& now) {
        if (!active_) return;
        clock_ns_.store(now.toNSec()); statistics_.advance(now); controller_.update(now.toSec());
    }
    // In-flight completion is joined before return; no callback can outlive the
    // adapter. Pending work is dropped and new dispatch is rejected by the same
    // original worker stop/busy gate used by ROS.
    void stop() { active_ = false; nmpc_.stop(); }
private:
    DroneController controller_;
    SensorStatistics statistics_;
    std::atomic<uint64_t> clock_ns_{0};
    NmpcExecution nmpc_;
    bool active_{false};
};
}
