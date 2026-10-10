#pragma once

#include <atomic>
#include <functional>
#include "px4_multirotor_controller/driver/nmpc_execution.h"
#include "px4_multirotor_controller/driver/sensor_statistics.h"

namespace px4_multirotor_controller {
// Owns domain scheduling and async work. The ROS node and the module only marshal
// inputs, supply the domain clock, and consume outputs; no transport-specific solver
// or second health policy lives beside this driver.
class ControllerDriver {
public:
    // on_nmpc_result is called after every NMPC solve result (success, failure or
    // rejection) has been posted to the state machine, from the solver's worker thread or
    // from the thread that requested the solve. The module wakes its host with it, so the
    // update that consumes the result runs now and not at the next period.
    explicit ControllerDriver(SensorData& sensor, std::function<void()> on_nmpc_result = {})
     : controller_(sensor), statistics_(sensor), on_nmpc_result_(std::move(on_nmpc_result)),
       nmpc_(controller_, [this] { return Time().fromNSec(clock_ns_.load()); },
             [this](::state_machine::Event event) {
                 const auto status = controller_.getStateMachine().postEvent(std::move(event));
                 if (on_nmpc_result_) on_nmpc_result_();
                 return status;
             }) {}
    DroneController& controller() { return controller_; }
    SensorStatistics& statistics() { return statistics_; }
    NmpcExecution& nmpc() { return nmpc_; }
    void start(const Time& now) { clock_ns_.store(now.toNSec()); statistics_.start(now); nmpc_.start(); active_ = true; }
    void update(const Time& now) {
        if (!active_) return;
        clock_ns_.store(now.toNSec()); statistics_.advance(now); controller_.update(now.toSec());
    }
    // In-flight completion is joined before return; no callback can outlive the
    // caller. Pending work is dropped and new dispatch is rejected by the same
    // original worker stop/busy gate used by ROS.
    void stop() { active_ = false; nmpc_.stop(); }
private:
    DroneController controller_;
    SensorStatistics statistics_;
    std::atomic<uint64_t> clock_ns_{0};
    std::function<void()> on_nmpc_result_;
    NmpcExecution nmpc_;
    bool active_{false};
};
}
