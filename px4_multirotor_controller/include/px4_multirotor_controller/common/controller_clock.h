#pragma once

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <ratio>

namespace px4_multirotor_controller {

// The controller's own clock: the time passed to DroneController::update().
//
// State timers (setpoint throttling, request retry intervals, log gates)
// measure this time, not the host's wall clock. So they follow simulated or
// replayed time exactly and stay identical to wall time on hardware, where
// update() receives the current ROS time. The value is per thread and is set
// at the start of every update(), so it is valid on the thread that updates
// the controller (the ROS node's loop thread, or the module's home host worker).
struct ControllerClock {
    using rep = double;
    using period = std::ratio<1>;
    using duration = std::chrono::duration<double>;
    using time_point = std::chrono::time_point<ControllerClock, duration>;
    static constexpr bool is_steady = true;

    static time_point now() noexcept { return time_point(duration(current())); }
    static void set(double seconds) noexcept { current() = seconds; }

   private:
    static double& current() noexcept {
        thread_local double seconds = 0.0;
        return seconds;
    }
};

inline double controllerDueAt(double start, double interval) {
    double due = start + interval;
    if (due - start < interval)
        due = std::nextafter(due, std::numeric_limits<double>::infinity());
    return due;
}

inline double controllerStrictDueAt(double start, double interval) {
    double due = start + interval;
    if (due - start <= interval)
        due = std::nextafter(due, std::numeric_limits<double>::infinity());
    return due;
}

inline std::optional<double> earlierDeadline(std::optional<double> a, std::optional<double> b) {
    if (!a) return b;
    if (!b) return a;
    return *a < *b ? a : b;
}

// The flight states use this controller-time interval, including when ROS
// time is paused or replayed. Reset does not start a stopped timer.
class ControllerTimer {
   public:
    void start() { start_time_ = ControllerClock::now(); started_ = true; }
    ControllerClock::duration elapsed() const {
        return started_ ? ControllerClock::now() - start_time_ : ControllerClock::duration(0);
    }
    bool started() const { return started_; }
    void reset() { start_time_ = ControllerClock::now(); }
    void stop() { started_ = false; }

    // First representable controller time at which elapsed() >= interval.
    // Keep the same subtraction used by the existing due check; plain
    // start + interval can round down and repeatedly arm an early deadline.
    std::optional<double> dueAt(double interval) const {
        if (!started_) return std::nullopt;
        return controllerDueAt(start_time_.time_since_epoch().count(), interval);
    }

   private:
    ControllerClock::time_point start_time_{};
    bool started_{false};
};

}  // namespace px4_multirotor_controller
