#pragma once

#include <chrono>
#include <ratio>

#include <state_machine/runtime/steady_timer.hpp>

namespace px4_multirotor_controller {

// The controller's own clock: the time passed to DroneController::update().
//
// State timers (setpoint throttling, request retry intervals, log gates)
// measure this time, not the host's wall clock. So they follow simulated or
// replayed time exactly and stay identical to wall time on hardware, where
// update() receives the current ROS time. The value is per thread and is set
// at the start of every update(), so it is valid on the thread that updates
// the controller (the ROS node's loop thread, or the owner thread of the
// module).
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

using ControllerTimer = ::state_machine::runtime::Timer<ControllerClock>;

}  // namespace px4_multirotor_controller
