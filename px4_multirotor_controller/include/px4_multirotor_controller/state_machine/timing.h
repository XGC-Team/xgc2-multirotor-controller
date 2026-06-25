#pragma once

#include <state_machine/runtime/steady_timer.hpp>

namespace px4_multirotor_controller {

template <typename Clock, typename Duration>
bool shouldRunEvery(::state_machine::runtime::Timer<Clock, Duration>& timer,
                    double interval_seconds, bool reset_on_fire) {
    if (!timer.started()) {
        timer.start();
        return true;
    }
    if (timer.elapsed().count() >= interval_seconds) {
        if (reset_on_fire) {
            timer.reset();
        }
        return true;
    }
    return false;
}

}  // namespace px4_multirotor_controller
