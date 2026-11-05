#pragma once

#include "px4_multirotor_controller/common/controller_clock.h"

namespace px4_multirotor_controller {

inline bool shouldRunEvery(ControllerTimer& timer, double interval_seconds, bool reset_on_fire) {
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
