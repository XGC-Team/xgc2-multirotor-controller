#pragma once

#include <chrono>
#include <state_machine/state_machine.hpp>

namespace px4_multirotor_controller {
namespace state_machine_queries {

inline double activeStateElapsedSeconds(const ::state_machine::StateMachine& fsm,
                                        ::state_machine::RegionId region) {
    const auto state = fsm.currentState(region);
    if (state == 0) {
        return 0.0;
    }
    return std::chrono::duration<double>(fsm.elapsed(state)).count();
}

inline bool isActiveStateTimeout(const ::state_machine::StateMachine& fsm,
                                 ::state_machine::RegionId region, double seconds) {
    return activeStateElapsedSeconds(fsm, region) > seconds;
}

}  // namespace state_machine_queries
}  // namespace px4_multirotor_controller
