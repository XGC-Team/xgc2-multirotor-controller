#pragma once

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

class MpcTrajectoryBuffer {
   public:
    MpcTrajectoryState& active() {
        return active_;
    }
    const MpcTrajectoryState& active() const {
        return active_;
    }

    bool hasPending() const {
        return pending_.is_valid;
    }
    const MpcTrajectoryState& pending() const {
        return pending_;
    }

    void cachePending(const MpcTrajectoryState& trajectory) {
        pending_ = trajectory;
        pending_.is_valid = true;
        pending_.new_data_received = false;
    }

    bool promotePending(const ros::Time& activation_time) {
        if (!pending_.is_valid) {
            return false;
        }

        active_ = pending_;
        active_.planning_time = activation_time;
        active_.new_data_received = false;
        pending_ = MpcTrajectoryState{};
        return true;
    }

   private:
    MpcTrajectoryState active_;
    MpcTrajectoryState pending_;
};

}  // namespace px4_multirotor_controller
