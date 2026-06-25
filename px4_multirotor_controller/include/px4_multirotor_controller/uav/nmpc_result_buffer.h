#pragma once

#include <ros/ros.h>

#include <mutex>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

namespace nmpc_solver_status {
constexpr int kDispatcherBusy = -1;
constexpr int kReferenceSamplingFailed = -2;
constexpr int kBackendUnavailable = -3;
}  // namespace nmpc_solver_status

struct NmpcSolveResult {
    uint64_t sequence{0};
    bool success{false};
    bool timed_out{false};
    int solver_status{0};
    double solve_time_ms{0.0};
    ros::Time stamp;
    AttitudeRateTarget target;
};

class NmpcResultBuffer {
   public:
    void store(const NmpcSolveResult& result);
    bool consumeNewerThan(uint64_t sequence, NmpcSolveResult& result) const;
    bool hasFreshSuccess(const ros::Time& now, double timeout) const;

   private:
    mutable std::mutex mutex_;
    NmpcSolveResult latest_;
    bool has_result_{false};
};

}  // namespace px4_multirotor_controller
