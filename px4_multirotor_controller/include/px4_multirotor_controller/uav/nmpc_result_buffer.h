#pragma once


#include <mutex>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/common/time.h"

namespace px4_multirotor_controller {

namespace nmpc_solver_status {
constexpr int kDispatcherBusy = -1;
constexpr int kReferenceSamplingFailed = -2;
constexpr int kBackendUnavailable = -3;
}  // namespace nmpc_solver_status

struct NmpcSolveResult {
    uint64_t control_generation{0};
    uint64_t sequence{0};
    bool success{false};
    bool timed_out{false};
    int solver_status{0};
    double solve_time_ms{0.0};
    Time stamp;
    AttitudeRateTarget target;
};

class NmpcResultBuffer {
   public:
    uint64_t beginGeneration();
    void invalidateGeneration(uint64_t generation = 0);
    uint64_t activeGeneration() const;
    bool isGenerationActive(uint64_t generation) const;
    uint64_t reserveRequestSequence();
    uint64_t lastRequestSequence() const;
    bool store(const NmpcSolveResult& result);
    bool consumeNewerThan(uint64_t sequence, NmpcSolveResult& result) const;
    bool hasFreshSuccess(const Time& now, double timeout) const;
    static bool isResultTimestampFresh(const Time& stamp, const Time& now, double timeout);

   private:
    mutable std::mutex mutex_;
    NmpcSolveResult latest_;
    bool has_result_{false};
    uint64_t generation_counter_{0};
    uint64_t active_generation_{0};
    uint64_t request_counter_{0};
};

}  // namespace px4_multirotor_controller
