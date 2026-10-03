#include "px4_multirotor_controller/uav/nmpc_result_buffer.h"
#include "px4_multirotor_controller/common/time.h"

namespace px4_multirotor_controller {

uint64_t NmpcResultBuffer::beginGeneration() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_generation_ = ++generation_counter_;
    has_result_ = false;
    return active_generation_;
}
void NmpcResultBuffer::invalidateGeneration(uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation == 0 || generation == active_generation_) {
        active_generation_ = 0;
        has_result_ = false;
    }
}
uint64_t NmpcResultBuffer::activeGeneration() const {
    std::lock_guard<std::mutex> lock(mutex_); return active_generation_;
}
bool NmpcResultBuffer::isGenerationActive(uint64_t generation) const {
    std::lock_guard<std::mutex> lock(mutex_); return generation != 0 && generation == active_generation_;
}
uint64_t NmpcResultBuffer::reserveRequestSequence() {
    std::lock_guard<std::mutex> lock(mutex_); return ++request_counter_;
}
uint64_t NmpcResultBuffer::lastRequestSequence() const {
    std::lock_guard<std::mutex> lock(mutex_); return request_counter_;
}
bool NmpcResultBuffer::store(const NmpcSolveResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_generation_ == 0 || result.control_generation != active_generation_ ||
        result.sequence == 0 || result.sequence > request_counter_ ||
        result.stamp.nsec >= 1000000000U) return false;
    if (has_result_ && result.sequence <= latest_.sequence) {
        return false;
    }
    latest_ = result;
    has_result_ = true;
    return true;
}

bool NmpcResultBuffer::consumeNewerThan(uint64_t sequence, NmpcSolveResult& result) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_result_ || active_generation_ == 0 || latest_.control_generation != active_generation_ || latest_.sequence <= sequence) {
        return false;
    }
    result = latest_;
    return true;
}

bool NmpcResultBuffer::isResultTimestampFresh(const Time& stamp, const Time& now, double timeout) {
    // Time already rejects nonfinite fromSec inputs; public sec/nsec fields
    // still require canonical nanoseconds. Logical time zero remains valid.
    if (stamp.nsec >= 1000000000U || now.nsec >= 1000000000U || stamp > now) return false;
    const double age = static_cast<double>(now.toNSec() - stamp.toNSec()) * 1e-9;
    return timeout <= 0.0 || age <= timeout;
}

bool NmpcResultBuffer::hasFreshSuccess(const Time& now, double timeout) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_result_ && active_generation_ != 0 && latest_.control_generation == active_generation_ && latest_.success &&
           isResultTimestampFresh(latest_.stamp, now, timeout);
}

}  // namespace px4_multirotor_controller
