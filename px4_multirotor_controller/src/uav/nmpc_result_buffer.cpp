#include "px4_multirotor_controller/uav/nmpc_result_buffer.h"

namespace px4_multirotor_controller {

void NmpcResultBuffer::store(const NmpcSolveResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_result_ && result.sequence < latest_.sequence) {
        return;
    }
    latest_ = result;
    has_result_ = true;
}

bool NmpcResultBuffer::consumeNewerThan(uint64_t sequence, NmpcSolveResult& result) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_result_ || latest_.sequence <= sequence) {
        return false;
    }
    result = latest_;
    return true;
}

bool NmpcResultBuffer::hasFreshSuccess(const ros::Time& now, double timeout) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_result_ && latest_.success &&
           (timeout <= 0.0 || (now - latest_.stamp).toSec() <= timeout);
}

}  // namespace px4_multirotor_controller
