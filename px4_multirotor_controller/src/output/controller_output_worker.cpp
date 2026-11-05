#include "px4_multirotor_controller/output/controller_output_worker.h"

#include <type_traits>
#include <utility>

#include "px4_multirotor_controller/output/debug_output_consumer.h"
#include "px4_multirotor_controller/output/px4_command.h"

namespace px4_multirotor_controller {

ControllerOutputWorker::~ControllerOutputWorker() { stop(); }

void ControllerOutputWorker::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return;
    running_ = true;
    try {
        worker_ = std::thread(&ControllerOutputWorker::run, this);
    } catch (...) {
        running_ = false;
        throw;
    }
}

void ControllerOutputWorker::stop() {
    std::queue<ControllerOutput> discarded;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        pending_.swap(discarded);
    }
    ready_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void ControllerOutputWorker::post(ControllerOutput output) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        pending_.push(std::move(output));
    }
    ready_.notify_one();
}

void ControllerOutputWorker::run() {
    for (;;) {
        ControllerOutput output;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [this] { return !running_ || !pending_.empty(); });
            if (!running_) return;
            output = std::move(pending_.front());
            pending_.pop();
        }
        try {
            execute(output);
        } catch (...) {
            // The original asynchronous worker isolated failed tasks too.
        }
    }
}

void ControllerOutputWorker::execute(const ControllerOutput& output) {
    std::visit([](const auto& item) {
        using T = std::decay_t<decltype(item)>;
        if constexpr (std::is_same_v<T, OutputArm>) {
            detail::sendArmDisarmCommand(item.client, item.arm, "ArmOutputTask");
        } else if constexpr (std::is_same_v<T, OutputMode>) {
            detail::sendModeCommand(item.client, item.mode);
        } else if constexpr (std::is_same_v<T, OutputEvents> ||
                             std::is_same_v<T, OutputTracking> ||
                             std::is_same_v<T, OutputReference>) {
            item.publisher.publish(item.message);
        } else if constexpr (std::is_same_v<T, OutputSensorStats>) {
            for (size_t i = 0; i < item.stats.size(); ++i)
                item.publishers[i].publish(DebugOutputConsumer::makeTopicStatsMessage(item.stats[i]));
            item.publishers[7].publish(DebugOutputConsumer::makeVrpnQualityMessage(item.quality));
        } else if constexpr (std::is_same_v<T, OutputSensorDebug>) {
            DebugOutputConsumer::printSensorDebug(item.sensor);
        }
    }, output);
}

}  // namespace px4_multirotor_controller
