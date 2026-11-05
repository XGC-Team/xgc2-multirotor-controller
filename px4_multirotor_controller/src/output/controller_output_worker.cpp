#include "px4_multirotor_controller/output/controller_output_worker.h"

#include <type_traits>
#include <utility>

#include "px4_multirotor_controller/output/debug_output_consumer.h"
#include "px4_multirotor_controller/output/px4_command.h"

namespace px4_multirotor_controller {

ControllerOutputWorker::~ControllerOutputWorker() = default;
void ControllerOutputWorker::start() { worker_.start(); }
void ControllerOutputWorker::stop() { worker_.stop(); }
void ControllerOutputWorker::post(ControllerOutput output) { worker_.post(std::move(output)); }

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
