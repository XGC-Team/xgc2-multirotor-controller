#include "px4_multirotor_controller/ros_log_sink.h"

#include <ros/console.h>

#include "px4_multirotor_controller/common/core_log.h"

namespace px4_multirotor_controller {
namespace {

void rosSink(LogLevel level, const char* message) {
    switch (level) {
        case LogLevel::kError:
            ROS_ERROR("%s", message);
            break;
        case LogLevel::kWarn:
            ROS_WARN("%s", message);
            break;
        default:
            ROS_INFO("%s", message);
            break;
    }
}

}  // namespace

void installRosLogSink() {
    setLogSink(&rosSink);
}

}  // namespace px4_multirotor_controller
