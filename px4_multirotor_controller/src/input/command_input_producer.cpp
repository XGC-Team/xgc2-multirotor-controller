#include "px4_multirotor_controller/input/command_input_producer.h"

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <utility>

#include "px4_multirotor_controller/driver/command_input.h"

namespace px4_multirotor_controller {
CommandInputProducer::CommandInputProducer(ros::NodeHandle& nh, CommandSink command_sink,
                                           uint32_t queue_size)
    : command_sink_(std::move(command_sink)) {
    command_sub_ =
        nh.subscribe("/command", queue_size, &CommandInputProducer::commandCallback, this);
}

void CommandInputProducer::commandCallback(const std_msgs::String::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[CommandInputProducer] Received null command message");
        return;
    }
    if (msg->data.empty()) {
        ROS_WARN("[CommandInputProducer] Received empty command");
        return;
    }
    static constexpr size_t MAX_COMMAND_LENGTH = 64;
    if (msg->data.length() > MAX_COMMAND_LENGTH) {
        ROS_WARN("[CommandInputProducer] Command too long (%zu chars), ignoring",
                 msg->data.length());
        return;
    }

    const auto command = parseFlightCommand(msg->data);
    if (!command) {
        ROS_WARN("[CommandInputProducer] Unknown command: %s", msg->data.c_str());
        return;
    }

    if (!command_sink_) {
        ROS_ERROR("[CommandInputProducer] Event sink is not configured");
        return;
    }

    auto status = command_sink_(*command, ros::Time::now().toSec());
    if (!status.ok()) {
        ROS_ERROR("[CommandInputProducer] Failed to post command event %s: %s", msg->data.c_str(),
                  status.message.c_str());
        return;
    }

    ROS_INFO("[CommandInputProducer] Received %s command", msg->data.c_str());
}

}  // namespace px4_multirotor_controller
