#include "px4_multirotor_controller/input/command_input_producer.h"

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <utility>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {
namespace {

const std::unordered_map<std::string, ::state_machine::EventId> kCommandEventMap = {
    {"takeoff", event_type::CMD_TAKEOFF}, {"Takeoff", event_type::CMD_TAKEOFF},
    {"TAKEOFF", event_type::CMD_TAKEOFF}, {"land", event_type::CMD_LAND},
    {"Land", event_type::CMD_LAND},       {"LAND", event_type::CMD_LAND},
    {"hover", event_type::CMD_HOVER},     {"Hover", event_type::CMD_HOVER},
    {"HOVER", event_type::CMD_HOVER},     {"custom1", event_type::CMD_CUSTOM1},
    {"Custom1", event_type::CMD_CUSTOM1}, {"CUSTOM1", event_type::CMD_CUSTOM1},
};

}  // namespace

CommandInputProducer::CommandInputProducer(ros::NodeHandle& nh, EventSink event_sink,
                                           uint32_t queue_size)
    : event_sink_(std::move(event_sink)) {
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

    const auto it = kCommandEventMap.find(msg->data);
    if (it == kCommandEventMap.end()) {
        ROS_WARN("[CommandInputProducer] Unknown command: %s", msg->data.c_str());
        return;
    }

    ::state_machine::Event event(it->second,
                                 ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = "command";

    if (!event_sink_) {
        ROS_ERROR("[CommandInputProducer] Event sink is not configured");
        return;
    }

    auto status = event_sink_(std::move(event));
    if (!status.ok()) {
        ROS_ERROR("[CommandInputProducer] Failed to post command event %s: %s", msg->data.c_str(),
                  status.message.c_str());
        return;
    }

    ROS_INFO("[CommandInputProducer] Received %s command", msg->data.c_str());
}

}  // namespace px4_multirotor_controller
