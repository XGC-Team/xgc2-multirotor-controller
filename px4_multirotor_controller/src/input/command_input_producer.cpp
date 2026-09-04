#include "px4_multirotor_controller/input/command_input_producer.h"

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <utility>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {
namespace {

const std::unordered_map<std::string, ::state_machine::EventId> kCommandEventMap = {
    {"takeoff", event_type::TAKEOFF_REQUESTED},
    {"Takeoff", event_type::TAKEOFF_REQUESTED},
    {"TAKEOFF", event_type::TAKEOFF_REQUESTED},
    {"land", event_type::LANDING_REQUESTED},
    {"Land", event_type::LANDING_REQUESTED},
    {"LAND", event_type::LANDING_REQUESTED},
    {"hover", event_type::HOVER_REQUESTED},
    {"Hover", event_type::HOVER_REQUESTED},
    {"HOVER", event_type::HOVER_REQUESTED},
    {"custom1", event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"Custom1", event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"CUSTOM1", event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"start", event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"Start", event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"START", event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"track", event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"Track", event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"TRACK", event_type::TRAJECTORY_TRACKING_REQUESTED},
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
