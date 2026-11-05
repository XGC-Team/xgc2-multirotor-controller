#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <functional>
#include <state_machine/state_machine.hpp>
#include "px4_multirotor_controller/driver/command_input.h"

namespace px4_multirotor_controller {

class CommandInputProducer {
   public:
    using CommandSink = std::function<::state_machine::Status(FlightCommand, double)>;

    CommandInputProducer(ros::NodeHandle& nh, CommandSink command_sink, uint32_t queue_size);

   private:
    void commandCallback(const std_msgs::String::ConstPtr& msg);

    CommandSink command_sink_;
    ros::Subscriber command_sub_;
};

}  // namespace px4_multirotor_controller
