#pragma once

#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/uav/reference_activation.h"

namespace px4_multirotor_controller {

class ReferenceActivationOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    ReferenceActivationOutputConsumer(
        ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
        DroneController& controller, uint32_t queue_size);

    std::string name() const override {
        return "ReferenceActivationOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    DroneController& controller_;
    ros::Publisher activation_pub_;
    ReferenceActivation activation_;
};

}  // namespace px4_multirotor_controller
