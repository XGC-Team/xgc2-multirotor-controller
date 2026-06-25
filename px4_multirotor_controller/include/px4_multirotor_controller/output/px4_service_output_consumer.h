#pragma once

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

namespace px4_multirotor_controller {

class Px4ServiceOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    Px4ServiceOutputConsumer(
        ros::NodeHandle& nh,
        ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor);

    std::string name() const override {
        return "Px4ServiceOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;
    void initializeClientsIfNeeded();

   private:
    ros::NodeHandle& nh_;
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    bool clients_initialized_{false};
};

}  // namespace px4_multirotor_controller
