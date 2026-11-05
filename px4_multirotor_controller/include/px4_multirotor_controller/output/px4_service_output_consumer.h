#pragma once

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <string>

#include "px4_multirotor_controller/uav/flight_effect.h"

namespace px4_multirotor_controller {

class Px4ServiceOutputConsumer final {
   public:
    Px4ServiceOutputConsumer(
        ros::NodeHandle& nh,
        ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor);

    bool handle(const FlightEffect& effect);
    void initializeClientsIfNeeded();

   private:
    ros::NodeHandle& nh_;
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    bool clients_initialized_{false};
};

}  // namespace px4_multirotor_controller
