#pragma once

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>

#include <string>

#include "px4_multirotor_controller/output/controller_output_worker.h"
#include "px4_multirotor_controller/uav/flight_effect.h"

namespace px4_multirotor_controller {

class Px4ServiceOutputConsumer final {
   public:
    Px4ServiceOutputConsumer(ros::NodeHandle& nh, ControllerOutputWorker& worker);

    bool handle(const FlightEffect& effect);
    void initializeClientsIfNeeded();

   private:
    ros::NodeHandle& nh_;
    ControllerOutputWorker& worker_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    bool clients_initialized_{false};
};

}  // namespace px4_multirotor_controller
