#pragma once

#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <ros/ros.h>

#include "px4_multirotor_controller/output/controller_output_worker.h"
#include <string>

#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/uav/reference_activation.h"

namespace px4_multirotor_controller {

class ReferenceActivationOutputConsumer final {
   public:
    ReferenceActivationOutputConsumer(
        ros::NodeHandle& nh, ControllerOutputWorker& worker,
        DroneController& controller, uint32_t queue_size);

    bool handle(const FlightEffect& effect);

   private:
    ControllerOutputWorker& worker_;
    DroneController& controller_;
    ros::Publisher activation_pub_;
    ReferenceActivation activation_;
};

}  // namespace px4_multirotor_controller
