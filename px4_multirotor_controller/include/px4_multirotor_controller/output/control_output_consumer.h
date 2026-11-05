#pragma once

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <string>

#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

class ControlOutputConsumer final {
   public:
    ControlOutputConsumer(ros::NodeHandle& nh,
                          ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
                          DroneController& controller, uint32_t queue_size);

    bool handle(const FlightEffect& effect);

   private:
    mavros_msgs::PositionTarget makeSetpointMessage(const Setpoint& setpoint) const;
    mavros_msgs::AttitudeTarget makeAttitudeRateMessage(const AttitudeRateTarget& target) const;

    DroneController& controller_;
    ros::Publisher setpoint_raw_pub_;
    ros::Publisher attitude_target_pub_;
};

}  // namespace px4_multirotor_controller
