#pragma once

#include <hover_thrust_estimator_msgs/HoverThrustEstimate.h>
#include <mavros_msgs/PositionTarget.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <ros/ros.h>
#include <functional>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

class TrajectoryInputProducer {
   public:
    TrajectoryInputProducer(ros::NodeHandle& nh, DroneController& controller,
                            SensorData& sensor_data, uint32_t queue_size,
                            std::function<void()> on_input = {});

   private:
    void algSetpointCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void activeAnalyticCallback(
        const multirotor_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg);
    void activeSampledCallback(
        const multirotor_reference_trajectory_msgs::SampledReference::ConstPtr& msg);
    void hoverThrustCallback(const hover_thrust_estimator_msgs::HoverThrustEstimate::ConstPtr& msg);

    DroneController& controller_;
    SensorData& sensor_data_;
    std::function<void()> on_input_;
    ros::Subscriber alg_setpoint_sub_;
    ros::Subscriber active_analytic_sub_;
    ros::Subscriber active_sampled_sub_;
    ros::Subscriber hover_thrust_sub_;
};

}  // namespace px4_multirotor_controller
