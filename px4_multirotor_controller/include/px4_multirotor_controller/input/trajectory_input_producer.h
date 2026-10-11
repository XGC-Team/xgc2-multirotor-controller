#pragma once

#include <hover_thrust_estimator_msgs/HoverThrustEstimate.h>
#include <mavros_msgs/PositionTarget.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <ros/ros.h>

#include <functional>
#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

class TrajectoryInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    TrajectoryInputProducer(ros::NodeHandle& nh, DroneController& controller,
                            SensorData& sensor_data, EventSink event_sink, uint32_t queue_size);

   private:
    void algSetpointCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
    void activeAnalyticCallback(
        const multirotor_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg);
    void activeSampledCallback(
        const multirotor_reference_trajectory_msgs::SampledReference::ConstPtr& msg);
    void hoverThrustCallback(const hover_thrust_estimator_msgs::HoverThrustEstimate::ConstPtr& msg);
    void postInputEvent(::state_machine::EventId event_id, const char* source);

    DroneController& controller_;
    SensorData& sensor_data_;
    EventSink event_sink_;
    ros::Subscriber alg_setpoint_sub_;
    ros::Subscriber active_analytic_sub_;
    ros::Subscriber active_sampled_sub_;
    ros::Subscriber hover_thrust_sub_;
};

}  // namespace px4_multirotor_controller
