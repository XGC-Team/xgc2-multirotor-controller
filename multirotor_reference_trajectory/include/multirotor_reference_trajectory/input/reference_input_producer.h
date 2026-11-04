#pragma once

#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <ros/ros.h>

#include <functional>
#include <string>

#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"
#include "std_msgs/Empty.h"

namespace multirotor_reference_trajectory {

class ReferenceInputProducer {
   public:
    ReferenceInputProducer(ros::NodeHandle& nh, ReferenceTrajectoryRuntime& runtime,
                           const std::string& analytic_topic, const std::string& sampled_topic,
                           const std::string& reset_topic, uint32_t queue_size,
                           std::function<void()> on_input);

   private:
    void analyticCallback(
        const multirotor_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg);
    void sampledCallback(
        const multirotor_reference_trajectory_msgs::SampledReference::ConstPtr& msg);
    void resetCallback(const std_msgs::Empty::ConstPtr& msg);
    ReferenceTrajectoryRuntime& runtime_;
    std::function<void()> on_input_;
    ros::Subscriber analytic_sub_;
    ros::Subscriber sampled_sub_;
    ros::Subscriber reset_sub_;
};

}  // namespace multirotor_reference_trajectory
