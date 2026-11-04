#include "multirotor_reference_trajectory/input/reference_input_producer.h"

#include <utility>

#include "multirotor_reference_trajectory/ros_reference_conversion.h"

namespace multirotor_reference_trajectory {

ReferenceInputProducer::ReferenceInputProducer(ros::NodeHandle& nh,
                                               ReferenceTrajectoryRuntime& runtime,
                                               const std::string& analytic_topic,
                                               const std::string& sampled_topic,
                                               const std::string& reset_topic, uint32_t queue_size,
                                               std::function<void()> on_input)
    : runtime_(runtime), on_input_(std::move(on_input)) {
    analytic_sub_ =
        nh.subscribe(analytic_topic, queue_size, &ReferenceInputProducer::analyticCallback, this);
    sampled_sub_ =
        nh.subscribe(sampled_topic, queue_size, &ReferenceInputProducer::sampledCallback, this);
    reset_sub_ =
        nh.subscribe(reset_topic, queue_size, &ReferenceInputProducer::resetCallback, this);
}

void ReferenceInputProducer::analyticCallback(
    const multirotor_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null analytic reference");
        return;
    }
    if (!runtime_.acceptAnalytic(toCore(*msg), ros::Time::now().toSec())) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected analytic reference");
        return;
    }
    on_input_();
}

void ReferenceInputProducer::sampledCallback(
    const multirotor_reference_trajectory_msgs::SampledReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null sampled reference");
        return;
    }
    if (!runtime_.acceptSampled(toCore(*msg), ros::Time::now().toSec())) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected sampled reference");
        return;
    }
    on_input_();
}

void ReferenceInputProducer::resetCallback(const std_msgs::Empty::ConstPtr& msg) {
    (void)msg;
    runtime_.reset();
    on_input_();
}

}  // namespace multirotor_reference_trajectory
