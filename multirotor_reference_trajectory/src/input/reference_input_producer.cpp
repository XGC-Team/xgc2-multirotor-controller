#include "multirotor_reference_trajectory/input/reference_input_producer.h"

#include <utility>

namespace multirotor_reference_trajectory {

ReferenceInputProducer::ReferenceInputProducer(ros::NodeHandle& nh,
                                               ReferenceTrajectoryRuntime& runtime,
                                               const std::string& analytic_topic,
                                               const std::string& waypoint_topic,
                                               const std::string& sampled_topic,
                                               const std::string& reset_topic, uint32_t queue_size)
    : runtime_(runtime) {
    analytic_sub_ =
        nh.subscribe(analytic_topic, queue_size, &ReferenceInputProducer::analyticCallback, this);
    waypoint_sub_ =
        nh.subscribe(waypoint_topic, queue_size, &ReferenceInputProducer::waypointCallback, this);
    sampled_sub_ =
        nh.subscribe(sampled_topic, queue_size, &ReferenceInputProducer::sampledCallback, this);
    reset_sub_ =
        nh.subscribe(reset_topic, queue_size, &ReferenceInputProducer::resetCallback, this);
}

void ReferenceInputProducer::analyticCallback(const AnalyticReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null analytic reference");
        return;
    }
    if (!runtime_.acceptAnalytic(*msg)) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected analytic reference");
        return;
    }
    post(event_type::ANALYTIC_RECEIVED, "analytic_reference");
}

void ReferenceInputProducer::waypointCallback(const WaypointReferenceRequest::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null waypoint reference");
        return;
    }
    if (!runtime_.acceptWaypoint(*msg)) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected waypoint request");
        return;
    }
    post(event_type::WAYPOINT_RECEIVED, "waypoint_reference");
}

void ReferenceInputProducer::sampledCallback(const SampledReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null sampled reference");
        return;
    }
    if (!runtime_.acceptSampled(*msg)) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected sampled reference");
        return;
    }
    post(event_type::SAMPLED_RECEIVED, "sampled_reference");
}

void ReferenceInputProducer::resetCallback(const std_msgs::Empty::ConstPtr& msg) {
    (void)msg;
    runtime_.reset();
    post(event_type::RESET_REQUESTED, "reset");
}

void ReferenceInputProducer::post(uint32_t event_id, const char* source) {
    ::state_machine::Event event(event_id,
                                 ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = source;
    const auto status = runtime_.postEvent(std::move(event));
    if (!status.ok()) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Failed to post event %u from %s: %s",
                          event_id, source, status.message.c_str());
    }
}

}  // namespace multirotor_reference_trajectory
