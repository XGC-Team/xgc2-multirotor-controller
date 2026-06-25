#include "multirotor_reference_trajectory/multirotor_reference_trajectory_node.h"

#include <algorithm>
#include <cmath>

namespace multirotor_reference_trajectory {

ReferenceTrajectoryNode::ReferenceTrajectoryNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), output_executor_(nh_) {
    loadParams();
    runtime_.setConfig(config_);
    output_dispatcher_.addConsumer(std::make_unique<ReferenceOutputConsumer>(
        nh_, output_executor_, runtime_, status_topic_, active_analytic_topic_,
        active_polynomial_topic_, active_sampled_topic_, queue_size_));
    input_producer_ = std::make_unique<ReferenceInputProducer>(
        nh_, runtime_, analytic_topic_, waypoint_topic_, sampled_topic_, reset_topic_, queue_size_);
    output_executor_.start();
    ROS_INFO(
        "[ReferenceTrajectoryNode] Initialized: analytic=%s waypoint=%s sampled=%s "
        "status=%s active_analytic=%s active_polynomial=%s active_sampled=%s",
        analytic_topic_.c_str(), waypoint_topic_.c_str(), sampled_topic_.c_str(),
        status_topic_.c_str(), active_analytic_topic_.c_str(), active_polynomial_topic_.c_str(),
        active_sampled_topic_.c_str());
}

ReferenceTrajectoryNode::~ReferenceTrajectoryNode() {
    output_executor_.stop();
}

void ReferenceTrajectoryNode::run(double main_frequency_hz) {
    const double frequency =
        std::isfinite(main_frequency_hz) && main_frequency_hz > 0.0 ? main_frequency_hz : 100.0;
    ROS_INFO("[ReferenceTrajectoryNode] Starting main loop at %.1f Hz", frequency);
    ros::Rate rate(frequency);
    while (ros::ok()) {
        ros::spinOnce();
        runtime_.update(ros::Time::now().toSec());
        dispatchOutputEvents(runtime_.stateMachine().currentOutputEvents());
        rate.sleep();
    }
}

void ReferenceTrajectoryNode::loadParams() {
    int queue_size = static_cast<int>(queue_size_);
    private_nh_.param("queue_size", queue_size, queue_size);
    queue_size_ = static_cast<uint32_t>(std::max(1, queue_size));
    private_nh_.param("analytic_topic", analytic_topic_, analytic_topic_);
    private_nh_.param("waypoint_topic", waypoint_topic_, waypoint_topic_);
    private_nh_.param("sampled_topic", sampled_topic_, sampled_topic_);
    private_nh_.param("reset_topic", reset_topic_, reset_topic_);
    private_nh_.param("status_topic", status_topic_, status_topic_);
    private_nh_.param("active_analytic_topic", active_analytic_topic_, active_analytic_topic_);
    private_nh_.param("active_polynomial_topic", active_polynomial_topic_,
                      active_polynomial_topic_);
    private_nh_.param("active_sampled_topic", active_sampled_topic_, active_sampled_topic_);

    private_nh_.param("status_rate", config_.status_rate_hz, config_.status_rate_hz);
    private_nh_.param("active_publish_rate", config_.active_publish_rate_hz,
                      config_.active_publish_rate_hz);
    private_nh_.param("validation_sample_dt", config_.validation_sample_dt,
                      config_.validation_sample_dt);
    private_nh_.param("trajectory_timeout", config_.trajectory_timeout, config_.trajectory_timeout);
    private_nh_.param("min_lead_time", config_.min_lead_time, config_.min_lead_time);
    private_nh_.param("max_velocity", config_.limits.max_velocity, config_.limits.max_velocity);
    private_nh_.param("max_acceleration", config_.limits.max_acceleration,
                      config_.limits.max_acceleration);
    private_nh_.param("max_jerk", config_.limits.max_jerk, config_.limits.max_jerk);
    private_nh_.param("max_snap", config_.limits.max_snap, config_.limits.max_snap);
    private_nh_.param("min_specific_thrust", config_.limits.min_specific_thrust,
                      config_.limits.min_specific_thrust);
}

void ReferenceTrajectoryNode::dispatchOutputEvents(
    const std::vector<::state_machine::Event>& events) {
    const auto result = output_dispatcher_.dispatch(events);
    for (const auto& event : result.unhandled_events) {
        ROS_WARN("[ReferenceTrajectoryNode] Unhandled output event id: %u",
                 static_cast<unsigned>(event.id));
    }
    for (const auto& failure : result.failures) {
        ROS_WARN("[ReferenceTrajectoryNode] Output consumer '%s' failed on event %u: %s",
                 failure.consumer_name.c_str(), static_cast<unsigned>(failure.event.id),
                 failure.message.c_str());
    }
}

}  // namespace multirotor_reference_trajectory
