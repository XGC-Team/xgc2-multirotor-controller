#include "multirotor_reference_trajectory/multirotor_reference_trajectory_node.h"

#include <algorithm>
#include <cmath>

namespace multirotor_reference_trajectory {
ReferenceTrajectoryNode::ReferenceTrajectoryNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~") {
    loadParams();
    runtime_.setConfig(config_);
    output_consumer_ = std::make_unique<ReferenceOutputConsumer>(
        nh_, output_worker_, runtime_, status_topic_, active_analytic_topic_,
        active_sampled_topic_, reference_path_topic_, reference_path_sample_dt_,
        reference_path_preview_duration_, queue_size_);
    turn_.emplace(*nh_.getCallbackQueue(), [this] { runTurn(); });
    deadline_.emplace(nh_, [this] { requestTurn(); });
    input_producer_ = std::make_unique<ReferenceInputProducer>(
        nh_, runtime_, analytic_topic_, sampled_topic_, reset_topic_, queue_size_,
        [this] { requestTurn(); });
    output_worker_.start();
    requestTurn();
    ROS_INFO(
        "[ReferenceTrajectoryNode] Initialized: analytic=%s sampled=%s "
        "status=%s active_analytic=%s active_sampled=%s reference_path=%s",
        analytic_topic_.c_str(), sampled_topic_.c_str(), status_topic_.c_str(),
        active_analytic_topic_.c_str(), active_sampled_topic_.c_str(),
        reference_path_topic_.c_str());
}

ReferenceTrajectoryNode::~ReferenceTrajectoryNode() {
    deadline_->cancel();
    turn_->cancel();
    output_worker_.stop();
}

void ReferenceTrajectoryNode::run() {
    ros::spin();
}

void ReferenceTrajectoryNode::requestTurn() {
    turn_->request();
}

void ReferenceTrajectoryNode::runTurn() {
    turn_->beginTurn();
    const double now = ros::Time::now().toSec();
    runtime_.update(now);
    for (const auto& effect : runtime_.effects())
        output_consumer_->handle(effect);
    armDeadline(now);
}

void ReferenceTrajectoryNode::armDeadline(double now) {
    const auto next = runtime_.nextWake();
    if (!next) {
        deadline_->cancel();
        return;
    }
    if (*next <= now) {
        requestTurn();
        return;
    }
    deadline_->arm(*next, [next, now] {
        const auto wait_ns = static_cast<int64_t>(
            std::ceil((static_cast<long double>(*next) - static_cast<long double>(now)) * 1e9L));
        ros::Duration delay;
        delay.fromNSec(wait_ns);
        return delay;
    });
}

void ReferenceTrajectoryNode::loadParams() {
    int queue_size = static_cast<int>(queue_size_);
    private_nh_.param("queue_size", queue_size, queue_size);
    queue_size_ = static_cast<uint32_t>(std::max(1, queue_size));
    private_nh_.param("analytic_topic", analytic_topic_, analytic_topic_);
    private_nh_.param("sampled_topic", sampled_topic_, sampled_topic_);
    private_nh_.param("reset_topic", reset_topic_, reset_topic_);
    private_nh_.param("status_topic", status_topic_, status_topic_);
    private_nh_.param("active_analytic_topic", active_analytic_topic_, active_analytic_topic_);
    private_nh_.param("active_sampled_topic", active_sampled_topic_, active_sampled_topic_);
    private_nh_.param("reference_path_topic", reference_path_topic_, reference_path_topic_);
    private_nh_.param("reference_path_sample_dt", reference_path_sample_dt_,
                      reference_path_sample_dt_);
    if (!std::isfinite(reference_path_sample_dt_) || reference_path_sample_dt_ <= 0.0) {
        reference_path_sample_dt_ = 0.02;
    }
    private_nh_.param("reference_path_preview_duration", reference_path_preview_duration_,
                      reference_path_preview_duration_);
    if (!std::isfinite(reference_path_preview_duration_) ||
        reference_path_preview_duration_ <= 0.0) {
        reference_path_preview_duration_ = 20.0;
    }

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

}  // namespace multirotor_reference_trajectory
