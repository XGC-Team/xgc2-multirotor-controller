#include "multirotor_reference_trajectory/output/reference_output_consumer.h"

#include <cmath>
#include <memory>
#include <utility>

namespace multirotor_reference_trajectory {
namespace {

template <typename Message>
std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>> makePublishTask(
    std::string name, const ros::Publisher& pub, Message msg) {
    return std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
        std::move(name),
        [pub, msg = std::move(msg)](ros::NodeHandle&) mutable { pub.publish(msg); });
}

}  // namespace

ReferenceOutputConsumer::ReferenceOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    ReferenceTrajectoryRuntime& runtime, const std::string& status_topic,
    const std::string& active_analytic_topic, const std::string& active_polynomial_topic,
    const std::string& active_sampled_topic, const std::string& reference_path_topic,
    double reference_path_sample_dt, double reference_path_preview_duration, uint32_t queue_size)
    : executor_(executor),
      runtime_(runtime),
      reference_path_sample_dt_(reference_path_sample_dt > 0.0 ? reference_path_sample_dt : 0.5),
      reference_path_preview_duration_(
          reference_path_preview_duration > 0.0 ? reference_path_preview_duration : 20.0) {
    status_pub_ = nh.advertise<ReferenceStatus>(status_topic, queue_size, true);
    active_analytic_pub_ = nh.advertise<AnalyticReference>(active_analytic_topic, queue_size, true);
    active_polynomial_pub_ =
        nh.advertise<ActivePolynomialReference>(active_polynomial_topic, queue_size, true);
    active_sampled_pub_ = nh.advertise<SampledReference>(active_sampled_topic, queue_size, true);
    reference_path_pub_ = nh.advertise<nav_msgs::Path>(reference_path_topic, queue_size, true);
}

bool ReferenceOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id == output_event_type::PUBLISH_STATUS) {
        executor_.pushTask(
            makePublishTask("PublishReferenceStatus", status_pub_,
                            runtime_.makeStatus(event.timestamp > 0.0 ? event.timestamp
                                                                      : ros::Time::now().toSec())));
        return true;
    }
    if (event.id == output_event_type::PUBLISH_ACTIVE_ANALYTIC) {
        executor_.pushTask(makePublishTask("PublishActiveAnalytic", active_analytic_pub_,
                                           runtime_.activeAnalyticMessage()));
        publishReferencePath(event.timestamp);
        return true;
    }
    if (event.id == output_event_type::PUBLISH_ACTIVE_POLYNOMIAL) {
        executor_.pushTask(makePublishTask("PublishActivePolynomial", active_polynomial_pub_,
                                           runtime_.activePolynomialMessage()));
        publishReferencePath(event.timestamp);
        return true;
    }
    if (event.id == output_event_type::PUBLISH_ACTIVE_SAMPLED) {
        executor_.pushTask(makePublishTask("PublishActiveSampled", active_sampled_pub_,
                                           runtime_.activeSampledMessage()));
        publishReferencePath(event.timestamp);
        return true;
    }
    return false;
}

nav_msgs::Path ReferenceOutputConsumer::makeReferencePath(double stamp_sec) const {
    nav_msgs::Path path;
    path.header.stamp = ros::Time(stamp_sec > 0.0 ? stamp_sec : ros::Time::now().toSec());
    path.header.frame_id = "world";

    const trajectory::TrajectoryEvaluator3* evaluator = runtime_.evaluator();
    if (evaluator == nullptr || evaluator->duration() <= 0.0 ||
        !std::isfinite(evaluator->duration())) {
        return path;
    }

    const double duration = std::min(evaluator->duration(), reference_path_preview_duration_);
    const int sample_count =
        std::max(2, static_cast<int>(std::ceil(duration / reference_path_sample_dt_)) + 1);
    path.poses.reserve(static_cast<size_t>(sample_count));
    for (int i = 0; i < sample_count; ++i) {
        const double t = std::min(duration, static_cast<double>(i) * reference_path_sample_dt_);
        trajectory::FlatOutput3 flat;
        if (!evaluator->evaluate(t, flat)) {
            continue;
        }
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = flat.position.x();
        pose.pose.position.y = flat.position.y();
        pose.pose.position.z = flat.position.z();
        pose.pose.orientation.z = std::sin(0.5 * flat.yaw);
        pose.pose.orientation.w = std::cos(0.5 * flat.yaw);
        path.poses.push_back(pose);
    }
    return path;
}

void ReferenceOutputConsumer::publishReferencePath(double stamp_sec) {
    executor_.pushTask(
        makePublishTask("PublishReferencePath", reference_path_pub_, makeReferencePath(stamp_sec)));
}

}  // namespace multirotor_reference_trajectory
