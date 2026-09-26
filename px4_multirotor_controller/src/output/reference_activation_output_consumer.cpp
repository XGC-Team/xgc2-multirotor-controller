#include "px4_multirotor_controller/output/reference_activation_output_consumer.h"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/ros_reference_conversion.h"

namespace px4_multirotor_controller {
namespace {

template <typename Message>
std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>> makePublishTask(
    std::string name, const ros::Publisher& pub, Message msg) {
    return std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
        std::move(name),
        [pub, msg = std::move(msg)](ros::NodeHandle&) mutable { pub.publish(msg); });
}

}  // namespace

ReferenceActivationOutputConsumer::ReferenceActivationOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    DroneController& controller, uint32_t queue_size)
    : executor_(executor), controller_(controller) {
    activation_pub_ = nh.advertise<multirotor_reference_trajectory_msgs::AnalyticReference>(
        "alg/multirotor_reference_trajectory/request/analytic", queue_size);
}

bool ReferenceActivationOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION) {
        return false;
    }

    const double stamp = event.timestamp > 0.0 ? event.timestamp : ros::Time::now().toSec();
    auto msg = toRosReference(
        activation_.make(stamp, controller_.getSensorData(), controller_.getConfig()));
    msg.header.frame_id = "map";
    ROS_INFO(
        "[ReferenceActivationOutputConsumer] Activating UAV reference at "
        "t=%.3f p=[%.3f %.3f %.3f] id=%u rev=%u",
        msg.start_time.toSec(), msg.origin.position.x, msg.origin.position.y, msg.origin.position.z,
        msg.trajectory_id, msg.revision);
    executor_.pushTask(
        makePublishTask("PublishReferenceTrajectoryActivation", activation_pub_, msg));
    return true;
}

}  // namespace px4_multirotor_controller
