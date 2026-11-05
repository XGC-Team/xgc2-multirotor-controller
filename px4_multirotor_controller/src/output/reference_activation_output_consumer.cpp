#include "px4_multirotor_controller/output/reference_activation_output_consumer.h"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/ros_reference_conversion.h"

namespace px4_multirotor_controller {
ReferenceActivationOutputConsumer::ReferenceActivationOutputConsumer(
    ros::NodeHandle& nh, ControllerOutputWorker& worker,
    DroneController& controller, uint32_t queue_size)
    : worker_(worker), controller_(controller) {
    activation_pub_ = nh.advertise<multirotor_reference_trajectory_msgs::AnalyticReference>(
        "alg/multirotor_reference_trajectory/request/analytic", queue_size);
}

bool ReferenceActivationOutputConsumer::handle(const FlightEffect& effect) {
    const auto* activation = std::get_if<FlightReferenceActivation>(&effect);
    if (!activation) return false;

    const double stamp = activation->timestamp > 0.0 ? activation->timestamp : ros::Time::now().toSec();
    auto msg = toRosReference(
        activation_.make(stamp, controller_.getSensorData(), controller_.getConfig()));
    msg.header.frame_id = "map";
    ROS_INFO(
        "[ReferenceActivationOutputConsumer] Activating UAV reference at "
        "t=%.3f p=[%.3f %.3f %.3f] id=%u rev=%u",
        msg.start_time.toSec(), msg.origin.position.x, msg.origin.position.y, msg.origin.position.z,
        msg.trajectory_id, msg.revision);
    worker_.post(OutputReference{activation_pub_, std::move(msg)});
    return true;
}

}  // namespace px4_multirotor_controller
