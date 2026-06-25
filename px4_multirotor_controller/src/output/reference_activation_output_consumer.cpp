#include "px4_multirotor_controller/output/reference_activation_output_consumer.h"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {
namespace {

template <typename Message>
std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>> makePublishTask(
    std::string name, const ros::Publisher& pub, Message msg) {
    return std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
        std::move(name),
        [pub, msg = std::move(msg)](ros::NodeHandle&) mutable { pub.publish(msg); });
}

double finiteOr(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

}  // namespace

ReferenceActivationOutputConsumer::ReferenceActivationOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    DroneController& controller, uint32_t queue_size)
    : executor_(executor), controller_(controller) {
    activation_pub_ = nh.advertise<multirotor_reference_trajectory::AnalyticReference>(
        "alg/multirotor_reference_trajectory/request/analytic", queue_size);
}

bool ReferenceActivationOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION) {
        return false;
    }

    const auto msg =
        makeActivationMessage(event, controller_.getSensorData(), controller_.getConfig());
    ROS_INFO(
        "[ReferenceActivationOutputConsumer] Activating UAV reference at "
        "t=%.3f p=[%.3f %.3f %.3f] id=%u rev=%u",
        msg.start_time.toSec(), msg.origin.position.x, msg.origin.position.y, msg.origin.position.z,
        msg.trajectory_id, msg.revision);
    executor_.pushTask(
        makePublishTask("PublishReferenceTrajectoryActivation", activation_pub_, msg));
    return true;
}

multirotor_reference_trajectory::AnalyticReference
ReferenceActivationOutputConsumer::makeActivationMessage(const ::state_machine::Event& event,
                                                         const SensorData& sensor,
                                                         const ControllerConfig& config) {
    multirotor_reference_trajectory::AnalyticReference msg;
    const double stamp = event.timestamp > 0.0 ? event.timestamp : ros::Time::now().toSec();
    msg.header.stamp = ros::Time(stamp);
    msg.header.frame_id = "map";
    msg.request_id = ++request_id_;
    msg.trajectory_id = ++trajectory_id_;
    msg.revision = ++revision_;
    msg.analytic_type = multirotor_reference_trajectory::AnalyticReference::ANALYTIC_CIRCLE_ENTRY;
    msg.flags = 0U;
    msg.start_time = ros::Time(stamp + config.nmpc.reference_start_delay);
    msg.duration = config.nmpc.reference_duration;

    msg.origin.position.x = finiteOr(sensor.x, 0.0);
    msg.origin.position.y = finiteOr(sensor.y, 0.0);
    msg.origin.position.z = finiteOr(sensor.z, config.nmpc.reference_height);
    msg.origin.orientation.x = finiteOr(sensor.qx, 0.0);
    msg.origin.orientation.y = finiteOr(sensor.qy, 0.0);
    msg.origin.orientation.z = finiteOr(sensor.qz, 0.0);
    msg.origin.orientation.w = finiteOr(sensor.qw, 1.0);

    const double q_norm = std::sqrt(msg.origin.orientation.x * msg.origin.orientation.x +
                                    msg.origin.orientation.y * msg.origin.orientation.y +
                                    msg.origin.orientation.z * msg.origin.orientation.z +
                                    msg.origin.orientation.w * msg.origin.orientation.w);
    if (!std::isfinite(q_norm) || q_norm < 1e-9) {
        msg.origin.orientation.x = 0.0;
        msg.origin.orientation.y = 0.0;
        msg.origin.orientation.z = 0.0;
        msg.origin.orientation.w = 1.0;
    } else {
        msg.origin.orientation.x /= q_norm;
        msg.origin.orientation.y /= q_norm;
        msg.origin.orientation.z /= q_norm;
        msg.origin.orientation.w /= q_norm;
    }

    msg.params = {config.nmpc.reference_radius,      config.nmpc.reference_line_speed,
                  config.nmpc.reference_height,      config.nmpc.reference_z_amplitude,
                  config.nmpc.reference_z_frequency, config.nmpc.reference_entry_duration,
                  finiteOr(sensor.x, 0.0),           finiteOr(sensor.y, 0.0)};

    return msg;
}

}  // namespace px4_multirotor_controller
