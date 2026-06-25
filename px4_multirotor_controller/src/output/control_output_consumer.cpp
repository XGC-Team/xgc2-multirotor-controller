#include "px4_multirotor_controller/output/control_output_consumer.h"

#include <memory>
#include <string>
#include <utility>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"

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

ControlOutputConsumer::ControlOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    DroneController& controller, uint32_t queue_size)
    : executor_(executor), controller_(controller) {
    setpoint_raw_pub_ =
        nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", queue_size);
    attitude_target_pub_ =
        nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", queue_size);
}

bool ControlOutputConsumer::handle(const ::state_machine::Event& event) {
    switch (event.id) {
        case output_event_type::PUBLISH_SETPOINT:
            executor_.pushTask(makePublishTask("PublishSetpoint", setpoint_raw_pub_,
                                               makeSetpointMessage(controller_.getSetpoint())));
            return true;
        case output_event_type::PUBLISH_ATTITUDE_RATE_TARGET:
            executor_.pushTask(
                makePublishTask("PublishAttitudeRateTarget", attitude_target_pub_,
                                makeAttitudeRateMessage(controller_.getAttitudeRateTarget())));
            return true;
        default:
            return false;
    }
}

mavros_msgs::PositionTarget ControlOutputConsumer::makeSetpointMessage(
    const Setpoint& setpoint) const {
    mavros_msgs::PositionTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.coordinate_frame = setpoint.coordinate_frame;
    msg.type_mask = setpoint.type_mask;
    msg.position.x = setpoint.x;
    msg.position.y = setpoint.y;
    msg.position.z = setpoint.z;
    msg.velocity.x = setpoint.vx;
    msg.velocity.y = setpoint.vy;
    msg.velocity.z = setpoint.vz;
    msg.acceleration_or_force.x = setpoint.ax;
    msg.acceleration_or_force.y = setpoint.ay;
    msg.acceleration_or_force.z = setpoint.az;
    msg.yaw = quaternionToYaw(setpoint.qx, setpoint.qy, setpoint.qz, setpoint.qw);
    msg.yaw_rate = setpoint.yaw_rate;
    return msg;
}

mavros_msgs::AttitudeTarget ControlOutputConsumer::makeAttitudeRateMessage(
    const AttitudeRateTarget& target) const {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    msg.orientation.w = 1.0;
    msg.body_rate.x = target.body_rate_x;
    msg.body_rate.y = target.body_rate_y;
    msg.body_rate.z = target.body_rate_z;
    msg.thrust = clamp(target.thrust, 0.0, 1.0);
    return msg;
}

}  // namespace px4_multirotor_controller
