#include "px4_multirotor_controller/output/control_output_consumer.h"

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"

namespace px4_multirotor_controller {

ControlOutputConsumer::ControlOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    DroneController& controller, uint32_t queue_size)
    : controller_(controller) {
    // Keep the constructor contract, but never put control samples behind blocking RPCs.
    (void)executor;
    setpoint_raw_pub_ =
        nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", queue_size);
    attitude_target_pub_ =
        nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", queue_size);
}

bool ControlOutputConsumer::handle(const FlightEffect& effect) {
    if (std::holds_alternative<FlightSetpoint>(effect)) {
        // The ROS bounded transport queue owns delivery; blocking RPCs do not.
        setpoint_raw_pub_.publish(makeSetpointMessage(controller_.getSetpoint()));
        return true;
    }
    if (std::holds_alternative<FlightAttitude>(effect)) {
        attitude_target_pub_.publish(makeAttitudeRateMessage(controller_.getAttitudeRateTarget()));
        return true;
    }
    return false;
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
