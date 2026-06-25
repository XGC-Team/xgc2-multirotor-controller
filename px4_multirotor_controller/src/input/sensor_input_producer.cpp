#include "px4_multirotor_controller/input/sensor_input_producer.h"

#include <ros/ros.h>

#include <utility>

namespace px4_multirotor_controller {

SensorInputProducer::SensorInputProducer(ros::NodeHandle& nh, SensorData& sensor_data,
                                         ros1_utils::PositionQualityStats& vrpn_quality_stats,
                                         uint32_t queue_size, EventSink event_sink,
                                         std::function<void()> on_state_message)
    : nh_(nh),
      sensor_data_(sensor_data),
      vrpn_quality_stats_(vrpn_quality_stats),
      queue_size_(queue_size),
      event_sink_(std::move(event_sink)),
      on_state_message_(std::move(on_state_message)),
      stats_manager_(nh) {}

void SensorInputProducer::setVrpnQualityConfig(const ros1_utils::PositionQualityConfig& config) {
    vrpn_quality_detector_.setConfig(config);
}

void SensorInputProducer::setStateEstimateTopic(std::string state_estimate_topic) {
    if (started_) {
        ROS_WARN(
            "[SensorInputProducer] Ignoring UAV state estimate topic change "
            "after start");
        return;
    }
    if (!state_estimate_topic.empty()) {
        state_estimate_topic_ = std::move(state_estimate_topic);
    }
}

void SensorInputProducer::setVrpnTopics(std::string pose_topic, std::string twist_topic) {
    if (started_) {
        ROS_WARN("[SensorInputProducer] Ignoring VRPN topic change after start");
        return;
    }
    if (!pose_topic.empty()) {
        vrpn_pose_topic_ = std::move(pose_topic);
    }
    if (!twist_topic.empty()) {
        vrpn_twist_topic_ = std::move(twist_topic);
    }
}

void SensorInputProducer::start() {
    if (started_) {
        return;
    }

    stats_manager_.register_topic<estimator_vrpn_px4_rotor_state::RigidStateEstimate>(
        nh_, state_estimate_topic_, queue_size_, &SensorInputProducer::stateEstimateCallback, this,
        &sensor_data_.uav_state_estimate_stats);
    stats_manager_.register_topic<geometry_msgs::PoseStamped>(
        nh_, "mavros/local_position/pose", queue_size_, &SensorInputProducer::localPosCallback,
        this, &sensor_data_.local_pos_stats);
    stats_manager_.register_topic<geometry_msgs::TwistStamped>(
        nh_, "mavros/local_position/velocity_local", queue_size_,
        &SensorInputProducer::velocityCallback, this, &sensor_data_.local_velocity_stats);
    stats_manager_.register_topic<sensor_msgs::Imu>(nh_, "mavros/imu/data", queue_size_,
                                                    &SensorInputProducer::imuCallback, this,
                                                    &sensor_data_.imu_stats);
    stats_manager_.register_topic<mavros_msgs::State>(nh_, "mavros/state", queue_size_,
                                                      &SensorInputProducer::stateCallback, this,
                                                      &sensor_data_.state_stats);
    stats_manager_.register_topic<sensor_msgs::BatteryState>(nh_, "mavros/battery", queue_size_,
                                                             &SensorInputProducer::batteryCallback,
                                                             this, &sensor_data_.battery_stats);
    stats_manager_.register_topic<geometry_msgs::PoseStamped>(
        nh_, vrpn_pose_topic_, queue_size_, &SensorInputProducer::vrpnPoseCallback, this,
        &sensor_data_.vrpn_pose_stats, &vrpn_quality_stats_);
    stats_manager_.register_topic<geometry_msgs::TwistStamped>(
        nh_, vrpn_twist_topic_, queue_size_, &SensorInputProducer::vrpnTwistCallback, this,
        &sensor_data_.vrpn_twist_stats);

    stats_manager_.start();
    started_ = true;
}

void SensorInputProducer::resetNewFlags() {
    stats_manager_.resetNewFlags();
}

void SensorInputProducer::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    sensor_data_.local_x = msg->pose.position.x;
    sensor_data_.local_y = msg->pose.position.y;
    sensor_data_.local_z = msg->pose.position.z;
    sensor_data_.local_qx = msg->pose.orientation.x;
    sensor_data_.local_qy = msg->pose.orientation.y;
    sensor_data_.local_qz = msg->pose.orientation.z;
    sensor_data_.local_qw = msg->pose.orientation.w;
    postInputEvent(event_type::INPUT_LOCAL_POSITION_UPDATED, "mavros/local_position/pose");
}

void SensorInputProducer::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    sensor_data_.local_vx = msg->twist.linear.x;
    sensor_data_.local_vy = msg->twist.linear.y;
    sensor_data_.local_vz = msg->twist.linear.z;
    postInputEvent(event_type::INPUT_LOCAL_VELOCITY_UPDATED,
                   "mavros/local_position/velocity_local");
}

void SensorInputProducer::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    postInputEvent(event_type::INPUT_IMU_UPDATED, "mavros/imu/data");
}

void SensorInputProducer::stateEstimateCallback(
    const estimator_vrpn_px4_rotor_state::RigidStateEstimate::ConstPtr& msg) {
    sensor_data_.x = msg->position.x;
    sensor_data_.y = msg->position.y;
    sensor_data_.z = msg->position.z;
    sensor_data_.vx = msg->velocity.x;
    sensor_data_.vy = msg->velocity.y;
    sensor_data_.vz = msg->velocity.z;
    sensor_data_.qx = msg->orientation.x;
    sensor_data_.qy = msg->orientation.y;
    sensor_data_.qz = msg->orientation.z;
    sensor_data_.qw = msg->orientation.w;
    sensor_data_.wx = msg->angular_velocity.x;
    sensor_data_.wy = msg->angular_velocity.y;
    sensor_data_.wz = msg->angular_velocity.z;
    sensor_data_.uav_state_estimator_state = msg->estimator_state;
    sensor_data_.uav_state_estimator_flags = msg->flags;
    sensor_data_.uav_state_estimate_stamp = msg->header.stamp.toSec();
    postInputEvent(event_type::INPUT_UAV_STATE_ESTIMATE_UPDATED, "alg/state_estimator/state");
}

void SensorInputProducer::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    if (on_state_message_) {
        on_state_message_();
    }

    sensor_data_.fcu_connected = msg->connected;
    sensor_data_.fcu_armed = msg->armed;
    sensor_data_.fcu_guided = msg->guided;
    sensor_data_.fcu_manual_input = msg->manual_input;
    sensor_data_.fcu_mode = msg->mode;
    sensor_data_.fcu_system_status = msg->system_status;
    postInputEvent(event_type::INPUT_FCU_STATE_UPDATED, "mavros/state");
}

void SensorInputProducer::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    sensor_data_.battery_percentage = msg->percentage;
    postInputEvent(event_type::INPUT_BATTERY_UPDATED, "mavros/battery");
}

void SensorInputProducer::vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    sensor_data_.vrpn_x = msg->pose.position.x;
    sensor_data_.vrpn_y = msg->pose.position.y;
    sensor_data_.vrpn_z = msg->pose.position.z;
    sensor_data_.vrpn_qx = msg->pose.orientation.x;
    sensor_data_.vrpn_qy = msg->pose.orientation.y;
    sensor_data_.vrpn_qz = msg->pose.orientation.z;
    sensor_data_.vrpn_qw = msg->pose.orientation.w;
    vrpn_quality_stats_ = vrpn_quality_detector_.process(sensor_data_.vrpn_x, sensor_data_.vrpn_y,
                                                         sensor_data_.vrpn_z);
    postInputEvent(event_type::INPUT_VRPN_POSE_UPDATED, "pose");
}

void SensorInputProducer::vrpnTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    sensor_data_.vrpn_vx = msg->twist.linear.x;
    sensor_data_.vrpn_vy = msg->twist.linear.y;
    sensor_data_.vrpn_vz = msg->twist.linear.z;
    sensor_data_.vrpn_wx = msg->twist.angular.x;
    sensor_data_.vrpn_wy = msg->twist.angular.y;
    sensor_data_.vrpn_wz = msg->twist.angular.z;
    postInputEvent(event_type::INPUT_VRPN_TWIST_UPDATED, "twist");
}

void SensorInputProducer::postInputEvent(::state_machine::EventId event_id, const char* source) {
    if (!event_sink_) {
        ROS_ERROR("[SensorInputProducer] Event sink is not configured");
        return;
    }

    ::state_machine::Event event(event_id,
                                 ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = source;
    const auto status = event_sink_(std::move(event));
    if (!status.ok()) {
        ROS_ERROR_THROTTLE(1.0,
                           "[SensorInputProducer] Failed to post input event %u "
                           "from %s: %s",
                           event_id, source, status.message.c_str());
    }
}

}  // namespace px4_multirotor_controller
