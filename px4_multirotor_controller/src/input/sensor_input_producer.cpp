#include "px4_multirotor_controller/input/sensor_input_producer.h"

#include <ros/ros.h>

#include "px4_multirotor_controller/common/state_estimate_status.h"
#include "px4_multirotor_controller/ros_time_conversion.h"

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

void SensorInputProducer::setVrpnPoseTopic(std::string pose_topic) {
    if (started_) {
        ROS_WARN("[SensorInputProducer] Ignoring VRPN pose topic change after start");
        return;
    }
    if (!pose_topic.empty()) {
        vrpn_pose_topic_ = std::move(pose_topic);
    }
}

void SensorInputProducer::start() {
    if (started_) {
        return;
    }

    stats_manager_.register_topic<rigid_state_estimator_msgs::RigidStateEstimate>(
        nh_, state_estimate_topic_, queue_size_, &SensorInputProducer::stateEstimateCallback, this,
        &ros_stats_.state_estimate);
    stats_manager_.register_topic<geometry_msgs::PoseStamped>(
        nh_, "mavros/local_position/pose", queue_size_, &SensorInputProducer::localPosCallback,
        this, &ros_stats_.local_pos);
    stats_manager_.register_topic<geometry_msgs::TwistStamped>(
        nh_, "mavros/local_position/velocity_local", queue_size_,
        &SensorInputProducer::velocityCallback, this, &ros_stats_.local_velocity);
    stats_manager_.register_topic<sensor_msgs::Imu>(nh_, "mavros/imu/data", queue_size_,
                                                    &SensorInputProducer::imuCallback, this,
                                                    &ros_stats_.imu);
    stats_manager_.register_topic<mavros_msgs::State>(nh_, "mavros/state", queue_size_,
                                                      &SensorInputProducer::stateCallback, this,
                                                      &ros_stats_.state);
    stats_manager_.register_topic<sensor_msgs::BatteryState>(nh_, "mavros/battery", queue_size_,
                                                             &SensorInputProducer::batteryCallback,
                                                             this, &ros_stats_.battery);
    stats_manager_.register_topic<geometry_msgs::PoseStamped>(
        nh_, vrpn_pose_topic_, queue_size_, &SensorInputProducer::vrpnPoseCallback, this,
        &ros_stats_.vrpn_pose, &vrpn_quality_stats_);
    stats_manager_.start();
    started_ = true;
}

void SensorInputProducer::resetNewFlags() {
    stats_manager_.resetNewFlags();
}

namespace {

void copyStats(const ros1_utils::TopicStats& from, SensorData::TopicStats& to) {
    to.frequency_hz = from.frequency_hz;
    to.dt_max = from.dt_max;
    to.time_since_last_msg = from.time_since_last_msg;
    to.jitter = from.jitter;
    to.last_message_time = toCoreTime(from.last_message_time);
    to.is_active = from.is_active;
    to.is_new = from.is_new;
}

using Rse = rigid_state_estimator_msgs::RigidStateEstimate;
static_assert(state_estimate::STATE_SELF_CHECK == Rse::STATE_SELF_CHECK, "estimator state");
static_assert(state_estimate::STATE_INITIALIZING == Rse::STATE_INITIALIZING, "estimator state");
static_assert(state_estimate::STATE_RUNNING == Rse::STATE_RUNNING, "estimator state");
static_assert(state_estimate::STATE_COASTING == Rse::STATE_COASTING, "estimator state");
static_assert(state_estimate::STATE_FAULT == Rse::STATE_FAULT, "estimator state");
static_assert(state_estimate::FLAG_IMU_MISSING == Rse::FLAG_IMU_MISSING, "estimator flag");
static_assert(state_estimate::FLAG_VRPN_MISSING == Rse::FLAG_VRPN_MISSING, "estimator flag");
static_assert(state_estimate::FLAG_IMU_STALE == Rse::FLAG_IMU_STALE, "estimator flag");
static_assert(state_estimate::FLAG_VRPN_STALE == Rse::FLAG_VRPN_STALE, "estimator flag");
static_assert(state_estimate::FLAG_IMU_RATE_LOW == Rse::FLAG_IMU_RATE_LOW, "estimator flag");
static_assert(state_estimate::FLAG_VRPN_RATE_LOW == Rse::FLAG_VRPN_RATE_LOW, "estimator flag");
static_assert(state_estimate::FLAG_TIME_JUMP == Rse::FLAG_TIME_JUMP, "estimator flag");
static_assert(state_estimate::FLAG_COASTING == Rse::FLAG_COASTING, "estimator flag");
static_assert(state_estimate::FLAG_FAULT == Rse::FLAG_FAULT, "estimator flag");
static_assert(state_estimate::FLAG_INNOVATION_REJECTED == Rse::FLAG_INNOVATION_REJECTED, "estimator flag");
static_assert(state_estimate::FLAG_EXTRINSIC_UNVERIFIED == Rse::FLAG_EXTRINSIC_UNVERIFIED, "estimator flag");
static_assert(state_estimate::FLAG_COVARIANCE_HIGH == Rse::FLAG_COVARIANCE_HIGH, "estimator flag");
static_assert(state_estimate::FLAG_INVALID_IMU == Rse::FLAG_INVALID_IMU, "estimator flag");
static_assert(state_estimate::FLAG_INVALID_VRPN == Rse::FLAG_INVALID_VRPN, "estimator flag");
static_assert(state_estimate::FLAG_POSE_TIME_ALIGNMENT_REJECTED == Rse::FLAG_POSE_TIME_ALIGNMENT_REJECTED, "estimator flag");
static_assert(state_estimate::FLAG_VRPN_SUSPECTED == Rse::FLAG_VRPN_SUSPECTED, "estimator flag");
static_assert(state_estimate::FLAG_VRPN_FAULT == Rse::FLAG_VRPN_FAULT, "estimator flag");
static_assert(state_estimate::FLAG_VRPN_RECOVERY == Rse::FLAG_VRPN_RECOVERY, "estimator flag");
static_assert(state_estimate::FLAG_FILTER_DEGRADED == Rse::FLAG_FILTER_DEGRADED, "estimator flag");
static_assert(state_estimate::FLAG_FILTER_IMU_ONLY == Rse::FLAG_FILTER_IMU_ONLY, "estimator flag");

}  // namespace

void SensorInputProducer::syncStats() {
    copyStats(ros_stats_.state_estimate, sensor_data_.uav_state_estimate_stats);
    copyStats(ros_stats_.local_pos, sensor_data_.local_pos_stats);
    copyStats(ros_stats_.local_velocity, sensor_data_.local_velocity_stats);
    copyStats(ros_stats_.imu, sensor_data_.imu_stats);
    copyStats(ros_stats_.state, sensor_data_.state_stats);
    copyStats(ros_stats_.battery, sensor_data_.battery_stats);
    copyStats(ros_stats_.vrpn_pose, sensor_data_.vrpn_pose_stats);
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
    const rigid_state_estimator_msgs::RigidStateEstimate::ConstPtr& msg) {
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
    sensor_data_.ax = msg->linear_acceleration.x;
    sensor_data_.ay = msg->linear_acceleration.y;
    sensor_data_.az = msg->linear_acceleration.z;
    sensor_data_.gx = msg->gravity.x;
    sensor_data_.gy = msg->gravity.y;
    sensor_data_.gz = msg->gravity.z;
    sensor_data_.accel_bias_x = msg->accel_bias.x;
    sensor_data_.accel_bias_y = msg->accel_bias.y;
    sensor_data_.accel_bias_z = msg->accel_bias.z;
    sensor_data_.uav_state_estimator_state = msg->estimator_state;
    sensor_data_.uav_state_estimator_flags = msg->flags;
    sensor_data_.uav_state_estimate_stamp = msg->header.stamp.toSec();
    sensor_data_.uav_state_filter_inertial_stamp = msg->filter_inertial_stamp_sec;
    sensor_data_.uav_state_filter_pose_stamp = msg->filter_pose_stamp_sec;
    sensor_data_.uav_state_last_vrpn_pose_stamp = msg->last_vrpn_pose_stamp_sec;
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
