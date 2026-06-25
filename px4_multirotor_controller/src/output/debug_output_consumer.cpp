#include "px4_multirotor_controller/output/debug_output_consumer.h"

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

}  // namespace

DebugOutputConsumer::DebugOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    DroneController& controller, const SensorData& sensor_data,
    const ros1_utils::PositionQualityStats& vrpn_quality_stats, const bool& debug_print_enabled,
    uint32_t queue_size)
    : executor_(executor),
      controller_(controller),
      sensor_data_(sensor_data),
      vrpn_quality_stats_(vrpn_quality_stats),
      debug_print_enabled_(debug_print_enabled) {
    controller_status_pub_ = nh.advertise<std_msgs::String>("custom/statustext", queue_size);
    events_pub_ = nh.advertise<std_msgs::UInt32MultiArray>("alg/state_machine_events", queue_size);
    stats_state_estimate_pub_ =
        nh.advertise<std_msgs::Float32MultiArray>("alg/stats/state_estimate", queue_size);
    stats_local_pos_pub_ =
        nh.advertise<std_msgs::Float32MultiArray>("alg/stats/local_pos", queue_size);
    stats_local_vel_pub_ =
        nh.advertise<std_msgs::Float32MultiArray>("alg/stats/local_velocity", queue_size);
    stats_imu_pub_ = nh.advertise<std_msgs::Float32MultiArray>("alg/stats/imu", queue_size);
    stats_state_pub_ = nh.advertise<std_msgs::Float32MultiArray>("alg/stats/state", queue_size);
    stats_battery_pub_ = nh.advertise<std_msgs::Float32MultiArray>("alg/stats/battery", queue_size);
    stats_vrpn_pose_pub_ =
        nh.advertise<std_msgs::Float32MultiArray>("alg/stats/vrpn_pose", queue_size);
    stats_vrpn_twist_pub_ =
        nh.advertise<std_msgs::Float32MultiArray>("alg/stats/vrpn_twist", queue_size);
    vrpn_quality_pub_ = nh.advertise<std_msgs::Float32MultiArray>("alg/vrpn_quality", queue_size);
    tracking_error_pub_ =
        nh.advertise<std_msgs::Float32MultiArray>("alg/tracking/position_error", queue_size);
}

bool DebugOutputConsumer::handle(const ::state_machine::Event& event) {
    switch (event.id) {
        case output_event_type::PUBLISH_CONTROLLER_STATUS: {
            std_msgs::String msg;
            msg.data = controller_.getStateMachine().currentStateName(region_type::CONTROL);
            if (msg.data.empty()) {
                msg.data = "Unknown";
            }
            executor_.pushTask(
                makePublishTask("PublishControllerStatus", controller_status_pub_, std::move(msg)));
            return true;
        }
        case output_event_type::PUBLISH_STATE_MACHINE_EVENTS:
            executor_.pushTask(makePublishTask("PublishStateMachineEvents", events_pub_,
                                               snapshotStateMachineEvents()));
            return true;
        case output_event_type::PUBLISH_SENSOR_STATS: {
            const auto snapshot = snapshotSensorStats();
            executor_.pushTask(
                std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
                    "PublishSensorStats",
                    [snapshot, state_estimate_pub = stats_state_estimate_pub_,
                     local_pos_pub = stats_local_pos_pub_, local_vel_pub = stats_local_vel_pub_,
                     imu_pub = stats_imu_pub_, state_pub = stats_state_pub_,
                     battery_pub = stats_battery_pub_, vrpn_pose_pub = stats_vrpn_pose_pub_,
                     vrpn_twist_pub = stats_vrpn_twist_pub_,
                     quality_pub = vrpn_quality_pub_](ros::NodeHandle&) mutable {
                        state_estimate_pub.publish(makeTopicStatsMessage(snapshot.state_estimate));
                        local_pos_pub.publish(makeTopicStatsMessage(snapshot.local_pos));
                        local_vel_pub.publish(makeTopicStatsMessage(snapshot.local_velocity));
                        imu_pub.publish(makeTopicStatsMessage(snapshot.imu));
                        state_pub.publish(makeTopicStatsMessage(snapshot.state));
                        battery_pub.publish(makeTopicStatsMessage(snapshot.battery));
                        vrpn_pose_pub.publish(makeTopicStatsMessage(snapshot.vrpn_pose));
                        vrpn_twist_pub.publish(makeTopicStatsMessage(snapshot.vrpn_twist));
                        quality_pub.publish(makeVrpnQualityMessage(snapshot.vrpn_quality));
                    }));
            return true;
        }
        case output_event_type::PUBLISH_TRACKING_ERROR:
            executor_.pushTask(makePublishTask("PublishTrackingError", tracking_error_pub_,
                                               snapshotTrackingError()));
            return true;
        case output_event_type::PRINT_SENSOR_DEBUG:
            if (debug_print_enabled_) {
                const SensorData snapshot = sensor_data_;
                executor_.pushTask(
                    std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
                        "PrintSensorDebug",
                        [snapshot](ros::NodeHandle&) { printSensorDebug(snapshot); }));
            }
            return true;
        default:
            return false;
    }
}

std_msgs::Float32MultiArray DebugOutputConsumer::snapshotTrackingError() const {
    std_msgs::Float32MultiArray msg;
    msg.data.reserve(10);

    UavReferencePoint reference;
    const ros::Time now = ros::Time::now();
    if (!controller_.activeTrajectoryCache().sample(
            now, controller_.getConfig().nmpc.reference_timeout, reference)) {
        return msg;
    }

    const float ex = static_cast<float>(sensor_data_.x - reference.position.x());
    const float ey = static_cast<float>(sensor_data_.y - reference.position.y());
    const float ez = static_cast<float>(sensor_data_.z - reference.position.z());
    const float norm = std::sqrt(ex * ex + ey * ey + ez * ez);
    msg.data.push_back(ex);
    msg.data.push_back(ey);
    msg.data.push_back(ez);
    msg.data.push_back(norm);
    msg.data.push_back(static_cast<float>(reference.position.x()));
    msg.data.push_back(static_cast<float>(reference.position.y()));
    msg.data.push_back(static_cast<float>(reference.position.z()));
    msg.data.push_back(static_cast<float>(sensor_data_.x));
    msg.data.push_back(static_cast<float>(sensor_data_.y));
    msg.data.push_back(static_cast<float>(sensor_data_.z));
    return msg;
}

std_msgs::Float32MultiArray DebugOutputConsumer::makeTopicStatsMessage(
    const SensorData::TopicStats& stats) {
    std_msgs::Float32MultiArray msg;
    msg.data.reserve(5);
    msg.data.push_back(static_cast<float>(stats.frequency_hz));
    msg.data.push_back(static_cast<float>(stats.dt_max));
    msg.data.push_back(static_cast<float>(stats.jitter));
    msg.data.push_back(static_cast<float>(stats.time_since_last_msg));
    msg.data.push_back(stats.is_active ? 1.0f : 0.0f);
    return msg;
}

std_msgs::Float32MultiArray DebugOutputConsumer::makeVrpnQualityMessage(
    const ros1_utils::PositionQualityStats& stats) {
    std_msgs::Float32MultiArray msg;
    msg.data.reserve(10);
    msg.data.push_back(static_cast<float>(stats.x_mean_deviation));
    msg.data.push_back(static_cast<float>(stats.y_mean_deviation));
    msg.data.push_back(static_cast<float>(stats.z_mean_deviation));
    msg.data.push_back(static_cast<float>(stats.last_jump_magnitude));
    msg.data.push_back(static_cast<float>(stats.effective_frequency_hz));
    msg.data.push_back(stats.x_is_valid ? 1.0f : 0.0f);
    msg.data.push_back(stats.y_is_valid ? 1.0f : 0.0f);
    msg.data.push_back(stats.z_is_valid ? 1.0f : 0.0f);
    msg.data.push_back(stats.frame_is_valid ? 1.0f : 0.0f);
    msg.data.push_back(stats.position_jump_detected ? 1.0f : 0.0f);
    return msg;
}

DebugOutputConsumer::SensorStatsSnapshot DebugOutputConsumer::snapshotSensorStats() const {
    return SensorStatsSnapshot{sensor_data_.uav_state_estimate_stats,
                               sensor_data_.local_pos_stats,
                               sensor_data_.local_velocity_stats,
                               sensor_data_.imu_stats,
                               sensor_data_.state_stats,
                               sensor_data_.battery_stats,
                               sensor_data_.vrpn_pose_stats,
                               sensor_data_.vrpn_twist_stats,
                               vrpn_quality_stats_};
}

std_msgs::UInt32MultiArray DebugOutputConsumer::snapshotStateMachineEvents() const {
    std_msgs::UInt32MultiArray msg;
    const auto events = controller_.getStateMachine().currentEvents();
    msg.data.reserve(events.size());
    for (const auto& record : events) {
        msg.data.push_back(record.event.id);
    }
    return msg;
}

void DebugOutputConsumer::printSensorDebug(const SensorData& sensor_data) {
    ROS_INFO("[SensorData Debug] ================================");
    ROS_INFO(
        "[SensorData Debug] StateEstimate: state=%u flags=0x%08x "
        "stamp=%.3f active=%s",
        static_cast<unsigned>(sensor_data.uav_state_estimator_state),
        static_cast<unsigned>(sensor_data.uav_state_estimator_flags),
        sensor_data.uav_state_estimate_stamp,
        sensor_data.uav_state_estimate_stats.is_active ? "true" : "false");
    ROS_INFO("[SensorData Debug] Control position: x=%.3f, y=%.3f, z=%.3f", sensor_data.x,
             sensor_data.y, sensor_data.z);
    ROS_INFO("[SensorData Debug] Quaternion: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f", sensor_data.qx,
             sensor_data.qy, sensor_data.qz, sensor_data.qw);
    ROS_INFO("[SensorData Debug] Velocity: vx=%.3f, vy=%.3f, vz=%.3f", sensor_data.vx,
             sensor_data.vy, sensor_data.vz);
    ROS_INFO("[SensorData Debug] Angular velocity: wx=%.3f, wy=%.3f, wz=%.3f", sensor_data.wx,
             sensor_data.wy, sensor_data.wz);
    ROS_INFO("[SensorData Debug] Battery: %.1f%%", sensor_data.battery_percentage * 100.0);
    ROS_INFO(
        "[SensorData Debug] LocalPos: %.1f Hz, dt_max=%.6f s, jitter=%.6f "
        "s, time_since_last=%.3f s, active=%s",
        sensor_data.local_pos_stats.frequency_hz, sensor_data.local_pos_stats.dt_max,
        sensor_data.local_pos_stats.jitter, sensor_data.local_pos_stats.time_since_last_msg,
        sensor_data.local_pos_stats.is_active ? "true" : "false");
    ROS_INFO(
        "[SensorData Debug] LocalVel: %.1f Hz, dt_max=%.6f s, jitter=%.6f "
        "s, time_since_last=%.3f s, active=%s",
        sensor_data.local_velocity_stats.frequency_hz, sensor_data.local_velocity_stats.dt_max,
        sensor_data.local_velocity_stats.jitter,
        sensor_data.local_velocity_stats.time_since_last_msg,
        sensor_data.local_velocity_stats.is_active ? "true" : "false");
    ROS_INFO(
        "[SensorData Debug] IMU: %.1f Hz, dt_max=%.6f s, jitter=%.6f s, "
        "time_since_last=%.3f s, active=%s",
        sensor_data.imu_stats.frequency_hz, sensor_data.imu_stats.dt_max,
        sensor_data.imu_stats.jitter, sensor_data.imu_stats.time_since_last_msg,
        sensor_data.imu_stats.is_active ? "true" : "false");
    ROS_INFO(
        "[SensorData Debug] State: %.1f Hz, dt_max=%.6f s, jitter=%.6f s, "
        "time_since_last=%.3f s, active=%s",
        sensor_data.state_stats.frequency_hz, sensor_data.state_stats.dt_max,
        sensor_data.state_stats.jitter, sensor_data.state_stats.time_since_last_msg,
        sensor_data.state_stats.is_active ? "true" : "false");
    ROS_INFO(
        "[SensorData Debug] Battery: %.1f Hz, dt_max=%.6f s, jitter=%.6f s, "
        "time_since_last=%.3f s, active=%s",
        sensor_data.battery_stats.frequency_hz, sensor_data.battery_stats.dt_max,
        sensor_data.battery_stats.jitter, sensor_data.battery_stats.time_since_last_msg,
        sensor_data.battery_stats.is_active ? "true" : "false");
    ROS_INFO(
        "[SensorData Debug] FCU: connected=%s, armed=%s, guided=%s, "
        "manual_input=%s, mode=%s, status=%u",
        sensor_data.fcu_connected ? "true" : "false", sensor_data.fcu_armed ? "true" : "false",
        sensor_data.fcu_guided ? "true" : "false", sensor_data.fcu_manual_input ? "true" : "false",
        sensor_data.fcu_mode.c_str(), static_cast<unsigned>(sensor_data.fcu_system_status));
    ROS_INFO("[SensorData Debug] ================================");
}

}  // namespace px4_multirotor_controller
