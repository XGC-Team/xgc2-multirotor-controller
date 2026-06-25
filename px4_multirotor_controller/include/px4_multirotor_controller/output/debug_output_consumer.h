#pragma once

#include <ros/ros.h>
#include <ros1_utils/param_utils.h>
#include <ros1_utils/topic_stats.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

class DebugOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    DebugOutputConsumer(ros::NodeHandle& nh,
                        ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
                        DroneController& controller, const SensorData& sensor_data,
                        const ros1_utils::PositionQualityStats& vrpn_quality_stats,
                        const bool& debug_print_enabled, uint32_t queue_size);

    std::string name() const override {
        return "DebugOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    struct SensorStatsSnapshot {
        SensorData::TopicStats state_estimate;
        SensorData::TopicStats local_pos;
        SensorData::TopicStats local_velocity;
        SensorData::TopicStats imu;
        SensorData::TopicStats state;
        SensorData::TopicStats battery;
        SensorData::TopicStats vrpn_pose;
        SensorData::TopicStats vrpn_twist;
        ros1_utils::PositionQualityStats vrpn_quality;
    };

    static std_msgs::Float32MultiArray makeTopicStatsMessage(const SensorData::TopicStats& stats);
    static std_msgs::Float32MultiArray makeVrpnQualityMessage(
        const ros1_utils::PositionQualityStats& stats);
    static void printSensorDebug(const SensorData& sensor_data);

    SensorStatsSnapshot snapshotSensorStats() const;
    std_msgs::Float32MultiArray snapshotTrackingError() const;
    std_msgs::UInt32MultiArray snapshotStateMachineEvents() const;

    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    DroneController& controller_;
    const SensorData& sensor_data_;
    const ros1_utils::PositionQualityStats& vrpn_quality_stats_;
    const bool& debug_print_enabled_;

    ros::Publisher controller_status_pub_;
    ros::Publisher events_pub_;
    ros::Publisher stats_state_estimate_pub_;
    ros::Publisher stats_local_pos_pub_;
    ros::Publisher stats_local_vel_pub_;
    ros::Publisher stats_imu_pub_;
    ros::Publisher stats_state_pub_;
    ros::Publisher stats_battery_pub_;
    ros::Publisher stats_vrpn_pose_pub_;
    ros::Publisher stats_vrpn_twist_pub_;
    ros::Publisher vrpn_quality_pub_;
    ros::Publisher tracking_error_pub_;
};

}  // namespace px4_multirotor_controller
