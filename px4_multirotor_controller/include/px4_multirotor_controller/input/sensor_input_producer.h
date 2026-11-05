#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/ros.h>
#include <ros1_utils/param_utils.h>
#include <ros1_utils/topic_stats.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include <functional>
#include <memory>
#include <string>

#include "px4_multirotor_controller/driver/sensor_statistics.h"

namespace px4_multirotor_controller {

class ControllerDriver;

class SensorInputProducer {
   public:
    SensorInputProducer(ros::NodeHandle& nh, SensorData& sensor_data,
                        ros1_utils::PositionQualityStats& vrpn_quality_stats,
                        SensorStatistics& statistics, uint32_t queue_size,
                        std::function<void()> on_state_message, ControllerDriver* driver = nullptr,
                        std::function<void()> on_input = {});

    void setVrpnQualityConfig(const ros1_utils::PositionQualityConfig& config);
    void setStateEstimateTopic(std::string state_estimate_topic);
    void setVrpnPoseTopic(std::string pose_topic);
    void start();
    void resetNewFlags();
    // Copy the ROS-side topic stats into SensorData. The node calls this right
    // before DroneController::update(), the only place the core reads them.
    void syncStats();

   private:
    void stateEstimateCallback(const rigid_state_estimator_msgs::RigidStateEstimate::ConstPtr& msg);
    void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void localPvCallback(const geometry_msgs::PoseStamped::ConstPtr& position,
                         const geometry_msgs::TwistStamped::ConstPtr& velocity);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void targetAttitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ControllerDriver* driver_{nullptr};
    ros::NodeHandle& nh_;
    SensorData& sensor_data_;
    ros1_utils::PositionQualityStats& vrpn_quality_stats_;
    uint32_t queue_size_{5};
    std::function<void()> on_state_message_;
    std::function<void()> on_input_;
    SensorStatistics& statistics_;
    std::vector<ros::Subscriber> subscribers_;
    using LocalPvPolicy = message_filters::sync_policies::ExactTime<geometry_msgs::PoseStamped,
                                                                   geometry_msgs::TwistStamped>;
    std::unique_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> paired_position_;
    std::unique_ptr<message_filters::Subscriber<geometry_msgs::TwistStamped>> paired_velocity_;
    std::unique_ptr<message_filters::Synchronizer<LocalPvPolicy>> local_pv_sync_;
    ros1_utils::PositionQualityDetector vrpn_quality_detector_;
    std::string state_estimate_topic_{"alg/state_estimator/state"};
    std::string vrpn_pose_topic_{"pose"};
    bool started_{false};
};

}  // namespace px4_multirotor_controller
