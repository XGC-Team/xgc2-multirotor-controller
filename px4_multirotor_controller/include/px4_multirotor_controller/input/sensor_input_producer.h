#pragma once

#include <estimator_vrpn_px4_rotor_state/RigidStateEstimate.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <ros1_utils/param_utils.h>
#include <ros1_utils/topic_stats.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include <functional>
#include <state_machine/state_machine.hpp>
#include <string>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

class SensorInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    SensorInputProducer(ros::NodeHandle& nh, SensorData& sensor_data,
                        ros1_utils::PositionQualityStats& vrpn_quality_stats, uint32_t queue_size,
                        EventSink event_sink, std::function<void()> on_state_message);

    void setVrpnQualityConfig(const ros1_utils::PositionQualityConfig& config);
    void setStateEstimateTopic(std::string state_estimate_topic);
    void setVrpnTopics(std::string pose_topic, std::string twist_topic);
    void start();
    void resetNewFlags();

   private:
    void stateEstimateCallback(
        const estimator_vrpn_px4_rotor_state::RigidStateEstimate::ConstPtr& msg);
    void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vrpnTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void postInputEvent(::state_machine::EventId event_id, const char* source);

    ros::NodeHandle& nh_;
    SensorData& sensor_data_;
    ros1_utils::PositionQualityStats& vrpn_quality_stats_;
    uint32_t queue_size_{5};
    EventSink event_sink_;
    std::function<void()> on_state_message_;
    ros1_utils::TopicStatsManager stats_manager_;
    ros1_utils::PositionQualityDetector vrpn_quality_detector_;
    std::string state_estimate_topic_{"alg/state_estimator/state"};
    std::string vrpn_pose_topic_{"pose"};
    std::string vrpn_twist_topic_{"twist"};
    bool started_{false};
};

}  // namespace px4_multirotor_controller
