#pragma once

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>

#include <cstdint>
#include <memory>
#include <state_machine/runtime/async_task_executor.hpp>
#include <string>
#include <utility>

namespace px4_multirotor_controller {

namespace detail {

inline bool sendArmDisarmCommand(ros::ServiceClient* client, bool arm, bool force_disarm,
                                 const char* log_prefix) {
    if (!client) {
        ROS_ERROR("[%s] Service client not set!", log_prefix);
        return false;
    }

    constexpr uint16_t kMavCmdComponentArmDisarm = 400;

    mavros_msgs::CommandLong srv;
    srv.request.command = kMavCmdComponentArmDisarm;
    srv.request.param1 = arm ? 1.0 : 0.0;
    srv.request.param2 = force_disarm ? 1.0 : 0.0;
    srv.request.param3 = 0.0;
    srv.request.param4 = 0.0;
    srv.request.param5 = 0.0;
    srv.request.param6 = 0.0;
    srv.request.param7 = 0.0;

    if (client->call(srv)) {
        const char* action = arm ? "Arm" : (force_disarm ? "ForceDisarm" : "Disarm");
        ROS_INFO("[%s] %s request sent, result: %s", log_prefix, action,
                 srv.response.success ? "SUCCESS" : "FAILED");
        return srv.response.success;
    }

    ROS_ERROR("[%s] Failed to call arm/disarm service", log_prefix);
    return false;
}

}  // namespace detail

class ArmOutputTask : public ::state_machine::runtime::Task<ros::NodeHandle> {
   public:
    ArmOutputTask(bool arm, ros::ServiceClient* client) : arm_(arm), client_(client) {}

    void execute(ros::NodeHandle& nh) override {
        (void)nh;
        detail::sendArmDisarmCommand(client_, arm_, false, "ArmOutputTask");
    }

    std::string name() const override {
        return arm_ ? "Arm" : "Disarm";
    }

   private:
    bool arm_;
    ros::ServiceClient* client_;
};

class KillOutputTask : public ::state_machine::runtime::Task<ros::NodeHandle> {
   public:
    explicit KillOutputTask(ros::ServiceClient* client) : client_(client) {}

    void execute(ros::NodeHandle& nh) override {
        (void)nh;
        detail::sendArmDisarmCommand(client_, false, true, "KillOutputTask");
    }

    std::string name() const override {
        return "Kill";
    }

   private:
    ros::ServiceClient* client_;
};

class SetModeOutputTask : public ::state_machine::runtime::Task<ros::NodeHandle> {
   public:
    SetModeOutputTask(std::string mode, ros::ServiceClient* client)
        : mode_(std::move(mode)), client_(client) {}

    void execute(ros::NodeHandle& nh) override {
        (void)nh;
        if (!client_) {
            ROS_ERROR("[SetModeOutputTask] Service client not set!");
            return;
        }

        mavros_msgs::SetMode srv;
        srv.request.custom_mode = mode_;

        if (client_->call(srv)) {
            ROS_INFO("[SetModeOutputTask] Set mode to '%s', result: %s", mode_.c_str(),
                     srv.response.mode_sent ? "SUCCESS" : "FAILED");
        } else {
            ROS_ERROR("[SetModeOutputTask] Failed to call set_mode service");
        }
    }

    std::string name() const override {
        return "SetMode(" + mode_ + ")";
    }

   private:
    std::string mode_;
    ros::ServiceClient* client_;
};

}  // namespace px4_multirotor_controller
