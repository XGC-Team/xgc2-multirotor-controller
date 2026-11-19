#pragma once

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>
#include <unistd.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "px4_multirotor_controller/output/normal_disarm_trace.h"

namespace px4_multirotor_controller {

namespace detail {

inline bool sendArmDisarmCommand(ros::ServiceClient* client, bool arm, const char* log_prefix,
                                 const NormalDisarmTrace* trace = nullptr) {
    if (!client) {
        ROS_ERROR("[%s] Service client not set!", log_prefix);
        return false;
    }

    constexpr uint16_t kMavCmdComponentArmDisarm = 400;

    mavros_msgs::CommandLong srv;
    srv.request.command = kMavCmdComponentArmDisarm;
    srv.request.param1 = arm ? 1.0 : 0.0;
    srv.request.param2 = 0.0;
    srv.request.param3 = 0.0;
    srv.request.param4 = 0.0;
    srv.request.param5 = 0.0;
    srv.request.param6 = 0.0;
    srv.request.param7 = 0.0;

    const bool replied = client->call(srv);
    if (!arm && trace && trace->sequence != 0) {
        const auto owner = normalDisarmIdentity(*trace, client->getService(),
                                                ros::this_node::getName(), ::getpid());
        if (!replied) {
            ROS_ERROR("[%s] Failed to call arm/disarm service: %s outcome=no_service_response",
                      log_prefix, owner.c_str());
        } else if (!srv.response.success) {
            ROS_WARN("[%s] Disarm request sent, result: FAILED %s outcome=rejected mav_result=%u",
                     log_prefix, owner.c_str(), static_cast<unsigned>(srv.response.result));
        } else {
            ROS_INFO("[%s] Disarm request sent, result: SUCCESS %s outcome=accepted mav_result=%u",
                     log_prefix, owner.c_str(), static_cast<unsigned>(srv.response.result));
        }
        return replied && srv.response.success;
    }
    if (replied) {
        const char* action = arm ? "Arm" : "Disarm";
        ROS_INFO("[%s] %s request sent, result: %s", log_prefix, action,
                 srv.response.success ? "SUCCESS" : "FAILED");
        return srv.response.success;
    }
    ROS_ERROR("[%s] Failed to call arm/disarm service", log_prefix);
    return false;
}

inline void sendModeCommand(ros::ServiceClient* client, const std::string& mode) {
    if (!client) {
        ROS_ERROR("[SetModeOutputTask] Service client not set!");
        return;
    }
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;
    if (client->call(srv)) {
        ROS_INFO("[SetModeOutputTask] Set mode to '%s', result: %s", mode.c_str(),
                 srv.response.mode_sent ? "SUCCESS" : "FAILED");
    } else {
        ROS_ERROR("[SetModeOutputTask] Failed to call set_mode service");
    }
}

}  // namespace detail

}  // namespace px4_multirotor_controller
