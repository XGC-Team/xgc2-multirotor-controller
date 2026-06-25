#include "px4_multirotor_controller/output/px4_service_output_consumer.h"

#include <ros/ros.h>

#include <memory>
#include <string>
#include <variant>

#include "px4_multirotor_controller/common/px4_command.h"
#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

Px4ServiceOutputConsumer::Px4ServiceOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor)
    : nh_(nh), executor_(executor) {}

void Px4ServiceOutputConsumer::initializeClientsIfNeeded() {
    if (clients_initialized_) {
        return;
    }

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command", true);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode", true);
    clients_initialized_ = true;

    ROS_INFO("[Px4ServiceOutputConsumer] Registered critical service clients");
    ROS_INFO("[Px4ServiceOutputConsumer]   mavros/cmd/command ready: %s",
             arming_client_.exists() ? "true" : "false");
    ROS_INFO("[Px4ServiceOutputConsumer]   mavros/set_mode ready: %s",
             set_mode_client_.exists() ? "true" : "false");
}

bool Px4ServiceOutputConsumer::handle(const ::state_machine::Event& event) {
    switch (event.id) {
        case output_event_type::REQUEST_ARMING: {
            const auto it = event.payload.find("arm");
            if (it == event.payload.end() || !std::holds_alternative<bool>(it->second)) {
                ROS_WARN(
                    "[Px4ServiceOutputConsumer] REQUEST_ARMING missing bool "
                    "payload 'arm'");
                return true;
            }
            if (!clients_initialized_) {
                ROS_WARN(
                    "[Px4ServiceOutputConsumer] Arming client not initialized, "
                    "dropping arm request");
                return true;
            }
            executor_.pushTask(
                std::make_unique<ArmOutputTask>(std::get<bool>(it->second), &arming_client_));
            return true;
        }
        case output_event_type::REQUEST_KILL:
            if (!clients_initialized_) {
                ROS_WARN(
                    "[Px4ServiceOutputConsumer] Arming client not initialized, "
                    "dropping kill request");
                return true;
            }
            executor_.pushTask(std::make_unique<KillOutputTask>(&arming_client_));
            return true;
        case output_event_type::REQUEST_MODE: {
            const auto it = event.payload.find("mode");
            if (it == event.payload.end() || !std::holds_alternative<std::string>(it->second)) {
                ROS_WARN(
                    "[Px4ServiceOutputConsumer] REQUEST_MODE missing string "
                    "payload 'mode'");
                return true;
            }
            if (!clients_initialized_) {
                ROS_WARN(
                    "[Px4ServiceOutputConsumer] SetMode client not initialized, "
                    "dropping mode request");
                return true;
            }
            executor_.pushTask(std::make_unique<SetModeOutputTask>(
                std::get<std::string>(it->second), &set_mode_client_));
            return true;
        }
        default:
            return false;
    }
}

}  // namespace px4_multirotor_controller
