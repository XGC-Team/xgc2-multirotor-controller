#include "px4_multirotor_controller/output/px4_service_output_consumer.h"

#include <ros/ros.h>

#include <memory>
#include <string>
#include <variant>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/output/px4_command.h"

namespace px4_multirotor_controller {

Px4ServiceOutputConsumer::Px4ServiceOutputConsumer(ros::NodeHandle& nh,
                                                   ControllerOutputWorker& worker)
    : nh_(nh), worker_(worker) {}

void Px4ServiceOutputConsumer::initializeClientsIfNeeded() {
    if (clients_initialized_) {
        return;
    }

    // These low-rate commands must reconnect after a flight service appears
    // or restarts; a failed persistent connection otherwise remains invalid.
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    clients_initialized_ = true;

    ROS_INFO("[Px4ServiceOutputConsumer] Registered critical service clients");
}

bool Px4ServiceOutputConsumer::handle(const FlightEffect& effect) {
    if (const auto* arm = std::get_if<FlightArmRequest>(&effect)) {
        if (!clients_initialized_) {
            ROS_WARN("[Px4ServiceOutputConsumer] Arming client not initialized, dropping arm request");
            return true;
        }
        worker_.post(OutputArm{arm->arm, &arming_client_});
        return true;
    }
    if (const auto* mode = std::get_if<FlightModeRequest>(&effect)) {
        if (!clients_initialized_) {
            ROS_WARN("[Px4ServiceOutputConsumer] SetMode client not initialized, dropping mode request");
            return true;
        }
        worker_.post(OutputMode{mode->mode, &set_mode_client_});
        return true;
    }
    return false;
}

}  // namespace px4_multirotor_controller
