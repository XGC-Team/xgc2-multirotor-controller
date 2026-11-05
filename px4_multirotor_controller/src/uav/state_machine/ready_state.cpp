#include "px4_multirotor_controller/uav/state_machine/ready_state.h"

#include <utility>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

ReadyState::ReadyState(DroneController& controller) : controller_(controller) {}

void ReadyState::onEnter() {
    controller_.logInfo("[ReadyState] Entering Ready State");

    // 请求 disarm，确保系统处于未解锁状态
    // 但如果健康管理判定飞机可能在空中，跳过disarm
    const auto& sensor_data = controller_.getSensorData();
    if (sensor_checks::isAirborne(sensor_data, controller_.getConfig().tracking_backend)) {
        controller_.logWarn("[ReadyState] Airborne flag set, skipping disarm request");
    } else {
        controller_.emitFlightEffect(FlightArmRequest{controller_.getCurrentTime(), false});
    }
    return;
}


void ReadyState::onExit() {
    controller_.logInfo("[ReadyState] Exiting Ready State");
    return;
}

}  // namespace px4_multirotor_controller
