#include "px4_multirotor_controller/uav/state_machine/ready_state.h"

#include <utility>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

ReadyState::ReadyState(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult ReadyState::onEnter(::state_machine::StateContext& ctx) {
    controller_.logInfo("[ReadyState] Entering Ready State");

    // 请求 disarm，确保系统处于未解锁状态
    // 但如果健康管理判定飞机可能在空中，跳过disarm
    const auto& sensor_data = controller_.getSensorData();
    if (sensor_checks::isAirborne(sensor_data)) {
        controller_.logWarn("[ReadyState] Airborne flag set, skipping disarm request");
    } else {
        ::state_machine::Event event(output_event_type::REQUEST_ARMING,
                                     ::state_machine::EventTimestamp{controller_.getCurrentTime()});
        event.payload["arm"] = false;
        ctx.emitOutput(std::move(event));
    }
    return {};
}

::state_machine::ActionResult ReadyState::onTick(::state_machine::StateContext&) {
    // 就绪状态无需周期性控制输出，等待外部命令或 guard 转移。
    return {};
}

::state_machine::ActionResult ReadyState::onExit(::state_machine::StateContext&) {
    controller_.logInfo("[ReadyState] Exiting Ready State");
    return {};
}

}  // namespace px4_multirotor_controller
