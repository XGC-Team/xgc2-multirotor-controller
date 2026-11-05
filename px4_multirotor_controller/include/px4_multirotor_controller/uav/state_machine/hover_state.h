#pragma once

#include "px4_multirotor_controller/uav/flight_signal.h"
#include "px4_multirotor_controller/driver/command_input.h"

#include "px4_multirotor_controller/common/controller_clock.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 悬停状态
// 无人机在当前位置保持悬停
class HoverState {
   public:
    explicit HoverState(DroneController& controller);


    void onEnter();
    void onTick();
    void onEvent(FlightCommand command);
    void onExit();

   private:
    void configureHoverSetpoint();
    void publishSetpointIfDue();

    DroneController& controller_;
    ControllerTimer setpoint_publish_timer_;  // Setpoint 输出事件节流计时器

    static constexpr uint16_t HOVER_POSITION_VELOCITY_TYPE_MASK = 0b110111000000;
    static constexpr uint16_t IGNORE_YAW_BIT = 1u << 10;
    static constexpr double SETPOINT_PUBLISH_INTERVAL = kLocalSetpointPublishInterval;
};

}  // namespace px4_multirotor_controller
