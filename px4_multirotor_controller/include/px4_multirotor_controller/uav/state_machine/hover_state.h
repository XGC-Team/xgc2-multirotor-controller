#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 悬停状态
// 无人机在当前位置保持悬停
class HoverState : public ::state_machine::State {
   public:
    explicit HoverState(DroneController& controller);
    ~HoverState() override = default;

    std::string name() const override {
        return "Hover";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void configureHoverSetpoint();
    void publishSetpointIfDue(::state_machine::StateContext& ctx);

    DroneController& controller_;
    ::state_machine::runtime::Timer<> setpoint_publish_timer_;  // Setpoint 输出事件节流计时器

    static constexpr uint16_t HOVER_POSITION_VELOCITY_TYPE_MASK = 0b110111000000;
    static constexpr uint16_t IGNORE_YAW_BIT = 1u << 10;
    static constexpr double SETPOINT_PUBLISH_INTERVAL = 0.1;  // 10Hz
};

}  // namespace px4_multirotor_controller
