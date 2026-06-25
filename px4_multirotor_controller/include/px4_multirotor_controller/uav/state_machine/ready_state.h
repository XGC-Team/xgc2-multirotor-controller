#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 就绪状态
// 系统已就绪，等待起飞命令
class ReadyState : public ::state_machine::State {
   public:
    explicit ReadyState(DroneController& controller);
    ~ReadyState() override = default;

    std::string name() const override {
        return "Ready";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    DroneController& controller_;
};

}  // namespace px4_multirotor_controller
