#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 起飞-请求 OFFBOARD 模式状态
// 职责：
// - 持续输出起飞 setpoint，保证 PX4 可进入 OFFBOARD
// - 每隔3秒请求一次 OFFBOARD 模式
// - 检测新的 OFFBOARD 状态确认帧
// - 确认后发送 OFFBOARD_READY 事件，自动转移到 ARM_REQUEST
// - 超时保护由状态机 transition guard 统一处理
class TakeoffOffboardRequestState : public ::state_machine::State {
   public:
    explicit TakeoffOffboardRequestState(DroneController& controller);
    ~TakeoffOffboardRequestState() override = default;

    std::string name() const override {
        return "TakeoffOffboardRequest";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void publishSetpointIfDue(::state_machine::StateContext& ctx);
    void requestOffboardModeIfDue(::state_machine::StateContext& ctx);
    void updateOffboardConfirmation();
    void postOffboardReadyOnce(::state_machine::StateContext& ctx);

    DroneController& controller_;
    ::state_machine::runtime::Timer<> mode_request_timer_;      // OFFBOARD 模式请求计时器
    ::state_machine::runtime::Timer<> setpoint_publish_timer_;  // Setpoint 输出事件节流计时器
    int confirmed_offboard_frames_{0};         // 接收到的连续 OFFBOARD 状态帧数
    bool offboard_ready_event_posted_{false};  // 是否已经投递 OFFBOARD_READY

    static constexpr int REQUIRED_OFFBOARD_FRAMES = 1;
    static constexpr double OFFBOARD_REQUEST_INTERVAL = 3.0;  // OFFBOARD请求间隔（秒）
    static constexpr double SETPOINT_PUBLISH_INTERVAL = 0.1;  // 10Hz
};

}  // namespace px4_multirotor_controller
