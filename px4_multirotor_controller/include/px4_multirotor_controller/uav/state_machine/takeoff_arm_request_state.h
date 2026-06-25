#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 起飞-请求ARM解锁状态
// 职责：
// - 持续输出起飞 setpoint
// - 每隔3秒请求一次 ARM 解锁
// - 检测新的 ARM 状态确认帧
// - 确认后发送 ARM_READY 事件，自动转移到 ASCENDING
// - 超时保护由状态机 transition guard 统一处理
class TakeoffArmRequestState : public ::state_machine::State {
   public:
    explicit TakeoffArmRequestState(DroneController& controller);
    ~TakeoffArmRequestState() override = default;

    std::string name() const override {
        return "TakeoffArmRequest";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void publishSetpointIfDue(::state_machine::StateContext& ctx);
    void requestArmIfDue(::state_machine::StateContext& ctx);
    void updateArmConfirmation();
    void postArmReadyOnce(::state_machine::StateContext& ctx);

    DroneController& controller_;
    ::state_machine::runtime::Timer<> arm_request_timer_;       // ARM 请求计时器
    ::state_machine::runtime::Timer<> setpoint_publish_timer_;  // Setpoint 输出事件节流计时器
    int confirmed_arm_frames_{0};         // 接收到的连续 ARM 状态帧数
    bool arm_ready_event_posted_{false};  // 是否已经投递 ARM_READY

    static constexpr int REQUIRED_ARM_FRAMES = 1;
    static constexpr double ARM_REQUEST_INTERVAL = 3.0;       // ARM请求间隔（秒）
    static constexpr double SETPOINT_PUBLISH_INTERVAL = 0.1;  // 10Hz
};

}  // namespace px4_multirotor_controller
