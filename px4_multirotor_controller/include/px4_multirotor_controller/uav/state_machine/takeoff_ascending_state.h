#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 起飞-上升状态
// 职责：
// - 持续输出起飞 setpoint
// - 上升到目标高度
// - 检测高度到达条件（高度阈值 + 速度稳定，连续5帧）
// - 到达后发送 ALTITUDE_REACHED 事件转移到 Hover
// - 超时后发送 TAKEOFF_TIMEOUT 事件转移到 Landing
class TakeoffAscendingState : public ::state_machine::State {
   public:
    explicit TakeoffAscendingState(DroneController& controller);
    ~TakeoffAscendingState() override = default;

    std::string name() const override {
        return "TakeoffAscending";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void publishSetpointIfDue(::state_machine::StateContext& ctx);
    void logStatusIfDue();
    bool postTimeoutIfNeeded(::state_machine::StateContext& ctx);
    void updateAltitudeConfirmation();
    void postAltitudeReachedOnce(::state_machine::StateContext& ctx);

    DroneController& controller_;
    ::state_machine::runtime::Timer<> status_log_timer_;        // 状态日志节流计时器
    ::state_machine::runtime::Timer<> setpoint_publish_timer_;  // Setpoint 输出事件节流计时器

    // 起飞判定参数
    static constexpr double ALTITUDE_THRESHOLD = 0.1;         // 高度到达阈值（米）
    static constexpr double VELOCITY_THRESHOLD = 0.05;        // 速度稳定阈值（米/秒）
    static constexpr double ASCENDING_TIMEOUT = 30.0;         // 上升超时时间（秒）
    static constexpr double SETPOINT_PUBLISH_INTERVAL = 0.1;  // 10Hz
    static constexpr int CONSECUTIVE_SETTLED_FRAMES = 5;      // 连续满足条件的帧数阈值

    bool altitude_reached_event_posted_{false};  // 是否已经投递 ALTITUDE_REACHED
    bool timeout_event_posted_{false};           // 是否已经投递 TAKEOFF_TIMEOUT
    int confirmed_settled_frames_{0};            // 连续满足到达条件的帧数
};

}  // namespace px4_multirotor_controller
