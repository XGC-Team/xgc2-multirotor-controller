#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 起飞初始化状态
// 职责：
// - 记录起飞初始位置和高度
// - 配置起飞 Setpoint（保持XY，上升到目标高度）
// - 开始10Hz Setpoint输出
// - 可配置地跳过初始化阶段的 DISARM 请求与 ALTCTL 安全门
// - 未跳过时，请求切换到定高模式（ALTCTL）
// - 未跳过时，接收到两帧新的 ALTCTL 状态后自动转移到 OFFBOARD_REQUEST
class TakeoffInitState : public ::state_machine::State {
   public:
    explicit TakeoffInitState(DroneController& controller);
    ~TakeoffInitState() override = default;

    std::string name() const override {
        return "TakeoffInit";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void configureTakeoffSetpoint();
    void requestDisarmIfRequired(::state_machine::StateContext& ctx);
    void requestAltctlModeIfDue(::state_machine::StateContext& ctx);
    void updateAltctlConfirmation();
    void postAltctlReadyOnce(::state_machine::StateContext& ctx);

    DroneController& controller_;
    double initial_altitude_{0.0};           // 初始高度（用于日志）
    double target_altitude_{0.0};            // 本次起飞目标高度
    bool skip_altctl_gate_{false};           // 是否跳过 DISARM 与 ALTCTL 安全门
    bool altctl_ready_event_posted_{false};  // 是否已经投递 ALTCTL_READY
    int altctl_frame_count_{0};              // 接收到的连续 ALTCTL 状态帧数
    ::state_machine::runtime::Timer<> altctl_request_timer_;  // ALTCTL模式请求计时器
    ::state_machine::runtime::Timer<> setpoint_publish_timer_;  // Setpoint 输出事件节流计时器

    static constexpr int REQUIRED_ALTCTL_FRAMES = 2;
    static constexpr uint16_t TAKEOFF_POSITION_TYPE_MASK = 0b111111111000;
    static constexpr double ALTCTL_REQUEST_INTERVAL = 3.0;    // ALTCTL请求间隔（秒）
    static constexpr double SETPOINT_PUBLISH_INTERVAL = 0.1;  // 10Hz
};

}  // namespace px4_multirotor_controller
