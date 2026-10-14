#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/controller_clock.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// Native FCU landing handoff; completion is fresh connected FCU disarm feedback.
class LandingState : public ::state_machine::State {
   public:
    explicit LandingState(DroneController& controller);
    ~LandingState() override = default;

    std::string name() const override {
        return "Landing";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    // 退出原因枚举
    enum class ExitReason {
        UNKNOWN,   // 未知原因（默认）
        TOUCHDOWN  // 正常着陆
    };

    void logStatusIfDue();
    void reportTimeoutIfNeeded(::state_machine::StateContext& ctx);
    void postTouchdownOnce(::state_machine::StateContext& ctx);

    DroneController& controller_;
    double initial_altitude_{0.0};                              // 初始高度
    ControllerTimer log_timer_;               // 日志节流计时器
    double max_landing_duration_{LANDING_TIMEOUT};
    // Retain the previous diagnostic budget; this is not a descent setpoint or surface test.
    static constexpr double LANDING_DIAGNOSTIC_DESCENT_RATE = 0.2;
    static constexpr double LANDING_TIMEOUT = 60.0;            // 降落超时时间（秒）
    static constexpr double LANDING_TIME_MARGIN = 5.0;           // s

    bool touchdown_event_posted_{false};           // 是否已投递 TOUCHDOWN
    bool timeout_reported_{false};                 // Timeout is diagnostic, not touchdown.
    ExitReason exit_reason_{ExitReason::UNKNOWN};  // 退出原因
};

}  // namespace px4_multirotor_controller
