#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 降落状态
// 无人机从当前位置降落到地面
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
        UNKNOWN,          // 未知原因（默认）
        TOUCHDOWN,        // 正常着陆
        LANDING_TIMEOUT,  // 降落超时
        OTHER             // 其他原因（如紧急停止）
    };

    static double getDescentVelocity(double altitude);
    void configureLandingSetpoint();
    void emitLandingSetpoint(::state_machine::StateContext& ctx);
    void updateDescentVelocityIfNeeded();
    void publishSetpointIfDue(::state_machine::StateContext& ctx);
    void logStatusIfDue();
    bool postTimeoutIfNeeded(::state_machine::StateContext& ctx);
    void updateTouchdownConfirmation();
    void postTouchdownOnce(::state_machine::StateContext& ctx);

    DroneController& controller_;
    Setpoint landing_setpoint_;                                 // 降落设定点
    double initial_altitude_{0.0};                              // 初始高度
    ::state_machine::runtime::Timer<> log_timer_;               // 日志节流计时器
    ::state_machine::runtime::Timer<> setpoint_publish_timer_;  // Setpoint 输出事件节流计时器
    double max_landing_duration_{LANDING_TIMEOUT};
    static constexpr uint16_t LANDING_VELOCITY_TYPE_MASK = 0b110111000111;
    static constexpr double LANDING_ALTITUDE_THRESHOLD = 0.3;  // 切换下降速度的高度阈值
    static constexpr double LANDING_VZ_HIGH_ALTITUDE = -0.2;   // 高空下降速度
    static constexpr double LANDING_VZ_LOW_ALTITUDE = -0.9;    // 低空下降速度
    static constexpr double SETPOINT_PUBLISH_INTERVAL = 0.1;   // 10Hz
    static constexpr double GROUND_ALTITUDE = 0.1;             // 着陆判断阈值
    static constexpr double LANDING_TIMEOUT = 60.0;            // 降落超时时间（秒）
    static constexpr double TOUCHDOWN_VELOCITY_THRESHOLD = 0.1;  // m/s
    static constexpr double LANDING_TIME_MARGIN = 5.0;           // s
    static constexpr int CONSECUTIVE_SETTLED_FRAMES = 5;  // 连续满足条件的帧数阈值

    bool touchdown_event_posted_{false};           // 是否已投递 TOUCHDOWN
    bool timeout_event_posted_{false};             // 是否已投递 LANDING_TIMEOUT
    int confirmed_landed_frames_{0};               // 连续满足着陆条件的帧数
    ExitReason exit_reason_{ExitReason::UNKNOWN};  // 退出原因
};

}  // namespace px4_multirotor_controller
