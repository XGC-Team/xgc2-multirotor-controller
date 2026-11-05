#pragma once

#include "px4_multirotor_controller/uav/flight_signal.h"
#include "px4_multirotor_controller/driver/command_input.h"

#include "px4_multirotor_controller/common/controller_clock.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 起飞-上升状态
// 职责：
// - 持续输出起飞 setpoint
// - 上升到目标高度
// - 检测高度到达条件（|z - 目标| <= 高度阈值，双侧 + 速度稳定，连续5帧）
// - 到达后发送 ALTITUDE_REACHED 事件转移到 Hover
// - 超时后发送 TAKEOFF_TIMEOUT 事件转移到 Landing
class TakeoffAscendingState {
   public:
    explicit TakeoffAscendingState(DroneController& controller);


    void onEnter();
    void onTick();
    void onEvent(FlightCommand command);
    void onExit();
    std::optional<double> nextWake() const;
    std::optional<double> timeoutAfterEntry() const;

   private:
    void publishSetpointIfDue();
    void logStatusIfDue();
    bool postTimeoutIfNeeded();
    void updateAltitudeConfirmation();
    void postAltitudeReachedOnce();

    DroneController& controller_;
    ControllerTimer status_log_timer_;        // 状态日志节流计时器
    ControllerTimer setpoint_publish_timer_;  // Setpoint 输出事件节流计时器

    // 起飞判定参数
    static constexpr double ALTITUDE_THRESHOLD = 0.1;         // 高度到达阈值（米）
    static constexpr double VELOCITY_THRESHOLD = 0.05;        // 速度稳定阈值（米/秒）
    static constexpr double ASCENDING_TIMEOUT = 30.0;         // 上升超时时间（秒）
    static constexpr double SETPOINT_PUBLISH_INTERVAL = kLocalSetpointPublishInterval;
    static constexpr int CONSECUTIVE_SETTLED_FRAMES = 5;      // 连续满足条件的帧数阈值

    bool altitude_reached_event_posted_{false};  // 是否已经投递 ALTITUDE_REACHED
    bool timeout_event_posted_{false};           // 是否已经投递 TAKEOFF_TIMEOUT
    int confirmed_settled_frames_{0};            // 连续满足到达条件的帧数
};

}  // namespace px4_multirotor_controller
