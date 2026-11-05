#pragma once

#include "px4_multirotor_controller/uav/flight_signal.h"

#include "px4_multirotor_controller/common/controller_clock.h"
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
class TakeoffOffboardRequestState {
   public:
    explicit TakeoffOffboardRequestState(DroneController& controller);


    void onEnter();
    void onTick();
    void onExit();
    std::optional<double> nextWake() const;

   private:
    void publishSetpointIfDue();
    void requestOffboardModeIfDue();
    void updateOffboardConfirmation();
    void postOffboardReadyOnce();

    DroneController& controller_;
    ControllerTimer mode_request_timer_;      // OFFBOARD 模式请求计时器
    ControllerTimer setpoint_publish_timer_;  // Setpoint 输出事件节流计时器
    int confirmed_offboard_frames_{0};         // 接收到的连续 OFFBOARD 状态帧数
    bool offboard_ready_event_posted_{false};  // 是否已经投递 OFFBOARD_READY

    static constexpr int REQUIRED_OFFBOARD_FRAMES = 1;
    static constexpr double OFFBOARD_REQUEST_INTERVAL = 3.0;  // OFFBOARD请求间隔（秒）
    static constexpr double SETPOINT_PUBLISH_INTERVAL = kLocalSetpointPublishInterval;
};

}  // namespace px4_multirotor_controller
