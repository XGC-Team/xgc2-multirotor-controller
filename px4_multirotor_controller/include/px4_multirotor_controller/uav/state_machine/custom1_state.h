#pragma once

#include <state_machine/state_machine.hpp>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/control/sliding_mode_controller.h"
#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 自定义状态1（供用户扩展）
// 可用于测试、特殊任务或自定义飞行模式
class Custom1State : public ::state_machine::State {
   public:
    explicit Custom1State(DroneController& controller);
    ~Custom1State() override = default;

    std::string name() const override {
        return "Custom1";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    DroneController& controller_;

    // ========== 100Hz 轨迹发布控制 ==========
    double last_publish_time_{0.0};  // 上次发布时间（用于10ms周期判断）
    double publish_period_{0.01};    // 发布周期：10ms = 100Hz

    // ========== 数据字段：参考轨迹和控制输出分离 ==========
    // 参考轨迹（MPC插值结果，只读，不修改）
    Setpoint multirotor_reference_trajectory_;

    // 控制输出（发送给PX4的最终控制信号）
    Setpoint control_output_;

    // 论文中的 trajectory lifter：离散 MPC 第一拍 -> 连续时间参考
    TrajectoryLifter trajectory_lifter_;

    // ========== 滑模控制器（用于模式1和模式2）==========
    SlidingModeController sliding_controller_{3.0, 3.0,
                                              0.5};  // 默认参数：k1=3.0, k2=3.0, epsilon=0.5

    // ========== UAV NMPC 事件调度 ==========
    uint64_t request_sequence_{0};
    uint64_t in_flight_sequence_{0};
    uint64_t consumed_result_sequence_{0};
    bool request_in_flight_{false};
    double last_request_time_{0.0};
    double request_deadline_{0.0};
    double last_success_time_{0.0};
    uint32_t consecutive_failures_{0};
    bool nmpc_reference_seen_{false};
    bool reference_exit_event_posted_{false};
    ::state_machine::runtime::Timer<> nmpc_wait_log_timer_;
    ::state_machine::runtime::Timer<> trajectory_wait_log_timer_;

    // ========== 辅助函数 ==========
    void handleNmpcEventMode(::state_machine::StateContext& ctx, double current_time);
    void consumeNmpcResult(::state_machine::StateContext& ctx, double current_time);
    void dispatchNmpcRequest(::state_machine::StateContext& ctx, double current_time);
    void publishBackupSetpoint(::state_machine::StateContext& ctx, double current_time);
    void publishCurrentHoverSetpoint(::state_machine::StateContext& ctx, double current_time);
    void postReferenceExit(::state_machine::StateContext& ctx, double current_time,
                           uint32_t event_id, const char* reason);
    bool referenceWillFinishBeforeNextHorizon(double current_time) const;
    bool shouldDispatchNmpc(double current_time) const;

    /// @brief 按 MPC 规划周期边界将 pending 轨迹帧切换为 active
    bool updateActiveMpcFrame(double current_time);

    /// @brief 检查是否满足发布条件
    bool shouldPublish(double current_time) const;

    /// @brief 计算位置和速度跟踪误差
    void computeTrackingErrors(const SensorData& sensor_data, Eigen::Vector3d& e_p,
                               Eigen::Vector3d& e_v) const;

    /// @brief 根据控制模式设置 type_mask
    void setTypeMask(ControlMode mode, bool enable_yaw);

    /// @brief 计算控制输出（三种模式的统一入口）
    void computeControlOutput();
};

}  // namespace px4_multirotor_controller
