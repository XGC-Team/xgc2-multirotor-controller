#pragma once

#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/common/controller_clock.h"
#include "px4_multirotor_controller/driver/command_input.h"
#include "px4_multirotor_controller/uav/active_trajectory_cache.h"
#include "px4_multirotor_controller/uav/flight_effect.h"
#include "px4_multirotor_controller/uav/mpc_trajectory_buffer.h"
#include "px4_multirotor_controller/uav/nmpc_result_buffer.h"
#include "px4_multirotor_controller/uav/state_machine/health_monitor_state.h"

namespace px4_multirotor_controller {

class FlightRuntime;

// 无人机控制器核心
// 业务逻辑层，管理状态机和控制策略，不依赖ROS
class DroneController {
   public:
    // 构造函数：接受 SensorData 常量引用（只读，避免拷贝）
    explicit DroneController(const SensorData& sensor_data);
    ~DroneController();

    // 主更新循环（由ROS节点在每个控制周期调用）
    // current_time: 当前时间戳（秒），用于频率控制
    void update(double current_time);

    bool postCommand(FlightCommand command, double receipt_sec);
    void emitFlightEffect(FlightEffect effect) {
        flight_effects_.push_back(std::move(effect));
    }
    const std::vector<FlightEffect>& flightEffects() const { return flight_effects_; }

    FlightState flightState() const;
    std::string flightStateName() const;

    // 数据访问接口（供State对象使用）
    const SensorData& getSensorData() const {
        return sensor_data_;
    }

    const HealthMonitorState::PositionDistance& getPositionDistance() const;

    void emitFlightSignal(FlightSignal signal);
    std::chrono::steady_clock::duration flightElapsed() const;
    const std::vector<uint32_t>& flightDiagnosticEvents() const;
    ControllerTimer& nmpcStaleOutputLogTimer() { return nmpc_stale_output_log_timer_; }
    double& custom1PublishPeriod() { return custom1_publish_period_; }

    Setpoint& getSetpoint() {
        return setpoint_;
    }
    const Setpoint& getSetpoint() const {
        return setpoint_;
    }

    AttitudeRateTarget& getAttitudeRateTarget() {
        return attitude_rate_target_;
    }
    const AttitudeRateTarget& getAttitudeRateTarget() const {
        return attitude_rate_target_;
    }

    // MPC轨迹状态访问接口（供 Custom1State 使用）
    // 独立于状态机内部的 setpoint，用于连续轨迹生成
    MpcTrajectoryBuffer& mpcTrajectoryBuffer() {
        return mpc_trajectory_buffer_;
    }
    const MpcTrajectoryBuffer& mpcTrajectoryBuffer() const {
        return mpc_trajectory_buffer_;
    }
    ActiveTrajectoryCache& activeTrajectoryCache() {
        return active_trajectory_cache_;
    }
    const ActiveTrajectoryCache& activeTrajectoryCache() const {
        return active_trajectory_cache_;
    }
    NmpcResultBuffer& nmpcResultBuffer() {
        return nmpc_result_buffer_;
    }
    const NmpcResultBuffer& nmpcResultBuffer() const {
        return nmpc_result_buffer_;
    }

    // 获取当前时间（供State对象使用）
    double getCurrentTime() const {
        return current_time_;
    }

    // 配置访问接口
    ControllerConfig getConfig() const;
    void setConfig(const ControllerConfig& config);

    void requestCustom1Tracking();
    void clearCustom1Request();
    bool custom1Requested() const;
    bool custom1ReferenceReady() const;

    template <typename... Args>
    void logInfo(const char* format, Args... args) const {
        const auto buffer = formatLogMessage(format, args...);
        emitLogInfo(buffer.data());
    }
    template <typename... Args>
    void logWarn(const char* format, Args... args) const {
        const auto buffer = formatLogMessage(format, args...);
        emitLogWarn(buffer.data());
    }
    template <typename... Args>
    void logError(const char* format, Args... args) const {
        const auto buffer = formatLogMessage(format, args...);
        emitLogError(buffer.data());
    }

   private:
    template <typename... Args>
    static std::array<char, 1024> formatLogMessage(const char* format, Args... args) {
        std::array<char, 1024> buffer{};
        if constexpr (sizeof...(Args) == 0) {
            std::snprintf(buffer.data(), buffer.size(), "%s", format == nullptr ? "" : format);
        } else {
            std::snprintf(buffer.data(), buffer.size(), format, args...);
        }
        return buffer;
    }

    void emitLogInfo(const char* message) const;
    void emitLogWarn(const char* message) const;
    void emitLogError(const char* message) const;

    std::vector<FlightEffect> flight_effects_;

    // One owner for the typed flight value state and its ordered input turn.
    std::unique_ptr<FlightRuntime> flight_;
    ControllerTimer nmpc_stale_output_log_timer_;
    // This original Custom1 value survives leaving and re-entering the state.
    double custom1_publish_period_{kLocalSetpointPublishInterval};

    // 配置
    mutable std::mutex config_mutex_;
    ControllerConfig config_;

    // 数据（传感器数据为外部引用，只读）
    const SensorData& sensor_data_;  // 传感器数据常量引用（由ROS节点管理）
    Setpoint setpoint_;              // 状态机内部使用的 setpoint（状态间共享）
    AttitudeRateTarget attitude_rate_target_;  // 状态机输出的 body-rate + thrust
    MpcTrajectoryBuffer mpc_trajectory_buffer_;
    ActiveTrajectoryCache active_trajectory_cache_;
    NmpcResultBuffer nmpc_result_buffer_;
    double current_time_{0.0};  // 当前时间戳（秒）
    bool custom1_requested_{false};
};

}  // namespace px4_multirotor_controller
