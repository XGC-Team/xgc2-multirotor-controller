#pragma once

#include "px4_multirotor_controller/uav/flight_signal.h"

#include "px4_multirotor_controller/common/controller_clock.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/state_machine/timing.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

/**
 * 自检状态
 * - 循环检查传感器状态（通过 Guard 直接判断）
 * - 传感器正常 → 通过 Guard 转移到 Normal，并展开到默认 Ready
 */
class SelfCheckState {
   public:
    explicit SelfCheckState(DroneController& controller);


    void onEnter();
    void onTick();
    void onExit();
    std::optional<double> nextWake() const;

   private:
    DroneController& controller_;

    ControllerTimer status_log_timer_;  // 状态日志节流计时器

    static constexpr double LOG_INTERVAL = 2.0;  // 日志打印间隔（秒）
};

}  // namespace px4_multirotor_controller
