#pragma once

#include "px4_multirotor_controller/uav/flight_signal.h"

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

// 前向声明
class DroneController;

// 就绪状态
// 系统已就绪，等待起飞命令
class ReadyState {
   public:
    explicit ReadyState(DroneController& controller);


    void onEnter();
    void onExit();

   private:
    DroneController& controller_;
};

}  // namespace px4_multirotor_controller
