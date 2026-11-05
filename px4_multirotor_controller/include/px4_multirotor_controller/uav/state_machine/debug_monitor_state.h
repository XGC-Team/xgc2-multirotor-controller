#pragma once

#include "px4_multirotor_controller/uav/flight_signal.h"

namespace px4_multirotor_controller {

class DroneController;

class DebugMonitorState final {
   public:
    explicit DebugMonitorState(DroneController& controller);


    void onTick();

   private:
    static constexpr double STATUS_PUBLISH_INTERVAL = 0.2;
    static constexpr double DEBUG_PRINT_INTERVAL = 1.0;

    DroneController& controller_;
    double last_status_publish_time_{0.0};
    double last_debug_print_time_{0.0};
    bool status_publish_initialized_{false};
    bool debug_print_initialized_{false};
};

}  // namespace px4_multirotor_controller
