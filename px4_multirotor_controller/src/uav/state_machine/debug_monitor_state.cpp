#include "px4_multirotor_controller/uav/state_machine/debug_monitor_state.h"

#include <utility>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

DebugMonitorState::DebugMonitorState(DroneController& controller) : controller_(controller) {}

void DebugMonitorState::onTick() {
    const double now = controller_.getCurrentTime();
    if (!controller_.flightDiagnosticEvents().empty())
        controller_.emitFlightEffect(FlightEvents{now});

    if (!status_publish_initialized_ ||
        now - last_status_publish_time_ >= STATUS_PUBLISH_INTERVAL) {
        controller_.emitFlightEffect(FlightStatus{now});
        controller_.emitFlightEffect(FlightSensorStats{now});
        controller_.emitFlightEffect(FlightTrackingError{now});
        last_status_publish_time_ = now;
        status_publish_initialized_ = true;
    }

    if (!debug_print_initialized_ || now - last_debug_print_time_ >= DEBUG_PRINT_INTERVAL) {
        controller_.emitFlightEffect(FlightSensorDebug{now});
        last_debug_print_time_ = now;
        debug_print_initialized_ = true;
    }

    return;
}


}  // namespace px4_multirotor_controller
