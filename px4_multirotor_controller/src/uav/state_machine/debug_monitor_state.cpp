#include "px4_multirotor_controller/uav/state_machine/debug_monitor_state.h"

#include <utility>

#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

DebugMonitorState::DebugMonitorState(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult DebugMonitorState::onTick(::state_machine::StateContext& ctx) {
    const double now = controller_.getCurrentTime();
    emit(ctx, output_event_type::PUBLISH_STATE_MACHINE_EVENTS, now);

    if (!status_publish_initialized_ ||
        now - last_status_publish_time_ >= STATUS_PUBLISH_INTERVAL) {
        emit(ctx, output_event_type::PUBLISH_CONTROLLER_STATUS, now);
        emit(ctx, output_event_type::PUBLISH_SENSOR_STATS, now);
        emit(ctx, output_event_type::PUBLISH_TRACKING_ERROR, now);
        last_status_publish_time_ = now;
        status_publish_initialized_ = true;
    }

    if (!debug_print_initialized_ || now - last_debug_print_time_ >= DEBUG_PRINT_INTERVAL) {
        emit(ctx, output_event_type::PRINT_SENSOR_DEBUG, now);
        last_debug_print_time_ = now;
        debug_print_initialized_ = true;
    }

    return {};
}

void DebugMonitorState::emit(::state_machine::StateContext& ctx, ::state_machine::EventId event_id,
                             double timestamp) const {
    ::state_machine::Event event(event_id, ::state_machine::EventTimestamp{timestamp});
    event.source = "debug";
    auto status = ctx.emitOutput(std::move(event));
    if (!status.ok()) {
        controller_.logError("[DebugMonitorState] emit output event failed: %s",
                             status.message.c_str());
    }
}

}  // namespace px4_multirotor_controller
