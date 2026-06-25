#include "px4_multirotor_controller/uav/state_machine/takeoff_offboard_request_state.h"

#include <chrono>
#include <string>
#include <utility>

#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

TakeoffOffboardRequestState::TakeoffOffboardRequestState(DroneController& controller)
    : controller_(controller) {}

::state_machine::ActionResult TakeoffOffboardRequestState::onEnter(
    ::state_machine::StateContext& ctx) {
    controller_.logInfo("[TakeoffOffboardRequestState] Entering OFFBOARD Request State");

    confirmed_offboard_frames_ = 0;
    offboard_ready_event_posted_ = false;

    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                               ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    setpoint_publish_timer_.start();
    mode_request_timer_.start();

    controller_.logInfo("[TakeoffOffboardRequestState] Ready to request OFFBOARD mode");
    return {};
}

::state_machine::ActionResult TakeoffOffboardRequestState::onTick(
    ::state_machine::StateContext& ctx) {
    publishSetpointIfDue(ctx);
    requestOffboardModeIfDue(ctx);
    updateOffboardConfirmation();
    postOffboardReadyOnce(ctx);
    return {};
}

void TakeoffOffboardRequestState::publishSetpointIfDue(::state_machine::StateContext& ctx) {
    if (shouldRunEvery(setpoint_publish_timer_, SETPOINT_PUBLISH_INTERVAL, true)) {
        ctx.emitOutput(
            ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                                   ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    }
}

void TakeoffOffboardRequestState::requestOffboardModeIfDue(::state_machine::StateContext& ctx) {
    if (mode_request_timer_.elapsed().count() < OFFBOARD_REQUEST_INTERVAL) {
        return;
    }

    controller_.logInfo("[TakeoffOffboardRequestState] Requesting OFFBOARD mode (periodic)");
    ::state_machine::Event event(output_event_type::REQUEST_MODE,
                                 ::state_machine::EventTimestamp{controller_.getCurrentTime()});
    event.payload["mode"] = std::string("OFFBOARD");
    ctx.emitOutput(std::move(event));
    mode_request_timer_.reset();
}

void TakeoffOffboardRequestState::updateOffboardConfirmation() {
    const auto& sensor_data = controller_.getSensorData();
    if (!sensor_data.state_stats.is_new) {
        return;
    }

    if (sensor_data.fcu_mode == "OFFBOARD") {
        if (confirmed_offboard_frames_ < REQUIRED_OFFBOARD_FRAMES) {
            ++confirmed_offboard_frames_;
        }
        controller_.logInfo("[TakeoffOffboardRequestState] Received OFFBOARD frame %d/%d",
                            confirmed_offboard_frames_, REQUIRED_OFFBOARD_FRAMES);
        return;
    }

    if (confirmed_offboard_frames_ > 0) {
        controller_.logWarn(
            "[TakeoffOffboardRequestState] Mode changed to %s, resetting "
            "OFFBOARD counter (was %d)",
            sensor_data.fcu_mode.c_str(), confirmed_offboard_frames_);
        confirmed_offboard_frames_ = 0;
    }
}

void TakeoffOffboardRequestState::postOffboardReadyOnce(::state_machine::StateContext& ctx) {
    if (offboard_ready_event_posted_ || confirmed_offboard_frames_ < REQUIRED_OFFBOARD_FRAMES) {
        return;
    }

    controller_.logInfo("[TakeoffOffboardRequestState] OFFBOARD mode ready (%d frames received)",
                        confirmed_offboard_frames_);
    offboard_ready_event_posted_ = true;
    ctx.postInternalEvent(::state_machine::Event(
        event_type::OFFBOARD_READY, ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
}

::state_machine::ActionResult TakeoffOffboardRequestState::onExit(
    ::state_machine::StateContext& ctx) {
    const auto& sensor_data = controller_.getSensorData();
    const double elapsed_time =
        std::chrono::duration<double>(ctx.elapsed(state_type::TakeoffOffboardRequest)).count();
    controller_.logInfo(
        "[TakeoffOffboardRequestState] Exiting (mode: %s, OFFBOARD frames: "
        "%d, elapsed: %.2f s)",
        sensor_data.fcu_mode.c_str(), confirmed_offboard_frames_, elapsed_time);
    mode_request_timer_.stop();
    setpoint_publish_timer_.stop();
    return {};
}

}  // namespace px4_multirotor_controller
