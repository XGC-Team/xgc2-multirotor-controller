#include "px4_multirotor_controller/uav/state_machine/takeoff_offboard_request_state.h"

#include <chrono>
#include <string>
#include <utility>

#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

TakeoffOffboardRequestState::TakeoffOffboardRequestState(DroneController& controller) : controller_(controller) {}

void TakeoffOffboardRequestState::onEnter(
    ) {
    controller_.logInfo("[TakeoffOffboardRequestState] Entering OFFBOARD Request State");

    confirmed_offboard_frames_ = 0;
    offboard_ready_event_posted_ = false;

    controller_.emitFlightEffect(FlightSetpoint{controller_.getCurrentTime()});
    setpoint_publish_timer_.start();
    mode_request_timer_.start();

    controller_.logInfo("[TakeoffOffboardRequestState] Ready to request OFFBOARD mode");
    return;
}

void TakeoffOffboardRequestState::onTick(
    ) {
    publishSetpointIfDue();
    requestOffboardModeIfDue();
    updateOffboardConfirmation();
    postOffboardReadyOnce();
    return;
}

void TakeoffOffboardRequestState::publishSetpointIfDue() {
    if (shouldRunEvery(setpoint_publish_timer_, SETPOINT_PUBLISH_INTERVAL, true)) {
        controller_.emitFlightEffect(FlightSetpoint{controller_.getCurrentTime()});
    }
}

void TakeoffOffboardRequestState::requestOffboardModeIfDue() {
    if (mode_request_timer_.elapsed().count() < OFFBOARD_REQUEST_INTERVAL) {
        return;
    }

    controller_.logInfo("[TakeoffOffboardRequestState] Requesting OFFBOARD mode (periodic)");
    controller_.emitFlightEffect(FlightModeRequest{controller_.getCurrentTime(), "OFFBOARD"});
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

void TakeoffOffboardRequestState::postOffboardReadyOnce() {
    if (offboard_ready_event_posted_ || confirmed_offboard_frames_ < REQUIRED_OFFBOARD_FRAMES) {
        return;
    }

    controller_.logInfo("[TakeoffOffboardRequestState] OFFBOARD mode ready (%d frames received)",
                        confirmed_offboard_frames_);
    offboard_ready_event_posted_ = true;
    controller_.emitFlightSignal(FlightSignal::OffboardReady);
}

void TakeoffOffboardRequestState::onExit(
    ) {
    const auto& sensor_data = controller_.getSensorData();
    const double elapsed_time =
        std::chrono::duration<double>(controller_.flightElapsed()).count();
    controller_.logInfo(
        "[TakeoffOffboardRequestState] Exiting (mode: %s, OFFBOARD frames: "
        "%d, elapsed: %.2f s)",
        sensor_data.fcu_mode.c_str(), confirmed_offboard_frames_, elapsed_time);
    mode_request_timer_.stop();
    setpoint_publish_timer_.stop();
    return;
}

}  // namespace px4_multirotor_controller
