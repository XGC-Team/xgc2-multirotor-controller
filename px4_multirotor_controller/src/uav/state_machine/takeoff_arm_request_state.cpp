#include "px4_multirotor_controller/uav/state_machine/takeoff_arm_request_state.h"

#include <chrono>
#include <utility>

#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

TakeoffArmRequestState::TakeoffArmRequestState(DroneController& controller) : controller_(controller) {}

void TakeoffArmRequestState::onEnter() {
    controller_.logInfo("[TakeoffArmRequestState] Entering ARM Request State");

    confirmed_arm_frames_ = 0;
    arm_ready_event_posted_ = false;

    controller_.emitFlightEffect(FlightSetpoint{controller_.getCurrentTime()});
    setpoint_publish_timer_.start();
    arm_request_timer_.start();

    controller_.logInfo("[TakeoffArmRequestState] Ready to request ARM");
    return;
}

void TakeoffArmRequestState::onTick() {
    publishSetpointIfDue();
    requestArmIfDue();
    updateArmConfirmation();
    postArmReadyOnce();
    return;
}

void TakeoffArmRequestState::publishSetpointIfDue() {
    if (shouldRunEvery(setpoint_publish_timer_, SETPOINT_PUBLISH_INTERVAL, true)) {
        controller_.emitFlightEffect(FlightSetpoint{controller_.getCurrentTime()});
    }
}

void TakeoffArmRequestState::requestArmIfDue() {
    if (arm_request_timer_.elapsed().count() < ARM_REQUEST_INTERVAL) {
        return;
    }

    const auto& sensor_data = controller_.getSensorData();
    controller_.logInfo("[TakeoffArmRequestState] Requesting ARM (periodic, armed: %s)",
                        sensor_data.fcu_armed ? "true" : "false");
    controller_.emitFlightEffect(FlightArmRequest{controller_.getCurrentTime(), true});
    arm_request_timer_.reset();
}

void TakeoffArmRequestState::updateArmConfirmation() {
    const auto& sensor_data = controller_.getSensorData();
    if (!sensor_data.state_stats.is_new) {
        return;
    }

    if (sensor_data.fcu_armed) {
        if (confirmed_arm_frames_ < REQUIRED_ARM_FRAMES) {
            ++confirmed_arm_frames_;
        }
        controller_.logInfo("[TakeoffArmRequestState] Received ARM frame %d/%d",
                            confirmed_arm_frames_, REQUIRED_ARM_FRAMES);
        return;
    }

    if (confirmed_arm_frames_ > 0) {
        controller_.logWarn("[TakeoffArmRequestState] FCU disarmed, resetting ARM counter (was %d)",
                            confirmed_arm_frames_);
        confirmed_arm_frames_ = 0;
    }
}

void TakeoffArmRequestState::postArmReadyOnce() {
    if (arm_ready_event_posted_ || confirmed_arm_frames_ < REQUIRED_ARM_FRAMES) {
        return;
    }

    controller_.logInfo("[TakeoffArmRequestState] ARM ready (%d frames received)",
                        confirmed_arm_frames_);
    arm_ready_event_posted_ = true;
    controller_.emitFlightSignal(FlightSignal::ArmReady);
}

void TakeoffArmRequestState::onExit() {
    const auto& sensor_data = controller_.getSensorData();
    const double elapsed_time =
        std::chrono::duration<double>(controller_.flightElapsed()).count();
    controller_.logInfo(
        "[TakeoffArmRequestState] Exiting (armed: %s, ARM frames: %d, "
        "elapsed: %.2f s)",
        sensor_data.fcu_armed ? "true" : "false", confirmed_arm_frames_, elapsed_time);
    arm_request_timer_.stop();
    setpoint_publish_timer_.stop();
    return;
}

}  // namespace px4_multirotor_controller
