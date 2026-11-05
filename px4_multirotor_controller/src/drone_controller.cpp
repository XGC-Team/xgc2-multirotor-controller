#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/common/core_log.h"

#include "px4_multirotor_controller/common/controller_clock.h"


#include <chrono>
#include <stdexcept>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/uav/flight_state.h"

namespace px4_multirotor_controller {

DroneController::DroneController(const SensorData& sensor_data) : sensor_data_(sensor_data) {
    logInfo("[DroneController] Initializing...");
    flight_ = std::make_unique<FlightRuntime>(*this);
    logInfo("[DroneController] Initialized");
}

DroneController::~DroneController() = default;

void DroneController::update(double current_time) {
    flight_effects_.clear();
    current_time_ = current_time;
    ControllerClock::set(current_time);
    flight_->update();
}

bool DroneController::postCommand(FlightCommand command, double receipt_sec) {
    // State decisions have always used the turn clock; preserve command order.
    (void)receipt_sec;
    return flight_->postCommand(command);
}

void DroneController::emitFlightSignal(FlightSignal signal) { flight_->emitSignal(signal); }

std::chrono::steady_clock::duration DroneController::flightElapsed() const {
    return flight_->elapsed();
}

const HealthMonitorState::PositionDistance& DroneController::getPositionDistance() const {
    return flight_->positionDistance();
}

const std::vector<uint32_t>& DroneController::flightDiagnosticEvents() const {
    return flight_->diagnosticEvents();
}

FlightState DroneController::flightState() const { return flight_->flightState(); }

std::string DroneController::flightStateName() const {
    switch (flightState()) {
        case FlightState::SelfCheck: return "SelfCheck";
        case FlightState::Ready: return "Ready";
        case FlightState::TakeoffInit: return "TakeoffInit";
        case FlightState::TakeoffOffboardRequest: return "TakeoffOffboardRequest";
        case FlightState::TakeoffArmRequest: return "TakeoffArmRequest";
        case FlightState::TakeoffAscending: return "TakeoffAscending";
        case FlightState::Hover: return "Hover";
        case FlightState::Landing: return "Landing";
        case FlightState::Custom1: return "Custom1";
    }
    return {};
}

ControllerConfig DroneController::getConfig() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void DroneController::setConfig(const ControllerConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
}

void DroneController::requestCustom1Tracking() {
    custom1_requested_ = true;
}

void DroneController::clearCustom1Request() {
    custom1_requested_ = false;
}

bool DroneController::custom1Requested() const {
    return custom1_requested_;
}

bool DroneController::custom1ReferenceReady() const {
    const auto config = getConfig();
    if (config.tracking_backend != TrackingBackend::PX4_LOCAL) {
        return true;
    }
    const auto ready = [this, &config](const MpcTrajectoryState& sample) {
        if (!passThroughReferenceReady(sample)) {
            return false;
        }
        return passThroughPlanMatchesHover(
            sample, sensor_checks::worldX(sensor_data_, config.tracking_backend),
            sensor_checks::worldY(sensor_data_, config.tracking_backend),
            sensor_checks::worldZ(sensor_data_, config.tracking_backend),
            config.nmpc.plan_hover_xy_tol, config.nmpc.plan_hover_z_tol);
    };
    if (ready(mpc_trajectory_buffer_.active())) {
        return true;
    }
    return mpc_trajectory_buffer_.hasPending() && ready(mpc_trajectory_buffer_.pending());
}

void DroneController::emitLogInfo(const char* message) const {
    PMC_LOG_INFO("%s", message);
}

void DroneController::emitLogWarn(const char* message) const {
    PMC_LOG_WARN("%s", message);
}

void DroneController::emitLogError(const char* message) const {
    PMC_LOG_ERROR("%s", message);
}

}  // namespace px4_multirotor_controller
