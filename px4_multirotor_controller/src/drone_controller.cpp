#include "px4_multirotor_controller/drone_controller.h"

#include <ros/ros.h>

#include <stdexcept>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/common/state_machine_queries.h"
#include "px4_multirotor_controller/uav/state_machine/custom1_state.h"
#include "px4_multirotor_controller/uav/state_machine/debug_monitor_state.h"
#include "px4_multirotor_controller/uav/state_machine/health_monitor_state.h"
#include "px4_multirotor_controller/uav/state_machine/hover_state.h"
#include "px4_multirotor_controller/uav/state_machine/landing_state.h"
#include "px4_multirotor_controller/uav/state_machine/ready_state.h"
#include "px4_multirotor_controller/uav/state_machine/self_check_state.h"
#include "px4_multirotor_controller/uav/state_machine/takeoff_arm_request_state.h"
#include "px4_multirotor_controller/uav/state_machine/takeoff_ascending_state.h"
#include "px4_multirotor_controller/uav/state_machine/takeoff_init_state.h"
#include "px4_multirotor_controller/uav/state_machine/takeoff_offboard_request_state.h"

namespace px4_multirotor_controller {

using namespace event_type;  // 简化事件常量引用

DroneController::DroneController(const SensorData& sensor_data) : sensor_data_(sensor_data) {
    logInfo("[DroneController] Initializing...");

    auto builder = ::state_machine::StateMachine::builder("FlightStateMachine");
    builder.region(region_type::HEALTH)
        .name("health")
        .order(0)
        .initial(state_type::HealthMonitor)
        .state(state_type::HealthMonitor)
        .name("HealthMonitor")
        .impl(std::make_unique<HealthMonitorState>(*this))
        .endRegion()
        .region(region_type::CONTROL)
        .name("flight")
        .order(10)
        .initial(state_type::SelfCheck)
        .state(state_type::SelfCheck)
        .name("SelfCheck")
        .impl(std::make_unique<SelfCheckState>(*this))
        .state(state_type::Normal)
        .name("Normal")
        .initial(state_type::Ready)
        .state(state_type::Ready)
        .name("Ready")
        .impl(std::make_unique<ReadyState>(*this))
        .state(state_type::Takeoff)
        .name("Takeoff")
        .initial(state_type::TakeoffInit)
        .state(state_type::TakeoffInit)
        .name("TakeoffInit")
        .impl(std::make_unique<TakeoffInitState>(*this))
        .state(state_type::TakeoffOffboardRequest)
        .name("TakeoffOffboardRequest")
        .impl(std::make_unique<TakeoffOffboardRequestState>(*this))
        .state(state_type::TakeoffArmRequest)
        .name("TakeoffArmRequest")
        .impl(std::make_unique<TakeoffArmRequestState>(*this))
        .state(state_type::TakeoffAscending)
        .name("TakeoffAscending")
        .impl(std::make_unique<TakeoffAscendingState>(*this))
        .endState()
        .state(state_type::Hover)
        .name("Hover")
        .impl(std::make_unique<HoverState>(*this))
        .state(state_type::Custom1)
        .name("Custom1")
        .impl(std::make_unique<Custom1State>(*this))
        .endState()
        .state(state_type::Landing)
        .name("Landing")
        .impl(std::make_unique<LandingState>(*this))
        .endRegion()
        .region(region_type::DEBUG)
        .name("debug")
        .order(20)
        .initial(state_type::DebugMonitor)
        .state(state_type::DebugMonitor)
        .name("DebugMonitor")
        .impl(std::make_unique<DebugMonitorState>(*this))
        .endRegion()
        .transition()
        .from(state_type::SelfCheck)
        .to(state_type::Normal)
        .priority(transition_priority::AUTOMATIC)
        .when([this](const ::state_machine::GuardContext&) {
            return sensor_checks::areSensorsAllActive(sensor_data_);
        })
        .transition()
        .from(state_type::Ready)
        .to(state_type::SelfCheck)
        .priority(transition_priority::AUTOMATIC)
        .when([this](const ::state_machine::GuardContext&) {
            return !sensor_checks::areSensorsAllActive(sensor_data_);
        })
        .transition()
        .from(state_type::Ready)
        .to(state_type::Landing)
        .priority(transition_priority::CRITICAL)
        .when([this](const ::state_machine::GuardContext&) {
            return sensor_checks::isAirborne(sensor_data_);
        })
        .transition()
        .from(state_type::Ready)
        .to(state_type::Takeoff)
        .on(CMD_TAKEOFF)
        .priority(transition_priority::COMMAND)
        .transition()
        .from(state_type::TakeoffInit)
        .to(state_type::Ready)
        .priority(transition_priority::CRITICAL)
        .when([this](const ::state_machine::GuardContext&) {
            return sensor_checks::isFcuArmed(sensor_data_);
        })
        .transition()
        .from(state_type::TakeoffOffboardRequest)
        .to(state_type::Ready)
        .priority(transition_priority::CRITICAL)
        .when([this](const ::state_machine::GuardContext&) {
            return sensor_checks::isFcuArmed(sensor_data_);
        })
        .transition()
        .from(state_type::TakeoffInit)
        .to(state_type::TakeoffOffboardRequest)
        .on(ALTCTL_READY)
        .priority(transition_priority::AUTOMATIC)
        .when([this](const ::state_machine::GuardContext&) {
            return !sensor_checks::isFcuArmed(sensor_data_);
        })
        .transition()
        .from(state_type::TakeoffOffboardRequest)
        .to(state_type::TakeoffArmRequest)
        .on(OFFBOARD_READY)
        .priority(transition_priority::AUTOMATIC)
        .transition()
        .from(state_type::TakeoffArmRequest)
        .to(state_type::TakeoffAscending)
        .on(ARM_READY)
        .priority(transition_priority::AUTOMATIC)
        .transition()
        .from(state_type::TakeoffAscending)
        .to(state_type::Hover)
        .on(ALTITUDE_REACHED)
        .priority(transition_priority::AUTOMATIC)
        .transition()
        .from(state_type::TakeoffAscending)
        .to(state_type::Hover)
        .on(CMD_HOVER)
        .priority(transition_priority::COMMAND)
        .transition()
        .from(state_type::TakeoffInit)
        .to(state_type::Landing)
        .priority(transition_priority::AUTOMATIC)
        .when([this](const ::state_machine::GuardContext&) {
            return state_machine_ && state_machine_queries::isActiveStateTimeout(
                                         *state_machine_, region_type::CONTROL, 10.0);
        })
        .transition()
        .from(state_type::TakeoffOffboardRequest)
        .to(state_type::Landing)
        .priority(transition_priority::AUTOMATIC)
        .when([this](const ::state_machine::GuardContext&) {
            return state_machine_ && state_machine_queries::isActiveStateTimeout(
                                         *state_machine_, region_type::CONTROL, 10.0);
        })
        .transition()
        .from(state_type::TakeoffArmRequest)
        .to(state_type::Landing)
        .priority(transition_priority::AUTOMATIC)
        .when([this](const ::state_machine::GuardContext&) {
            return state_machine_ && state_machine_queries::isActiveStateTimeout(
                                         *state_machine_, region_type::CONTROL, 10.0);
        })
        .transition()
        .from(state_type::TakeoffAscending)
        .to(state_type::Landing)
        .on(TAKEOFF_TIMEOUT)
        .priority(transition_priority::AUTOMATIC)
        .transition()
        .from(state_type::TakeoffAscending)
        .to(state_type::Custom1)
        .on(CMD_CUSTOM1)
        .priority(transition_priority::COMMAND)
        .transition()
        .from(state_type::Hover)
        .to(state_type::Custom1)
        .on(CMD_CUSTOM1)
        .priority(transition_priority::COMMAND)
        .transition()
        .from(state_type::Custom1)
        .to(state_type::Hover)
        .on(CMD_HOVER)
        .priority(transition_priority::COMMAND)
        .transition()
        .from(state_type::Custom1)
        .to(state_type::Hover)
        .on(REFERENCE_TRAJECTORY_FINISHED)
        .priority(transition_priority::AUTOMATIC)
        .transition()
        .from(state_type::Custom1)
        .to(state_type::Hover)
        .on(INPUT_REFERENCE_TRAJECTORY_LOST)
        .priority(transition_priority::AUTOMATIC)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(CMD_LAND)
        .priority(transition_priority::USER_COMMAND)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_GEOFENCE_VIOLATION)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_VELOCITY_XY_EXCEEDED)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_VELOCITY_Z_EXCEEDED)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_CONTROL_SATURATION_XY)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_CONTROL_SATURATION_Z)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_POSITION_JUMP)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_UAV_STATE_ESTIMATE_UNUSABLE)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_TIMEOUT_UAV_STATE_ESTIMATE)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_TIMEOUT_LOCAL_POS)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_TIMEOUT_LOCAL_VELOCITY)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_TIMEOUT_IMU)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_TIMEOUT_STATE)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_TIMEOUT_BATTERY)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_TIMEOUT_VRPN_POSE)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Normal)
        .to(state_type::Landing)
        .on(SAFE_TIMEOUT_VRPN_TWIST)
        .priority(transition_priority::EMERGENCY)
        .transition()
        .from(state_type::Landing)
        .to(state_type::SelfCheck)
        .on(TOUCHDOWN)
        .priority(transition_priority::AUTOMATIC)
        .transition()
        .from(state_type::Landing)
        .to(state_type::SelfCheck)
        .on(LANDING_TIMEOUT)
        .priority(transition_priority::AUTOMATIC);

    auto machine_result = builder.build();
    if (!machine_result.status.ok()) {
        throw std::runtime_error(machine_result.status.message);
    }
    state_machine_ = std::move(machine_result.value);

    // 启动状态机
    if (auto status = state_machine_->start(); !status.ok()) {
        throw std::runtime_error(status.message);
    }

    logInfo("[DroneController] Initialized");
}

void DroneController::update(double current_time) {
    // 1. 更新当前时间
    current_time_ = current_time;

    // 2. 更新状态机
    if (state_machine_) {
        (void)state_machine_->update();
    }
}

ControllerConfig DroneController::getConfig() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void DroneController::setConfig(const ControllerConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
}

void DroneController::emitLogInfo(const char* message) const {
    ROS_INFO("%s", message);
}

void DroneController::emitLogWarn(const char* message) const {
    ROS_WARN("%s", message);
}

void DroneController::emitLogError(const char* message) const {
    ROS_ERROR("%s", message);
}

}  // namespace px4_multirotor_controller
