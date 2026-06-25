#include "px4_multirotor_controller/uav/state_machine/hover_state.h"

#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

HoverState::HoverState(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult HoverState::onEnter(::state_machine::StateContext& ctx) {
    controller_.logInfo("[HoverState] Entering Hover State");

    configureHoverSetpoint();
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                               ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    setpoint_publish_timer_.start();
    return {};
}

void HoverState::configureHoverSetpoint() {
    const auto& sensor_data = controller_.getSensorData();
    const auto& config = controller_.getConfig();
    Setpoint& setpoint = controller_.getSetpoint();

    setpoint.x = sensor_data.x;
    setpoint.y = sensor_data.y;
    setpoint.z = sensor_data.z;
    setpoint.vx = 0.0;
    setpoint.vy = 0.0;
    setpoint.vz = 0.0;
    setpoint.ax = 0.0;
    setpoint.ay = 0.0;
    setpoint.az = 0.0;
    setpoint.yaw_rate = 0.0;
    setpoint.type_mask = HOVER_POSITION_VELOCITY_TYPE_MASK;

    if (config.enable_yaw_control) {
        setpoint.qx = sensor_data.qx;
        setpoint.qy = sensor_data.qy;
        setpoint.qz = sensor_data.qz;
        setpoint.qw = sensor_data.qw;
        setpoint.type_mask &= ~IGNORE_YAW_BIT;
        controller_.logInfo("[HoverState] Yaw control enabled: holding current yaw");
    } else {
        setpoint.qx = 0.0;
        setpoint.qy = 0.0;
        setpoint.qz = 0.0;
        setpoint.qw = 1.0;
        controller_.logInfo("[HoverState] Yaw control disabled: ignoring yaw");
    }

    controller_.logInfo("[HoverState] Holding position (%.2f, %.2f, %.2f)", setpoint.x, setpoint.y,
                        setpoint.z);
}

::state_machine::ActionResult HoverState::onTick(::state_machine::StateContext& ctx) {
    publishSetpointIfDue(ctx);
    return {};
}

void HoverState::publishSetpointIfDue(::state_machine::StateContext& ctx) {
    if (shouldRunEvery(setpoint_publish_timer_, SETPOINT_PUBLISH_INTERVAL, true)) {
        ctx.emitOutput(
            ::state_machine::Event(output_event_type::PUBLISH_SETPOINT,
                                   ::state_machine::EventTimestamp{controller_.getCurrentTime()}));
    }
}

::state_machine::ActionResult HoverState::onExit(::state_machine::StateContext&) {
    controller_.logInfo("[HoverState] Exiting Hover State");
    setpoint_publish_timer_.stop();
    return {};
}

}  // namespace px4_multirotor_controller
