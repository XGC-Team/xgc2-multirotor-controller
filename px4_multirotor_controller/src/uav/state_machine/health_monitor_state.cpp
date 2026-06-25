#include "px4_multirotor_controller/uav/state_machine/health_monitor_state.h"

#include <cmath>
#include <utility>

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

using namespace event_type;

HealthMonitorState::HealthMonitorState(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult HealthMonitorState::onTick(::state_machine::StateContext& ctx) {
    const auto& sd = controller_.getSensorData();
    const auto& controller_config = controller_.getConfig();
    const auto& cfg = controller_config.safety;
    auto& ss = safety_state_;

    checkSensorActiveEdge(ctx, sd.uav_state_estimate_stats, ss.was_uav_state_estimate_active,
                          SAFE_TIMEOUT_UAV_STATE_ESTIMATE);
    checkSensorActiveEdge(ctx, sd.state_stats, ss.was_state_active, SAFE_TIMEOUT_STATE);
    checkSensorActiveEdge(ctx, sd.battery_stats, ss.was_battery_active, SAFE_TIMEOUT_BATTERY);

    const bool estimate_unusable = sd.uav_state_estimate_stats.is_active &&
                                   !sensor_checks::isStateEstimateUsableForControl(sd);
    if (!ss.state_estimate_unusable && estimate_unusable) {
        postSafetyEvent(ctx, SAFE_UAV_STATE_ESTIMATE_UNUSABLE,
                        "post state estimate unusable safety event");
    }
    ss.state_estimate_unusable = estimate_unusable;

    if (sd.uav_state_estimate_stats.is_new) {
        const bool currently_violated =
            (sd.x < cfg.fence_x_min || sd.x > cfg.fence_x_max || sd.y < cfg.fence_y_min ||
             sd.y > cfg.fence_y_max || sd.z < cfg.fence_z_min || sd.z > cfg.fence_z_max);
        if (!ss.geofence_violated && currently_violated) {
            postSafetyEvent(ctx, SAFE_GEOFENCE_VIOLATION, "post geofence safety event");
        }
        ss.geofence_violated = currently_violated;
    }

    if (sd.uav_state_estimate_stats.is_new) {
        const double velocity_xy = std::sqrt(sd.vx * sd.vx + sd.vy * sd.vy);
        const bool xy_exceeded = velocity_xy > cfg.max_velocity_xy;
        if (!ss.velocity_xy_exceeded && xy_exceeded) {
            postSafetyEvent(ctx, SAFE_VELOCITY_XY_EXCEEDED, "post velocity xy safety event");
        }
        ss.velocity_xy_exceeded = xy_exceeded;

        const bool z_exceeded = std::abs(sd.vz) > cfg.max_velocity_z;
        if (!ss.velocity_z_exceeded && z_exceeded) {
            postSafetyEvent(ctx, SAFE_VELOCITY_Z_EXCEEDED, "post velocity z safety event");
        }
        ss.velocity_z_exceeded = z_exceeded;
    }

    if (sd.imu_stats.is_new &&
        controller_config.tracking_backend != TrackingBackend::NMPC_ATTITUDE_RATE) {
        const auto& setpoint = controller_.getSetpoint();
        const double acc_xy = std::sqrt(setpoint.ax * setpoint.ax + setpoint.ay * setpoint.ay);
        const bool xy_saturated = acc_xy > cfg.acc_saturation_xy;
        if (!ss.control_saturated_xy && xy_saturated) {
            postSafetyEvent(ctx, SAFE_CONTROL_SATURATION_XY, "post control xy saturation event");
        }
        ss.control_saturated_xy = xy_saturated;

        const bool z_saturated = std::abs(setpoint.az) > cfg.acc_saturation_z;
        if (!ss.control_saturated_z && z_saturated) {
            postSafetyEvent(ctx, SAFE_CONTROL_SATURATION_Z, "post control z saturation event");
        }
        ss.control_saturated_z = z_saturated;
    } else if (controller_config.tracking_backend == TrackingBackend::NMPC_ATTITUDE_RATE) {
        ss.control_saturated_xy = false;
        ss.control_saturated_z = false;
    }

    return {};
}

void HealthMonitorState::postSafetyEvent(::state_machine::StateContext& ctx,
                                         ::state_machine::EventId event_id,
                                         const char* operation) const {
    ::state_machine::Event event(event_id,
                                 ::state_machine::EventTimestamp{controller_.getCurrentTime()});
    event.source = "health";
    auto status = ctx.postInternalEvent(std::move(event));
    if (!status.ok()) {
        controller_.logError("[HealthMonitorState] %s failed: %s", operation,
                             status.message.c_str());
    }
}

void HealthMonitorState::checkSensorActiveEdge(::state_machine::StateContext& ctx,
                                               const SensorData::TopicStats& stats,
                                               bool& was_active,
                                               ::state_machine::EventId event_id) const {
    const bool currently_active = stats.is_active;
    if (was_active && !currently_active) {
        postSafetyEvent(ctx, event_id, "post sensor safety event");
    }
    was_active = currently_active;
}

}  // namespace px4_multirotor_controller
