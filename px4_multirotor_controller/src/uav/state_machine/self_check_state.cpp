#include "px4_multirotor_controller/uav/state_machine/self_check_state.h"

#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {

SelfCheckState::SelfCheckState(DroneController& controller) : controller_(controller) {}

::state_machine::ActionResult SelfCheckState::onEnter(::state_machine::StateContext&) {
    controller_.logInfo("[SelfCheckState] Entering SelfCheck State");

    // 启动日志计时器
    status_log_timer_.start();
    return {};
}

::state_machine::ActionResult SelfCheckState::onTick(::state_machine::StateContext&) {
    // 每2秒打印一次传感器状态
    if (status_log_timer_.elapsed().count() >= LOG_INTERVAL) {
        const auto& sensor_data = controller_.getSensorData();
        const bool vrpn_pose_active = sensor_data.vrpn_pose_stats.is_active;
        const bool vrpn_twist_active = sensor_data.vrpn_twist_stats.is_active;
        const bool pose_consistent = sensor_checks::isVrpnPoseConsistent(sensor_data);
        const auto state_source = controller_.getConfig().state_source;
        const bool control_state_ready =
            sensor_checks::isControlStateUsableForControl(sensor_data, state_source);
        const double vrpn_local_diff = sensor_checks::vrpnLocalPositionDiff(sensor_data);
        const char* state_source_name =
            state_source == StateSource::VRPN_DIRECT ? "vrpn_direct" : "state_estimator";

        controller_.logInfo(
            "[SelfCheckState] Checking sensors... "
            "ControlState:%s(%s est=%u flags=0x%08x) LocalPos:%s Velocity:%s "
            "IMU:%s State:%s Battery:%s VRPNPose:%s VRPNTwist:%s "
            "PoseConsistency:%s Diff:%.3fm FCU:%s(%s)",
            control_state_ready ? "OK" : "X", state_source_name,
            static_cast<unsigned>(sensor_data.uav_state_estimator_state),
            static_cast<unsigned>(sensor_data.uav_state_estimator_flags),
            sensor_data.local_pos_stats.is_active ? "OK" : "X",
            sensor_data.local_velocity_stats.is_active ? "OK" : "X",
            sensor_data.imu_stats.is_active ? "OK" : "X",
            sensor_data.state_stats.is_active ? "OK" : "X",
            sensor_data.battery_stats.is_active ? "OK" : "X", vrpn_pose_active ? "OK" : "X",
            vrpn_twist_active ? "OK" : "X", pose_consistent ? "OK" : "X", vrpn_local_diff,
            sensor_data.fcu_connected ? "OK" : "X", sensor_data.fcu_mode.c_str());

        status_log_timer_.reset();
    }
    return {};
}

::state_machine::ActionResult SelfCheckState::onExit(::state_machine::StateContext&) {
    controller_.logInfo("[SelfCheckState] Exiting SelfCheck State");
    status_log_timer_.stop();
    return {};
}

}  // namespace px4_multirotor_controller
