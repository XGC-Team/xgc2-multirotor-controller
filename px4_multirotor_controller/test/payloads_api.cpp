// The payloads are plain old data in C++ too: the host copies them as bytes.
#include <cstdio>
#include <type_traits>

#include "px4_multirotor_controller/payloads.h"

template <class T>
constexpr bool plain() {
    return std::is_standard_layout<T>::value && std::is_trivially_copyable<T>::value;
}

static_assert(plain<xgc2_px4_state_estimate_v1>(), "state_estimate");
static_assert(plain<xgc2_px4_pose_v1>(), "pose");
static_assert(plain<xgc2_px4_velocity_v1>(), "velocity");
static_assert(plain<xgc2_px4_imu_v1>(), "imu");
static_assert(plain<xgc2_px4_fcu_state_v1>(), "fcu_state");
static_assert(plain<xgc2_px4_battery_v1>(), "battery");
static_assert(plain<xgc2_px4_command_v1>(), "command");
static_assert(plain<xgc2_px4_position_target_v1>(), "position_target");
static_assert(plain<xgc2_px4_hover_thrust_v1>(), "hover_thrust");
static_assert(plain<xgc2_px4_attitude_rate_target_v1>(), "attitude_rate_target");
static_assert(plain<xgc2_px4_fcu_request_v1>(), "fcu_request");
static_assert(plain<xgc2_px4_controller_status_v1>(), "controller_status");

int main() {
    std::puts("controller payload layout (C++) ok");
    return 0;
}
