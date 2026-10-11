/* The payloads header is plain C11: it compiles here, and its layout asserts hold. */
#include <stdio.h>
#include <string.h>

#include "px4_multirotor_controller/payloads.h"

int main(void) {
    xgc2_px4_state_estimate_v1 estimate;
    xgc2_px4_fcu_state_v1 fcu;
    xgc2_px4_position_target_v1 target;
    memset(&estimate, 0, sizeof estimate);
    memset(&fcu, 0, sizeof fcu);
    memset(&target, 0, sizeof target);
    estimate.orientation_xyzw[3] = 1.0;
    strcpy(fcu.mode, "OFFBOARD");
    target.type_mask = 3072;
    if (sizeof estimate != 216 || sizeof fcu != 48 || sizeof target != 104)
        return 1;
    if (strcmp(XGC2_PX4_STATE_ESTIMATE_SCHEMA, "xgc2.px4.state_estimate.v1") != 0)
        return 2;
    if (strcmp(XGC2_PX4_POSE_SCHEMA, "xgc2.px4.pose.v1") != 0)
        return 3;
    if (strcmp(XGC2_PX4_VELOCITY_SCHEMA, "xgc2.px4.velocity.v1") != 0)
        return 4;
    if (strcmp(XGC2_PX4_IMU_SCHEMA, "xgc2.px4.imu.v1") != 0)
        return 5;
    if (strcmp(XGC2_PX4_FCU_STATE_SCHEMA, "xgc2.px4.fcu_state.v1") != 0)
        return 6;
    if (strcmp(XGC2_PX4_BATTERY_SCHEMA, "xgc2.px4.battery.v1") != 0)
        return 7;
    if (strcmp(XGC2_PX4_COMMAND_SCHEMA, "xgc2.px4.command.v1") != 0)
        return 8;
    if (strcmp(XGC2_PX4_POSITION_TARGET_SCHEMA, "xgc2.px4.position_target.v1") != 0)
        return 9;
    if (strcmp(XGC2_PX4_HOVER_THRUST_SCHEMA, "xgc2.px4.hover_thrust.v1") != 0)
        return 10;
    if (strcmp(XGC2_PX4_ATTITUDE_RATE_TARGET_SCHEMA, "xgc2.px4.attitude_rate_target.v1") != 0)
        return 11;
    if (strcmp(XGC2_PX4_FCU_REQUEST_SCHEMA, "xgc2.px4.fcu_request.v1") != 0)
        return 12;
    if (strcmp(XGC2_PX4_CONTROLLER_STATUS_SCHEMA, "xgc2.px4.controller_status.v1") != 0)
        return 13;
    puts("controller payload layout (C11) ok");
    return 0;
}
