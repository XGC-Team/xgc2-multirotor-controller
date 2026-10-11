/* The payloads header is plain C11: it compiles here, and its layout asserts hold. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "multirotor_reference_trajectory/payloads.h"

int main(void) {
    xgc2_px4_reference_analytic_v1 analytic;
    xgc2_px4_reference_sampled_v1* sampled;
    xgc2_px4_reference_status_v1 status;
    memset(&analytic, 0, sizeof analytic);
    memset(&status, 0, sizeof status);
    analytic.params_len = XGC2_PX4_REFERENCE_MAX_PARAMS;
    analytic.params[XGC2_PX4_REFERENCE_MAX_PARAMS - 1] = 1.0;
    status.state = 4;
    if (sizeof analytic != 272 || sizeof status != 64)
        return 1;
    if (strcmp(XGC2_PX4_REFERENCE_ANALYTIC_SCHEMA, "xgc2.px4.reference_analytic.v1") != 0)
        return 2;
    if (strcmp(XGC2_PX4_REFERENCE_SAMPLED_SCHEMA, "xgc2.px4.reference_sampled.v1") != 0)
        return 3;
    if (strcmp(XGC2_PX4_REFERENCE_STATUS_SCHEMA, "xgc2.px4.reference_status.v1") != 0)
        return 4;
    if (strcmp(XGC2_PX4_REFERENCE_RESET_SCHEMA, "xgc2.px4.reference_reset.v1") != 0)
        return 5;
    sampled = (xgc2_px4_reference_sampled_v1*)calloc(1, sizeof *sampled);
    if (sampled == NULL)
        return 6;
    sampled->points_len = XGC2_PX4_REFERENCE_MAX_POINTS;
    sampled->points[XGC2_PX4_REFERENCE_MAX_POINTS - 1].yaw_accel = 2.0;
    free(sampled);
    puts("reference payload layout (C11) ok");
    return 0;
}
