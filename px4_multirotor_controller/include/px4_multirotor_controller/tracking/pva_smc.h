#pragma once

#include <cmath>
#include <xgc2_math/geometry/math_helpers.h>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

// The v1 feedback law, applied once to the lifted feedforward acceleration.
// Both the pair and the reference are in the controller's local ENU frame.
inline Setpoint pvaSmcSetpoint(const Setpoint& reference, const SensorData::LocalPv& measured,
                               const ControllerConfig& config) {
    using xgc2_math::math_helpers::sign;
    using xgc2_math::math_helpers::signBeta;
    const Eigen::Vector3d ep = measured.position - Eigen::Vector3d(reference.x, reference.y, reference.z);
    const Eigen::Vector3d ev = measured.velocity - Eigen::Vector3d(reference.vx, reference.vy, reference.vz);
    Eigen::Vector3d feedback;
    for (int axis = 0; axis < 3; ++axis) {
        const double k1 = config.smc.k1Axis(axis);
        const double k2 = config.smc.k2Axis(axis);
        const double rho = config.smc.boundaryLayerAxis(axis);
        const double slide =
            ev[axis] + signBeta(signBeta(ev[axis], 2.0) + k2 * std::atan(ep[axis]), 0.5);
        const double saturation = std::abs(slide) > rho ? sign(slide) : slide / rho;
        feedback[axis] = -k1 * saturation;
    }
    Setpoint output = reference;
    output.ax += feedback.x();
    output.ay += feedback.y();
    output.az += feedback.z();
    output.coordinate_frame = 1;
    output.type_mask = kDefaultPvaLocalTypeMask;
    if (config.enable_yaw_control)
        output.type_mask &= static_cast<uint16_t>(~kIgnoreYawBit);
    return output;
}

}  // namespace px4_multirotor_controller
