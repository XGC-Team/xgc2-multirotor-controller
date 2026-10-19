#pragma once

#include <xgc2_math/control/se3_nmpc_problem.hpp>

#include "px4_multirotor_controller/common/sensor_checks.h"

namespace px4_multirotor_controller {

// Only the selected observation source changes. Both strategies consume the
// same declared control frame and preserve their existing numerical controller.
inline bool readControlFeedback(const SensorData& sensor, TrackingBackend backend,
                                xgc2_math::control::Se3State& state) {
    if (!sensor_checks::isControlStateUsableForControl(sensor))
        return false;
    state.position << sensor_checks::worldX(sensor, backend),
        sensor_checks::worldY(sensor, backend), sensor_checks::worldZ(sensor, backend);
    state.velocity << sensor_checks::worldVx(sensor, backend),
        sensor_checks::worldVy(sensor, backend), sensor_checks::worldVz(sensor, backend);
    state.attitude = Eigen::Quaterniond(
        sensor_checks::worldQw(sensor, backend), sensor_checks::worldQx(sensor, backend),
        sensor_checks::worldQy(sensor, backend), sensor_checks::worldQz(sensor, backend));
    if (!std::isfinite(state.attitude.norm()) || state.attitude.norm() < 1.0e-9)
        return false;
    state.attitude.normalize();
    if (sensor.feedback_source == FeedbackSource::MAVROS_LOCAL)
        state.body_rate = sensor.mavros_body_rate;
    else
        state.body_rate << sensor.wx, sensor.wy, sensor.wz;
    return state.position.allFinite() && state.velocity.allFinite() && state.body_rate.allFinite();
}

}  // namespace px4_multirotor_controller
