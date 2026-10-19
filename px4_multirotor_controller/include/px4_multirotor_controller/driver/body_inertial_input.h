#pragma once

#include <Eigen/Geometry>
#include <cmath>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

// Body FLU specific force includes gravity. The quaternion rotates body FLU to
// the declared local ENU control frame. No estimator state or bias is invented.
struct BodyInertialInput {
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
    Eigen::Vector3d specific_force{Eigen::Vector3d::Zero()};
    double stamp_sec{0.0};
};

inline bool ingestBodyInertial(SensorData& sensor, const BodyInertialInput& input,
                               const double gravity) {
    const double norm = input.orientation.norm();
    sensor.mavros_inertial_valid =
        input.body_rate.allFinite() && input.specific_force.allFinite() &&
        input.orientation.coeffs().allFinite() && std::isfinite(norm) && norm > 1.0e-9 &&
        std::isfinite(input.stamp_sec) && std::isfinite(gravity);
    if (!sensor.mavros_inertial_valid)
        return false;
    const Eigen::Quaterniond orientation = input.orientation.normalized();
    sensor.mavros_body_rate = input.body_rate;
    sensor.mavros_world_acceleration =
        orientation * input.specific_force - gravity * Eigen::Vector3d::UnitZ();
    sensor.mavros_imu_stamp = input.stamp_sec;
    return true;
}

}  // namespace px4_multirotor_controller
