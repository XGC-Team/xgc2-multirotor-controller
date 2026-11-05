#pragma once

#include <cstdint>

#include "px4_multirotor_controller/common/types.h"

namespace px4_multirotor_controller {

// Existing flight facts produced during one owner turn. Numeric IDs are the
// established alg/state_machine_events diagnostic values.
enum class FlightSignal : uint32_t {
    AltctlReady = event_type::ALTCTL_READY,
    OffboardReady = event_type::OFFBOARD_READY,
    ArmReady = event_type::ARM_READY,
    AltitudeReached = event_type::ALTITUDE_REACHED,
    TakeoffTimeout = event_type::TAKEOFF_TIMEOUT,
    ReferenceFinished = event_type::REFERENCE_TRAJECTORY_FINISHED,
    Touchdown = event_type::TOUCHDOWN,
    TimeoutLocalPosition = event_type::SAFE_TIMEOUT_LOCAL_POS,
    TimeoutLocalVelocity = event_type::SAFE_TIMEOUT_LOCAL_VELOCITY,
    TimeoutImu = event_type::SAFE_TIMEOUT_IMU,
    TimeoutFcuState = event_type::SAFE_TIMEOUT_STATE,
    TimeoutEstimate = event_type::SAFE_TIMEOUT_UAV_STATE_ESTIMATE,
    EstimateUnusable = event_type::SAFE_UAV_STATE_ESTIMATE_UNUSABLE,
    Geofence = event_type::SAFE_GEOFENCE_VIOLATION,
    VelocityXY = event_type::SAFE_VELOCITY_XY_EXCEEDED,
    VelocityZ = event_type::SAFE_VELOCITY_Z_EXCEEDED,
    SaturationXY = event_type::SAFE_CONTROL_SATURATION_XY,
    SaturationZ = event_type::SAFE_CONTROL_SATURATION_Z,
};

}  // namespace px4_multirotor_controller
