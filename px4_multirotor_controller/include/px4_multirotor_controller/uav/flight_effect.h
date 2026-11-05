#pragma once

#include <cstdint>
#include <string>
#include <variant>

namespace px4_multirotor_controller {

// Actual ordered outputs of one controller turn. Values are consumed by the
// ROS and native edges after the turn, just as the previous output events were.
struct FlightSetpoint { double timestamp; };
struct FlightAttitude { double timestamp; };
struct FlightArmRequest { double timestamp; bool arm; };
struct FlightModeRequest { double timestamp; std::string mode; };
struct FlightNmpcRequest { double timestamp; uint64_t sequence; uint64_t generation; };
struct FlightReferenceActivation { double timestamp; };
struct FlightStatus { double timestamp; };
struct FlightSensorStats { double timestamp; };
struct FlightEvents { double timestamp; };
struct FlightTrackingError { double timestamp; };
struct FlightSensorDebug { double timestamp; };

using FlightEffect = std::variant<FlightSetpoint, FlightAttitude, FlightArmRequest,
                                  FlightModeRequest, FlightNmpcRequest,
                                  FlightReferenceActivation, FlightStatus,
                                  FlightSensorStats, FlightEvents,
                                  FlightTrackingError, FlightSensorDebug>;

}  // namespace px4_multirotor_controller
