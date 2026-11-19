#pragma once

#include <cstdint>
#include <string>
#include <iomanip>
#include <sstream>

#include "px4_multirotor_controller/uav/flight_effect.h"

namespace px4_multirotor_controller {

// Identifies one normal request through the existing output FIFO. It carries
// no control authority and never substitutes for FCU telemetry.
struct NormalDisarmTrace {
    uint64_t sequence{0};
    double requested_at{0.0};
    std::string slot_namespace;
    ArmIntentSource source{ArmIntentSource::Other};
};

inline const char* normalDisarmSource(ArmIntentSource source) {
    switch (source) {
        case ArmIntentSource::ReadyEntry: return "ready-entry";
        case ArmIntentSource::LandingTouchdown: return "landing-touchdown";
        default: return "other";
    }
}

inline std::string normalDisarmIdentity(const NormalDisarmTrace& trace,
                                         const std::string& service,
                                         const std::string& node, long pid) {
    std::ostringstream out;
    out << "slot=" << trace.slot_namespace << " service=" << service
        << " source=" << normalDisarmSource(trace.source) << " request=" << trace.sequence
        << " intent_ros_s=" << std::fixed << std::setprecision(9) << trace.requested_at
        << " node=" << node << " pid=" << pid;
    return out.str();
}

}  // namespace px4_multirotor_controller
