#pragma once
#include <optional>
#include <string>
#include <state_machine/state_machine.hpp>
#include <unordered_map>
#include "px4_multirotor_controller/common/types.h"
namespace px4_multirotor_controller {
inline std::optional<::state_machine::EventId> commandInputEvent(const std::string& text) {
    if (text.empty() || text.size() > 64) return std::nullopt;
    static const std::unordered_map<std::string, ::state_machine::EventId> commands = {
        {"takeoff", event_type::TAKEOFF_REQUESTED}, {"Takeoff", event_type::TAKEOFF_REQUESTED}, {"TAKEOFF", event_type::TAKEOFF_REQUESTED},
        {"land", event_type::LANDING_REQUESTED}, {"Land", event_type::LANDING_REQUESTED}, {"LAND", event_type::LANDING_REQUESTED},
        {"hover", event_type::HOVER_REQUESTED}, {"Hover", event_type::HOVER_REQUESTED}, {"HOVER", event_type::HOVER_REQUESTED},
        {"custom1", event_type::TRAJECTORY_TRACKING_REQUESTED}, {"Custom1", event_type::TRAJECTORY_TRACKING_REQUESTED}, {"CUSTOM1", event_type::TRAJECTORY_TRACKING_REQUESTED},
        {"start", event_type::TRAJECTORY_TRACKING_REQUESTED}, {"Start", event_type::TRAJECTORY_TRACKING_REQUESTED}, {"START", event_type::TRAJECTORY_TRACKING_REQUESTED},
        {"track", event_type::TRAJECTORY_TRACKING_REQUESTED}, {"Track", event_type::TRAJECTORY_TRACKING_REQUESTED}, {"TRACK", event_type::TRAJECTORY_TRACKING_REQUESTED}};
    const auto found = commands.find(text);
    return found == commands.end() ? std::nullopt : std::optional<::state_machine::EventId>(found->second);
}
}
