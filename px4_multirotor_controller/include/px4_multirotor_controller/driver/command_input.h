#pragma once
#include <optional>
#include <string>
#include <unordered_map>
namespace px4_multirotor_controller {
enum class FlightCommand { Takeoff, Land, Hover, Track };

inline std::optional<FlightCommand> parseFlightCommand(const std::string& text) {
    if (text.empty() || text.size() > 64) return std::nullopt;
    static const std::unordered_map<std::string, FlightCommand> commands = {
        {"takeoff", FlightCommand::Takeoff}, {"Takeoff", FlightCommand::Takeoff}, {"TAKEOFF", FlightCommand::Takeoff},
        {"land", FlightCommand::Land}, {"Land", FlightCommand::Land}, {"LAND", FlightCommand::Land},
        {"hover", FlightCommand::Hover}, {"Hover", FlightCommand::Hover}, {"HOVER", FlightCommand::Hover},
        {"custom1", FlightCommand::Track}, {"Custom1", FlightCommand::Track}, {"CUSTOM1", FlightCommand::Track},
        {"start", FlightCommand::Track}, {"Start", FlightCommand::Track}, {"START", FlightCommand::Track},
        {"track", FlightCommand::Track}, {"Track", FlightCommand::Track}, {"TRACK", FlightCommand::Track}};
    const auto found = commands.find(text);
    return found == commands.end() ? std::nullopt : std::optional<FlightCommand>(found->second);
}
}
