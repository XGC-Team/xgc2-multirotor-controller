#pragma once

// The controller's configuration from the JSON object of the module, with the reader and the
// validation of the ROS node (driver/controller_config.h). The module and its tests include this.

#include <module_support/json_config.hpp>
#include <stdexcept>
#include <string>
#include <variant>

#include "px4_multirotor_controller/driver/controller_config.h"

namespace px4_multirotor_controller {
namespace module {

// Keys that are absent keep the values of config/uav_nmpc.yaml, which is compiled into the core.
// Throws std::invalid_argument for a value of the wrong type, a missing world_boundary_json and a
// key that the controller does not read.
inline ControllerConfig readConfig(module_support::JsonConfig& json,
                                   ControllerParameters::Logger logger) {
    ControllerParameters parameters(
        [&json](const std::string& key, ControllerParameterValue& value) {
            return std::visit([&](auto& out) { return json.read(key, out); }, value);
        },
        std::move(logger));
    const ControllerConfig config = readControllerConfig(parameters);
    const std::string unknown = json.firstUnusedKey();
    if (!unknown.empty()) {
        throw std::invalid_argument("unknown configuration key '" + unknown + "'");
    }
    return config;
}

}  // namespace module
}  // namespace px4_multirotor_controller
