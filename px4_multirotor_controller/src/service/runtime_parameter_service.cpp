#include "px4_multirotor_controller/service/runtime_parameter_service.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <unordered_map>

namespace px4_multirotor_controller {
namespace {

using DoubleSetter = void (*)(ControllerConfig&, double);
using BoolSetter = void (*)(ControllerConfig&, bool);

bool parseDouble(const std::string& text, double& value) {
    char* end = nullptr;
    errno = 0;
    const double parsed = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

bool parseBool(const std::string& text, bool& value) {
    std::string normalized = text;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
        value = true;
        return true;
    }
    if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
        value = false;
        return true;
    }
    return false;
}

std::string formatDouble(double value) {
    std::ostringstream out;
    out.setf(std::ios::fixed, std::ios::floatfield);
    out.precision(6);
    out << value;
    return out.str();
}

std::string formatBool(bool value) {
    return value ? "true" : "false";
}

const std::unordered_map<std::string, DoubleSetter>& doubleSetters() {
    static const std::unordered_map<std::string, DoubleSetter> setters = {
        {"takeoff_altitude",
         [](ControllerConfig& cfg, double value) { cfg.takeoff_altitude = value; }},
        {"planning_period",
         [](ControllerConfig& cfg, double value) { cfg.planning_period = value; }},
        {"nmpc/control_period",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.control_period = value; }},
        {"nmpc/gravity", [](ControllerConfig& cfg, double value) { cfg.nmpc.gravity = value; }},
        {"nmpc/hover_thrust_ratio",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.hover_thrust_ratio = value; }},
        {"nmpc/min_hover_thrust",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.min_hover_thrust = value; }},
        {"nmpc/max_hover_thrust",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.max_hover_thrust = value; }},
        {"nmpc/specific_thrust_min",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.specific_thrust_min = value; }},
        {"nmpc/specific_thrust_max",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.specific_thrust_max = value; }},
        {"nmpc/hover_thrust_timeout",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.hover_thrust_timeout = value; }},
        {"nmpc/solve_timeout",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.solve_timeout = value; }},
        {"nmpc/result_timeout",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.result_timeout = value; }},
        {"nmpc/reference_timeout",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.reference_timeout = value; }},
        {"nmpc/reference_start_delay",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.reference_start_delay = value; }},
        {"nmpc/reference_duration",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.reference_duration = value; }},
        {"nmpc/reference_radius",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.reference_radius = value; }},
        {"nmpc/reference_line_speed",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.reference_line_speed = value; }},
        {"nmpc/reference_height",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.reference_height = value; }},
        {"nmpc/reference_z_amplitude",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.reference_z_amplitude = value; }},
        {"nmpc/reference_z_frequency",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.reference_z_frequency = value; }},
        {"nmpc/reference_entry_duration",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.reference_entry_duration = value; }},
        {"nmpc/log_period",
         [](ControllerConfig& cfg, double value) { cfg.nmpc.log_period = value; }},
        {"safety/fence_x_min",
         [](ControllerConfig& cfg, double value) { cfg.safety.fence_x_min = value; }},
        {"safety/fence_x_max",
         [](ControllerConfig& cfg, double value) { cfg.safety.fence_x_max = value; }},
        {"safety/fence_y_min",
         [](ControllerConfig& cfg, double value) { cfg.safety.fence_y_min = value; }},
        {"safety/fence_y_max",
         [](ControllerConfig& cfg, double value) { cfg.safety.fence_y_max = value; }},
        {"safety/fence_z_min",
         [](ControllerConfig& cfg, double value) { cfg.safety.fence_z_min = value; }},
        {"safety/fence_z_max",
         [](ControllerConfig& cfg, double value) { cfg.safety.fence_z_max = value; }},
        {"safety/max_velocity_xy",
         [](ControllerConfig& cfg, double value) { cfg.safety.max_velocity_xy = value; }},
        {"safety/max_velocity_z",
         [](ControllerConfig& cfg, double value) { cfg.safety.max_velocity_z = value; }},
        {"safety/acc_saturation_xy",
         [](ControllerConfig& cfg, double value) { cfg.safety.acc_saturation_xy = value; }},
        {"safety/acc_saturation_z",
         [](ControllerConfig& cfg, double value) { cfg.safety.acc_saturation_z = value; }},
        {"safety/position_jump_threshold",
         [](ControllerConfig& cfg, double value) { cfg.safety.position_jump_threshold = value; }},
    };
    return setters;
}

const std::unordered_map<std::string, BoolSetter>& boolSetters() {
    static const std::unordered_map<std::string, BoolSetter> setters = {
        {"skip_takeoff_init_disarm",
         [](ControllerConfig& cfg, bool value) { cfg.skip_takeoff_init_disarm = value; }},
        {"enable_yaw_control",
         [](ControllerConfig& cfg, bool value) { cfg.enable_yaw_control = value; }},
        {"nmpc/hover_thrust_enabled",
         [](ControllerConfig& cfg, bool value) { cfg.nmpc.hover_thrust_enabled = value; }},
        {"nmpc/enable_timing_log",
         [](ControllerConfig& cfg, bool value) { cfg.nmpc.enable_timing_log = value; }},
    };
    return setters;
}

}  // namespace

RuntimeParameterService::RuntimeParameterService(ros::NodeHandle& nh, DroneController& controller)
    : controller_(controller) {
    service_ =
        nh.advertiseService("set_runtime_parameters", &RuntimeParameterService::handle, this);
    ROS_INFO("[RuntimeParameterService] Ready: ~set_runtime_parameters");
}

bool RuntimeParameterService::handle(SetRuntimeParameters::Request& request,
                                     SetRuntimeParameters::Response& response) {
    if (request.names.size() != request.values.size()) {
        response.success = false;
        response.message = "names and values sizes differ";
        response.applied_values = currentValues(controller_.getConfig());
        return true;
    }

    ControllerConfig updated = controller_.getConfig();
    for (std::size_t i = 0; i < request.names.size(); ++i) {
        std::string error;
        if (!applyParameter(updated, request.names[i], request.values[i], error)) {
            response.success = false;
            response.message = error;
            response.applied_values = currentValues(controller_.getConfig());
            return true;
        }
    }

    std::string error;
    if (!validate(updated, error)) {
        response.success = false;
        response.message = error;
        response.applied_values = currentValues(controller_.getConfig());
        return true;
    }

    controller_.setConfig(updated);
    response.success = true;
    response.message = "runtime parameters updated";
    response.applied_values = currentValues(updated);

    ROS_INFO("[RuntimeParameterService] Updated %zu runtime parameter(s)", request.names.size());
    return true;
}

bool RuntimeParameterService::applyParameter(ControllerConfig& config, const std::string& name,
                                             const std::string& value, std::string& error) const {
    if (name == "nmpc/prediction_horizon") {
        error =
            "nmpc/prediction_horizon is generated into the acados solver and "
            "cannot be changed at runtime";
        return false;
    }

    const auto double_it = doubleSetters().find(name);
    if (double_it != doubleSetters().end()) {
        double parsed = 0.0;
        if (!parseDouble(value, parsed)) {
            error = "invalid double value for " + name + ": " + value;
            return false;
        }
        double_it->second(config, parsed);
        return true;
    }

    const auto bool_it = boolSetters().find(name);
    if (bool_it != boolSetters().end()) {
        bool parsed = false;
        if (!parseBool(value, parsed)) {
            error = "invalid bool value for " + name + ": " + value;
            return false;
        }
        bool_it->second(config, parsed);
        return true;
    }

    error = "unknown runtime parameter: " + name;
    return false;
}

bool RuntimeParameterService::validate(const ControllerConfig& config, std::string& error) const {
    const auto positive = [&error](double value, const char* name) {
        if (!std::isfinite(value) || value <= 0.0) {
            error = std::string(name) + " must be finite and > 0";
            return false;
        }
        return true;
    };

    if (!positive(config.takeoff_altitude, "takeoff_altitude") ||
        !positive(config.planning_period, "planning_period") ||
        !positive(config.nmpc.control_period, "nmpc/control_period") ||
        !positive(config.nmpc.gravity, "nmpc/gravity") ||
        !positive(config.nmpc.hover_thrust_timeout, "nmpc/hover_thrust_timeout") ||
        !positive(config.nmpc.solve_timeout, "nmpc/solve_timeout") ||
        !positive(config.nmpc.result_timeout, "nmpc/result_timeout") ||
        !positive(config.nmpc.reference_timeout, "nmpc/reference_timeout") ||
        !positive(config.nmpc.log_period, "nmpc/log_period")) {
        return false;
    }

    if (!std::isfinite(config.nmpc.reference_start_delay) ||
        config.nmpc.reference_start_delay < 0.0) {
        error = "nmpc/reference_start_delay must be finite and >= 0";
        return false;
    }
    if (!positive(config.nmpc.reference_duration, "nmpc/reference_duration") ||
        !positive(config.nmpc.reference_radius, "nmpc/reference_radius") ||
        !positive(config.nmpc.reference_height, "nmpc/reference_height") ||
        !positive(config.nmpc.reference_z_frequency, "nmpc/reference_z_frequency")) {
        return false;
    }
    if (!std::isfinite(config.nmpc.reference_line_speed) ||
        config.nmpc.reference_line_speed < 0.0) {
        error = "nmpc/reference_line_speed must be finite and >= 0";
        return false;
    }
    if (!std::isfinite(config.nmpc.reference_z_amplitude) ||
        config.nmpc.reference_z_amplitude < 0.0) {
        error = "nmpc/reference_z_amplitude must be finite and >= 0";
        return false;
    }
    if (!std::isfinite(config.nmpc.reference_entry_duration) ||
        config.nmpc.reference_entry_duration < 0.0) {
        error = "nmpc/reference_entry_duration must be finite and >= 0";
        return false;
    }
    if (config.nmpc.hover_thrust_ratio < 0.0 || config.nmpc.hover_thrust_ratio > 1.0 ||
        config.nmpc.min_hover_thrust < 0.0 || config.nmpc.max_hover_thrust > 1.0 ||
        config.nmpc.min_hover_thrust > config.nmpc.max_hover_thrust) {
        error = "hover thrust ratio and bounds must be inside [0, 1] and ordered";
        return false;
    }
    if (config.nmpc.specific_thrust_min < 0.0 ||
        config.nmpc.specific_thrust_min >= config.nmpc.specific_thrust_max) {
        error = "specific thrust bounds must be finite, non-negative, and ordered";
        return false;
    }
    if (config.safety.fence_x_min >= config.safety.fence_x_max ||
        config.safety.fence_y_min >= config.safety.fence_y_max ||
        config.safety.fence_z_min >= config.safety.fence_z_max) {
        error = "safety fence min values must be smaller than max values";
        return false;
    }
    if (!positive(config.safety.max_velocity_xy, "safety/max_velocity_xy") ||
        !positive(config.safety.max_velocity_z, "safety/max_velocity_z") ||
        !positive(config.safety.acc_saturation_xy, "safety/acc_saturation_xy") ||
        !positive(config.safety.acc_saturation_z, "safety/acc_saturation_z") ||
        !positive(config.safety.position_jump_threshold, "safety/position_jump_threshold")) {
        return false;
    }
    return true;
}

std::vector<std::string> RuntimeParameterService::currentValues(
    const ControllerConfig& config) const {
    return {
        "takeoff_altitude=" + formatDouble(config.takeoff_altitude),
        "planning_period=" + formatDouble(config.planning_period),
        "skip_takeoff_init_disarm=" + formatBool(config.skip_takeoff_init_disarm),
        "enable_yaw_control=" + formatBool(config.enable_yaw_control),
        "nmpc/control_period=" + formatDouble(config.nmpc.control_period),
        "nmpc/prediction_horizon=" + formatDouble(config.nmpc.prediction_horizon),
        "nmpc/gravity=" + formatDouble(config.nmpc.gravity),
        "nmpc/hover_thrust_ratio=" + formatDouble(config.nmpc.hover_thrust_ratio),
        "nmpc/min_hover_thrust=" + formatDouble(config.nmpc.min_hover_thrust),
        "nmpc/max_hover_thrust=" + formatDouble(config.nmpc.max_hover_thrust),
        "nmpc/specific_thrust_min=" + formatDouble(config.nmpc.specific_thrust_min),
        "nmpc/specific_thrust_max=" + formatDouble(config.nmpc.specific_thrust_max),
        "nmpc/hover_thrust_enabled=" + formatBool(config.nmpc.hover_thrust_enabled),
        "nmpc/hover_thrust_timeout=" + formatDouble(config.nmpc.hover_thrust_timeout),
        "nmpc/solve_timeout=" + formatDouble(config.nmpc.solve_timeout),
        "nmpc/result_timeout=" + formatDouble(config.nmpc.result_timeout),
        "nmpc/reference_timeout=" + formatDouble(config.nmpc.reference_timeout),
        "nmpc/reference_start_delay=" + formatDouble(config.nmpc.reference_start_delay),
        "nmpc/reference_duration=" + formatDouble(config.nmpc.reference_duration),
        "nmpc/reference_radius=" + formatDouble(config.nmpc.reference_radius),
        "nmpc/reference_line_speed=" + formatDouble(config.nmpc.reference_line_speed),
        "nmpc/reference_height=" + formatDouble(config.nmpc.reference_height),
        "nmpc/reference_z_amplitude=" + formatDouble(config.nmpc.reference_z_amplitude),
        "nmpc/reference_z_frequency=" + formatDouble(config.nmpc.reference_z_frequency),
        "nmpc/reference_entry_duration=" + formatDouble(config.nmpc.reference_entry_duration),
        "nmpc/enable_timing_log=" + formatBool(config.nmpc.enable_timing_log),
        "nmpc/log_period=" + formatDouble(config.nmpc.log_period),
        "safety/fence_x_min=" + formatDouble(config.safety.fence_x_min),
        "safety/fence_x_max=" + formatDouble(config.safety.fence_x_max),
        "safety/fence_y_min=" + formatDouble(config.safety.fence_y_min),
        "safety/fence_y_max=" + formatDouble(config.safety.fence_y_max),
        "safety/fence_z_min=" + formatDouble(config.safety.fence_z_min),
        "safety/fence_z_max=" + formatDouble(config.safety.fence_z_max),
        "safety/max_velocity_xy=" + formatDouble(config.safety.max_velocity_xy),
        "safety/max_velocity_z=" + formatDouble(config.safety.max_velocity_z),
        "safety/acc_saturation_xy=" + formatDouble(config.safety.acc_saturation_xy),
        "safety/acc_saturation_z=" + formatDouble(config.safety.acc_saturation_z),
        "safety/position_jump_threshold=" + formatDouble(config.safety.position_jump_threshold),
    };
}

}  // namespace px4_multirotor_controller
