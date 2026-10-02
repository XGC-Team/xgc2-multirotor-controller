#include "px4_multirotor_controller/driver/controller_config.h"
#include "controller_profile.h"
#include "px4_multirotor_controller/drone_controller.h"
#include "xgc2_math/geometry/math_helpers.h"
#include <jsoncpp/json/json.h>
#include <sstream>
#include <cmath>
#include <stdexcept>
namespace px4_multirotor_controller {
ControllerParameters::ControllerParameters(Getter getter, Logger logger, bool owning_profile)
 : override_(std::move(getter)), logger_(std::move(logger)) {
 if (!owning_profile) return; // ROS has already loaded its authored YAML into the private namespace.
 Json::Value profile;
 Json::CharReaderBuilder builder;
 std::string error;
 std::istringstream input(kControllerProfile);
 if (!Json::parseFromStream(builder, input, &profile, &error)) throw std::invalid_argument(error);
 profile_ = [profile](const std::string& key, ControllerParameterValue& value) {
   const Json::Value* node = &profile;
   size_t offset = 0;
   for (;;) {
     const size_t slash = key.find('/', offset);
     const std::string part = key.substr(offset, slash - offset);
     if (!node->isObject() || !node->isMember(part)) return false;
     node = &(*node)[part];
     if (slash == std::string::npos) break;
     offset = slash + 1;
   }
   std::visit([node](auto& out) {
     using T = std::decay_t<decltype(out)>;
     if constexpr (std::is_same_v<T, bool>) out = node->asBool();
     else if constexpr (std::is_same_v<T, int>) out = node->asInt();
     else if constexpr (std::is_same_v<T, double>) out = node->asDouble();
     else if constexpr (std::is_same_v<T, std::string>) out = node->asString();
     else { out.clear(); for (const auto& v : *node) out.push_back(v.asDouble()); }
   }, value);
   return true;
 };
}
bool ControllerParameters::read(const std::string& key, ControllerParameterValue& value) const {
 if (override_ && override_(key, value)) return true;
 return profile_ ? profile_(key, value) : false;
}
template<class T> void readParamWithLog(const ControllerParameters& p, const std::string& key, T& out, const char*) {
 (void)p.getParam(key, out);
}
Eigen::Vector3d getVector3Param(const ControllerParameters& nh, const std::string& name,
                                const Eigen::Vector3d& fallback) {
    std::vector<double> values;
    if (!nh.getParam(name, values)) {
        return fallback;
    }
    if (values.size() != 3U) {
        nh.logf(true, "[DroneRosNode] Parameter %s must have 3 values; keeping [%.3f %.3f %.3f]",
                 name.c_str(), fallback.x(), fallback.y(), fallback.z());
        return fallback;
    }
    Eigen::Vector3d result(values[0], values[1], values[2]);
    if (!result.array().isFinite().all()) {
        nh.logf(true, "[DroneRosNode] Parameter %s contains non-finite values; keeping [%.3f %.3f %.3f]",
                 name.c_str(), fallback.x(), fallback.y(), fallback.z());
        return fallback;
    }
    nh.logf(false, "[DroneRosNode] %s: [%.3f %.3f %.3f]", name.c_str(), result.x(), result.y(),
             result.z());
    return result;
}

ControllerConfig readControllerConfig(const ControllerParameters& parameters, const std::string& uav_name) {

    // 读取私有参数（使用私有命名空间句柄）
    ControllerConfig config;
    std::string boundary_json;
    if (!parameters.getParam("world_boundary_json", boundary_json)) {
        throw std::invalid_argument(
            "world_boundary_json must explicitly contain worldBoundary or null");
    }
    config.safety.world_boundary = parseWorldBoundary(boundary_json);
    readParamWithLog(parameters, "takeoff_altitude", config.takeoff_altitude,
                                "Takeoff altitude (m)");
    
    double per_uav_takeoff_altitude = config.takeoff_altitude;
    if (!uav_name.empty() &&
        parameters.getParam("takeoff_altitudes/" + uav_name, per_uav_takeoff_altitude)) {
        if (std::isfinite(per_uav_takeoff_altitude) && per_uav_takeoff_altitude > 0.0) {
            config.takeoff_altitude = per_uav_takeoff_altitude;
            parameters.logf(false, "[DroneRosNode] Per-UAV takeoff altitude for %s: %.3f m", uav_name.c_str(),
                     config.takeoff_altitude);
        } else {
            parameters.logf(true, 
                "[DroneRosNode] Invalid per-UAV takeoff altitude for %s: %.3f, "
                "keeping %.3f m",
                uav_name.c_str(), per_uav_takeoff_altitude, config.takeoff_altitude);
        }
    }
    parameters.param("skip_takeoff_init_disarm", config.skip_takeoff_init_disarm,
                      config.skip_takeoff_init_disarm);
    parameters.logf(false, "[DroneRosNode] Skip TakeoffInit DISARM and ALTCTL gate: %s",
             config.skip_takeoff_init_disarm ? "enabled" : "disabled");

    parameters.param("planning_period", config.planning_period, config.planning_period);
    if (!std::isfinite(config.planning_period) || config.planning_period <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid planning_period; using 0.100 s");
        config.planning_period = 0.1;
    }
    parameters.logf(false, "[DroneRosNode] MPC planning period: %.3f s", config.planning_period);

    parameters.logf(false, "[DroneRosNode] Control state source: state_estimator (fusion only)");

    std::string tracking_backend = "px4_local";
    parameters.param("tracking_backend", tracking_backend, tracking_backend);
    if (tracking_backend == "px4_local") {
        config.tracking_backend = TrackingBackend::PX4_LOCAL;
    } else if (tracking_backend == "nmpc") {
        config.tracking_backend = TrackingBackend::NMPC;
    } else if (tracking_backend == "dfbc") {
        config.tracking_backend = TrackingBackend::DFBC;
    } else if (tracking_backend == "smc") {
        config.tracking_backend = TrackingBackend::SMC;
    } else {
        parameters.logf(true, "[DroneRosNode] Unknown tracking_backend=%s, using px4_local",
                 tracking_backend.c_str());
        config.tracking_backend = TrackingBackend::PX4_LOCAL;
        tracking_backend = "px4_local";
    }
    parameters.logf(false, "[DroneRosNode] Tracking backend: %s", tracking_backend.c_str());

    std::string px4_local_lift = "legacy";
    parameters.param("px4_local_lift", px4_local_lift, px4_local_lift);
    if (px4_local_lift == "legacy") {
        config.px4_local_lift = Px4LocalLiftMode::Legacy;
    } else if (px4_local_lift == "zero_order_hold") {
        config.px4_local_lift = Px4LocalLiftMode::ZeroOrderHold;
    } else {
        parameters.logf(true, "[DroneRosNode] Unknown px4_local_lift=%s, using legacy", px4_local_lift.c_str());
        config.px4_local_lift = Px4LocalLiftMode::Legacy;
        px4_local_lift = "legacy";
    }
    parameters.param("smc/k1", config.smc.k1, config.smc.k1);
    parameters.param("smc/k2", config.smc.k2, config.smc.k2);
    parameters.param("smc/boundary_layer", config.smc.boundary_layer, config.smc.boundary_layer);

    int local_type_mask = static_cast<int>(config.local_type_mask);
    parameters.param("local_type_mask", local_type_mask, local_type_mask);
    if (local_type_mask < 0 || local_type_mask > 4095) {
        parameters.logf(true, "[DroneRosNode] Invalid local_type_mask=%d, using %u", local_type_mask,
                 static_cast<unsigned>(kDefaultPvaLocalTypeMask));
        config.local_type_mask = kDefaultPvaLocalTypeMask;
    } else {
        config.local_type_mask = static_cast<uint16_t>(local_type_mask);
    }

    // ========== 偏航角控制开关 ==========
    readParamWithLog(parameters, "enable_yaw_control", config.enable_yaw_control,
                                "Enable yaw control");

    // ========== DFBC attitude-rate 策略参数 ==========
    config.dfbc.position_natural_frequency = getVector3Param(
        parameters, "dfbc/position_natural_frequency", config.dfbc.position_natural_frequency);
    config.dfbc.position_damping_ratio = getVector3Param(parameters, "dfbc/position_damping_ratio",
                                                         config.dfbc.position_damping_ratio);
    parameters.param("dfbc/tilt_gain", config.dfbc.tilt_gain, config.dfbc.tilt_gain);
    parameters.param("dfbc/tilt_rate_damping", config.dfbc.tilt_rate_damping,
                      config.dfbc.tilt_rate_damping);
    parameters.param("dfbc/yaw_gain", config.dfbc.yaw_gain, config.dfbc.yaw_gain);
    parameters.param("dfbc/yaw_rate_damping", config.dfbc.yaw_rate_damping,
                      config.dfbc.yaw_rate_damping);
    parameters.param("dfbc/use_body_rate_feedforward", config.dfbc.use_body_rate_feedforward,
                      config.dfbc.use_body_rate_feedforward);
    parameters.param("dfbc/acceleration_correction_enabled",
                      config.dfbc.acceleration_correction_enabled,
                      config.dfbc.acceleration_correction_enabled);
    config.dfbc.acceleration_correction_gain = getVector3Param(
        parameters, "dfbc/acceleration_correction_gain", config.dfbc.acceleration_correction_gain);
    config.dfbc.acceleration_correction_limit =
        getVector3Param(parameters, "dfbc/acceleration_correction_limit",
                        config.dfbc.acceleration_correction_limit);
    parameters.param("dfbc/acceleration_correction_filter_tau",
                      config.dfbc.acceleration_correction_filter_tau,
                      config.dfbc.acceleration_correction_filter_tau);
    parameters.param("dfbc/acceleration_measurement_timeout",
                      config.dfbc.acceleration_measurement_timeout,
                      config.dfbc.acceleration_measurement_timeout);
    parameters.param("dfbc/log_period", config.dfbc.log_period, config.dfbc.log_period);

    // ========== UAV NMPC 后端参数 ==========
    parameters.param("nmpc/control_period", config.nmpc.control_period,
                      config.nmpc.control_period);
    parameters.param("nmpc/prediction_horizon", config.nmpc.prediction_horizon,
                      config.nmpc.prediction_horizon);
    parameters.param("nmpc/body_rate_time_constant", config.nmpc.body_rate_time_constant,
                      config.nmpc.body_rate_time_constant);
    parameters.param("nmpc/gravity", config.nmpc.gravity, config.nmpc.gravity);
    parameters.param("nmpc/hover_thrust_ratio", config.nmpc.hover_thrust_ratio,
                      config.nmpc.hover_thrust_ratio);
    parameters.param("nmpc/min_hover_thrust", config.nmpc.min_hover_thrust,
                      config.nmpc.min_hover_thrust);
    parameters.param("nmpc/max_hover_thrust", config.nmpc.max_hover_thrust,
                      config.nmpc.max_hover_thrust);
    parameters.param("nmpc/normalized_thrust_min", config.nmpc.normalized_thrust_min,
                      config.nmpc.normalized_thrust_min);
    parameters.param("nmpc/normalized_thrust_max", config.nmpc.normalized_thrust_max,
                      config.nmpc.normalized_thrust_max);
    parameters.param("nmpc/max_roll_pitch_body_rate", config.nmpc.max_roll_pitch_body_rate,
                      config.nmpc.max_roll_pitch_body_rate);
    parameters.param("nmpc/max_yaw_body_rate", config.nmpc.max_yaw_body_rate,
                      config.nmpc.max_yaw_body_rate);
    parameters.param("nmpc/max_roll_pitch_angular_acceleration",
                      config.nmpc.max_roll_pitch_angular_acceleration,
                      config.nmpc.max_roll_pitch_angular_acceleration);
    parameters.param("nmpc/max_yaw_angular_acceleration", config.nmpc.max_yaw_angular_acceleration,
                      config.nmpc.max_yaw_angular_acceleration);
    config.nmpc.angular_acceleration_weight = getVector3Param(
        parameters, "nmpc/angular_acceleration_weight", config.nmpc.angular_acceleration_weight);
    parameters.param("nmpc/enable_timing_log", config.nmpc.enable_timing_log,
                      config.nmpc.enable_timing_log);
    parameters.param("nmpc/log_period", config.nmpc.log_period, config.nmpc.log_period);

    parameters.param("hover_thrust/enabled", config.nmpc.hover_thrust_enabled,
                      config.nmpc.hover_thrust_enabled);
    parameters.param("nmpc/hover_thrust_enabled", config.nmpc.hover_thrust_enabled,
                      config.nmpc.hover_thrust_enabled);
    parameters.param("hover_thrust/timeout", config.nmpc.hover_thrust_timeout,
                      config.nmpc.hover_thrust_timeout);
    parameters.param("nmpc/hover_thrust_timeout", config.nmpc.hover_thrust_timeout,
                      config.nmpc.hover_thrust_timeout);
    parameters.param("nmpc/solve_timeout", config.nmpc.solve_timeout, config.nmpc.solve_timeout);
    parameters.param("nmpc/result_timeout", config.nmpc.result_timeout,
                      config.nmpc.result_timeout);
    parameters.param("nmpc/reference_start_delay", config.nmpc.reference_start_delay,
                      config.nmpc.reference_start_delay);
    parameters.param("nmpc/reference_duration", config.nmpc.reference_duration,
                      config.nmpc.reference_duration);
    parameters.param("nmpc/reference_radius", config.nmpc.reference_radius,
                      config.nmpc.reference_radius);
    parameters.param("nmpc/reference_line_speed", config.nmpc.reference_line_speed,
                      config.nmpc.reference_line_speed);
    parameters.param("nmpc/reference_height", config.nmpc.reference_height,
                      config.nmpc.reference_height);
    parameters.param("nmpc/reference_z_amplitude", config.nmpc.reference_z_amplitude,
                      config.nmpc.reference_z_amplitude);
    parameters.param("nmpc/reference_z_frequency", config.nmpc.reference_z_frequency,
                      config.nmpc.reference_z_frequency);
    parameters.param("nmpc/reference_entry_duration", config.nmpc.reference_entry_duration,
                      config.nmpc.reference_entry_duration);
    parameters.param("nmpc/reference_analytic_type", config.nmpc.reference_analytic_type,
                      config.nmpc.reference_analytic_type);
    parameters.param("nmpc/reference_torus_omega", config.nmpc.reference_torus_omega,
                      config.nmpc.reference_torus_omega);
    parameters.param("nmpc/reference_torus_scale", config.nmpc.reference_torus_scale,
                      config.nmpc.reference_torus_scale);

    if (!std::isfinite(config.nmpc.control_period) || config.nmpc.control_period <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/control_period; using 0.010 s");
        config.nmpc.control_period = 0.01;
    }
    if (!std::isfinite(config.nmpc.prediction_horizon) || config.nmpc.prediction_horizon <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/prediction_horizon; using 1.000 s");
        config.nmpc.prediction_horizon = 1.0;
    }
    if (!std::isfinite(config.nmpc.body_rate_time_constant) ||
        config.nmpc.body_rate_time_constant <= 1.0e-6) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/body_rate_time_constant; using 0.080 s");
        config.nmpc.body_rate_time_constant = 0.08;
    }
    if (!std::isfinite(config.nmpc.gravity) || config.nmpc.gravity <= 1e-6) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/gravity; using 9.8066");
        config.nmpc.gravity = 9.8066;
    }
    if (!std::isfinite(config.nmpc.max_roll_pitch_body_rate) ||
        config.nmpc.max_roll_pitch_body_rate <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/max_roll_pitch_body_rate; using 3.491 rad/s");
        config.nmpc.max_roll_pitch_body_rate = 3.4906585;
    }
    if (!std::isfinite(config.nmpc.max_yaw_body_rate) || config.nmpc.max_yaw_body_rate <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/max_yaw_body_rate; using 0.873 rad/s");
        config.nmpc.max_yaw_body_rate = 0.8726646;
    }
    if (!std::isfinite(config.nmpc.max_roll_pitch_angular_acceleration) ||
        config.nmpc.max_roll_pitch_angular_acceleration <= 0.0) {
        parameters.logf(true, 
            "[DroneRosNode] Invalid nmpc/max_roll_pitch_angular_acceleration; using 15.000 "
            "rad/s^2");
        config.nmpc.max_roll_pitch_angular_acceleration = 15.0;
    }
    if (!std::isfinite(config.nmpc.max_yaw_angular_acceleration) ||
        config.nmpc.max_yaw_angular_acceleration <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/max_yaw_angular_acceleration; using 2.000 rad/s^2");
        config.nmpc.max_yaw_angular_acceleration = 2.0;
    }
    if (!config.nmpc.angular_acceleration_weight.array().isFinite().all() ||
        (config.nmpc.angular_acceleration_weight.array() <= 0.0).any()) {
        parameters.logf(true, 
            "[DroneRosNode] Invalid nmpc/angular_acceleration_weight; using "
            "[0.04 0.04 2.25]");
        config.nmpc.angular_acceleration_weight = Eigen::Vector3d(0.04, 0.04, 2.25);
    }
    config.nmpc.hover_thrust_ratio =
        xgc2_math::math_helpers::clamp(config.nmpc.hover_thrust_ratio, 0.05, 0.95);
    config.nmpc.min_hover_thrust =
        xgc2_math::math_helpers::clamp(config.nmpc.min_hover_thrust, 0.0, 1.0);
    config.nmpc.max_hover_thrust = xgc2_math::math_helpers::clamp(
        config.nmpc.max_hover_thrust, config.nmpc.min_hover_thrust, 1.0);
    config.nmpc.normalized_thrust_min =
        xgc2_math::math_helpers::clamp(config.nmpc.normalized_thrust_min, 0.0, 1.0);
    config.nmpc.normalized_thrust_max = xgc2_math::math_helpers::clamp(
        config.nmpc.normalized_thrust_max, config.nmpc.normalized_thrust_min, 1.0);
    if (!std::isfinite(config.nmpc.hover_thrust_timeout) ||
        config.nmpc.hover_thrust_timeout <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid hover_thrust timeout; using 0.500 s");
        config.nmpc.hover_thrust_timeout = 0.5;
    }
    if (!std::isfinite(config.nmpc.solve_timeout) || config.nmpc.solve_timeout <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/solve_timeout; using 0.030 s");
        config.nmpc.solve_timeout = 0.03;
    }
    if (!std::isfinite(config.nmpc.result_timeout) || config.nmpc.result_timeout <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/result_timeout; using 0.100 s");
        config.nmpc.result_timeout = 0.1;
    }
    if (!std::isfinite(config.nmpc.reference_start_delay) ||
        config.nmpc.reference_start_delay < 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_start_delay; using 0.200 s");
        config.nmpc.reference_start_delay = 0.2;
    }
    if (!std::isfinite(config.nmpc.reference_duration) || config.nmpc.reference_duration <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_duration; using 60.000 s");
        config.nmpc.reference_duration = 60.0;
    }
    if (!std::isfinite(config.nmpc.reference_radius) || config.nmpc.reference_radius <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_radius; using 3.000 m");
        config.nmpc.reference_radius = 3.0;
    }
    if (!std::isfinite(config.nmpc.reference_line_speed) ||
        config.nmpc.reference_line_speed < 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_line_speed; using 1.000 m/s");
        config.nmpc.reference_line_speed = 1.0;
    }
    if (!std::isfinite(config.nmpc.reference_height) || config.nmpc.reference_height <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_height; using 3.000 m");
        config.nmpc.reference_height = 3.0;
    }
    if (!std::isfinite(config.nmpc.reference_z_amplitude) ||
        config.nmpc.reference_z_amplitude < 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_z_amplitude; using 0.000 m");
        config.nmpc.reference_z_amplitude = 0.0;
    }
    if (!std::isfinite(config.nmpc.reference_z_frequency) ||
        config.nmpc.reference_z_frequency <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_z_frequency; using 0.500 rad/s");
        config.nmpc.reference_z_frequency = 0.5;
    }
    if (!std::isfinite(config.nmpc.reference_entry_duration) ||
        config.nmpc.reference_entry_duration < 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_entry_duration; using 5.000 s");
        config.nmpc.reference_entry_duration = 5.0;
    }
    if (config.nmpc.reference_analytic_type < 0 || config.nmpc.reference_analytic_type > 9) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_analytic_type; using circle-entry");
        config.nmpc.reference_analytic_type = 3;
    }
    if (!std::isfinite(config.nmpc.reference_torus_omega) ||
        config.nmpc.reference_torus_omega <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_torus_omega; using 0.300 rad/s");
        config.nmpc.reference_torus_omega = 0.3;
    }
    if (!std::isfinite(config.nmpc.reference_torus_scale) ||
        config.nmpc.reference_torus_scale <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid nmpc/reference_torus_scale; using 2.000 m");
        config.nmpc.reference_torus_scale = 2.0;
    }
    if (!config.dfbc.position_natural_frequency.array().isFinite().all() ||
        (config.dfbc.position_natural_frequency.array() <= 0.0).any()) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/position_natural_frequency; using [2.0 2.0 2.2]");
        config.dfbc.position_natural_frequency = Eigen::Vector3d(2.0, 2.0, 2.2);
    }
    if (!config.dfbc.position_damping_ratio.array().isFinite().all() ||
        (config.dfbc.position_damping_ratio.array() <= 0.0).any()) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/position_damping_ratio; using [0.9 0.9 1.0]");
        config.dfbc.position_damping_ratio = Eigen::Vector3d(0.9, 0.9, 1.0);
    }
    if (!std::isfinite(config.dfbc.tilt_gain) || config.dfbc.tilt_gain <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/tilt_gain; using 6.000");
        config.dfbc.tilt_gain = 6.0;
    }
    if (!std::isfinite(config.dfbc.tilt_rate_damping) || config.dfbc.tilt_rate_damping < 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/tilt_rate_damping; using 1.000");
        config.dfbc.tilt_rate_damping = 1.0;
    }
    if (!std::isfinite(config.dfbc.yaw_gain) || config.dfbc.yaw_gain < 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/yaw_gain; using 0.300");
        config.dfbc.yaw_gain = 0.3;
    }
    if (!std::isfinite(config.dfbc.yaw_rate_damping) || config.dfbc.yaw_rate_damping < 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/yaw_rate_damping; using 0.200");
        config.dfbc.yaw_rate_damping = 0.2;
    }
    if (!config.dfbc.acceleration_correction_gain.array().isFinite().all() ||
        (config.dfbc.acceleration_correction_gain.array() < 0.0).any()) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/acceleration_correction_gain; using [0.35 0.35 0.0]");
        config.dfbc.acceleration_correction_gain = Eigen::Vector3d(0.35, 0.35, 0.0);
    }
    if (!config.dfbc.acceleration_correction_limit.array().isFinite().all() ||
        (config.dfbc.acceleration_correction_limit.array() < 0.0).any()) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/acceleration_correction_limit; using [2.0 2.0 0.0]");
        config.dfbc.acceleration_correction_limit = Eigen::Vector3d(2.0, 2.0, 0.0);
    }
    if (!std::isfinite(config.dfbc.acceleration_correction_filter_tau) ||
        config.dfbc.acceleration_correction_filter_tau < 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/acceleration_correction_filter_tau; using 0.000 s");
        config.dfbc.acceleration_correction_filter_tau = 0.0;
    }
    if (!std::isfinite(config.dfbc.acceleration_measurement_timeout) ||
        config.dfbc.acceleration_measurement_timeout <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/acceleration_measurement_timeout; using 0.050 s");
        config.dfbc.acceleration_measurement_timeout = 0.05;
    }
    if (!std::isfinite(config.dfbc.log_period) || config.dfbc.log_period <= 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid dfbc/log_period; using 1.000 s");
        config.dfbc.log_period = 1.0;
    }
    if (config.tracking_backend == TrackingBackend::NMPC ||
        config.tracking_backend == TrackingBackend::DFBC) {
        if (!config.nmpc.hover_thrust_enabled) {
            parameters.logf(true, 
                "[DroneRosNode] Attitude-rate tracking requires hover thrust estimate; "
                "forcing hover_thrust/enabled=true");
            config.nmpc.hover_thrust_enabled = true;
        }
    }
    if (config.tracking_backend == TrackingBackend::NMPC) {
        parameters.logf(false, 
            "[DroneRosNode] UAV NMPC: dt=%.3f horizon=%.3f gravity=%.4f "
            "hover=%.3f estimator=required hover_timeout=%.3f "
            "rate_tau=%.3f thrust_norm=[%.2f, %.2f] alpha_max=[roll_pitch %.2f yaw %.2f] "
            "W_alpha=[%.3f %.3f %.3f] "
            "body_rate_max=[roll_pitch %.2f yaw %.2f] "
            "solve_timeout=%.3f "
            "reference_type=%d circle_entry=[radius %.2f speed %.2f height %.2f z_amp %.2f] "
            "torus=[omega %.2f scale %.2f]",
            config.nmpc.control_period, config.nmpc.prediction_horizon, config.nmpc.gravity,
            config.nmpc.hover_thrust_ratio, config.nmpc.hover_thrust_timeout,
            config.nmpc.body_rate_time_constant, config.nmpc.normalized_thrust_min,
            config.nmpc.normalized_thrust_max, config.nmpc.max_roll_pitch_angular_acceleration,
            config.nmpc.max_yaw_angular_acceleration, config.nmpc.angular_acceleration_weight.x(),
            config.nmpc.angular_acceleration_weight.y(),
            config.nmpc.angular_acceleration_weight.z(), config.nmpc.max_roll_pitch_body_rate,
            config.nmpc.max_yaw_body_rate, config.nmpc.solve_timeout,
            config.nmpc.reference_analytic_type, config.nmpc.reference_radius,
            config.nmpc.reference_line_speed, config.nmpc.reference_height,
            config.nmpc.reference_z_amplitude, config.nmpc.reference_torus_omega,
            config.nmpc.reference_torus_scale);
    } else if (config.tracking_backend == TrackingBackend::DFBC) {
        parameters.logf(false, 
            "[DroneRosNode] UAV DFBC attitude-rate: dt=%.3f gravity=%.4f hover=required "
            "thrust_norm=[%.2f, %.2f] body_rate_max=[roll_pitch %.2f yaw %.2f] "
            "wn=[%.2f %.2f %.2f] zeta=[%.2f %.2f %.2f] tilt_gain=%.2f yaw_gain=%.2f "
            "feedforward=%s accel_fix=%s gain=[%.2f %.2f %.2f] limit=[%.2f %.2f %.2f] tau=%.3f",
            config.nmpc.control_period, config.nmpc.gravity, config.nmpc.normalized_thrust_min,
            config.nmpc.normalized_thrust_max, config.nmpc.max_roll_pitch_body_rate,
            config.nmpc.max_yaw_body_rate, config.dfbc.position_natural_frequency.x(),
            config.dfbc.position_natural_frequency.y(), config.dfbc.position_natural_frequency.z(),
            config.dfbc.position_damping_ratio.x(), config.dfbc.position_damping_ratio.y(),
            config.dfbc.position_damping_ratio.z(), config.dfbc.tilt_gain, config.dfbc.yaw_gain,
            config.dfbc.use_body_rate_feedforward ? "true" : "false",
            config.dfbc.acceleration_correction_enabled ? "true" : "false",
            config.dfbc.acceleration_correction_gain.x(),
            config.dfbc.acceleration_correction_gain.y(),
            config.dfbc.acceleration_correction_gain.z(),
            config.dfbc.acceleration_correction_limit.x(),
            config.dfbc.acceleration_correction_limit.y(),
            config.dfbc.acceleration_correction_limit.z(),
            config.dfbc.acceleration_correction_filter_tau);
    } else if (config.tracking_backend == TrackingBackend::SMC) {
        parameters.logf(false, 
            "[DroneRosNode] UAV SMC acceleration: dt=%.3f k1=%.3f k2=%.3f rho=%.4f "
            "feedback=mavros_local mask=%u frame=1",
            config.nmpc.control_period, config.smc.k1, config.smc.k2, config.smc.boundary_layer,
            static_cast<unsigned>(kSmcAccelerationTypeMask));
    } else if (config.tracking_backend == TrackingBackend::PX4_LOCAL) {
        parameters.logf(false, "[DroneRosNode] UAV PX4 local pass-through: default_mask=%u yaw=%s lift=%s",
                 static_cast<unsigned>(config.local_type_mask),
                 config.enable_yaw_control ? "true" : "false", px4_local_lift.c_str());
    }

    // ========== 安全限制参数 ==========
    // The explicit worldBoundary above is the only geofence configuration.

    // 位置跳变检测
    readParamWithLog(parameters, "position_jump_threshold",
                                config.safety.position_jump_threshold,
                                "Position jump threshold (m)");

    // 速度限制
    readParamWithLog(parameters, "max_velocity_xy", config.safety.max_velocity_xy,
                                "Max velocity XY (m/s)");
    readParamWithLog(parameters, "max_velocity_z", config.safety.max_velocity_z,
                                "Max velocity Z (m/s)");

    // 加速度饱和检测
    readParamWithLog(parameters, "acc_saturation_xy", config.safety.acc_saturation_xy,
                                "Acc saturation XY (m/s²)");
    readParamWithLog(parameters, "acc_saturation_z", config.safety.acc_saturation_z,
                                "Acc saturation Z (m/s²)");
    readParamWithLog(parameters, "state_estimate_unusable_trip_delay",
                                config.safety.state_estimate_unusable_trip_delay,
                                "State estimate unusable trip delay (s)");
    if (!std::isfinite(config.safety.state_estimate_unusable_trip_delay) ||
        config.safety.state_estimate_unusable_trip_delay < 0.0) {
        parameters.logf(true, "[DroneRosNode] Invalid state_estimate_unusable_trip_delay; using 0.150 s");
        config.safety.state_estimate_unusable_trip_delay = 0.15;
    }


    return config;
}
}
