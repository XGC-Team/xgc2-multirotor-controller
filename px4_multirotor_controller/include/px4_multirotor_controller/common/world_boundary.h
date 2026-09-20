#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace px4_multirotor_controller {

// Consumer of L's existing worldBoundary schema v1. Coordinates have already
// been expressed in the Experiment world; this controller adds no offset.
struct WorldControlBounds {
    double x_min, x_max, y_min, y_max, z_min, z_max;

    bool outside(double x, double y, double z) const {
        return x < x_min || x > x_max || y < y_min || y > y_max || z < z_min || z > z_max;
    }
};

struct WorldBoundary {
    std::optional<WorldControlBounds> control_bounds;
    // Retained contract metadata, never substituted for a control endpoint.
    std::optional<double> ground_z;
};

// Explicit JSON null and controlBounds:null mean unset. Missing, partial,
// duplicate, non-finite and unsupported contracts throw before setConfig.
std::optional<WorldBoundary> parseWorldBoundary(const std::string& raw);

struct ControllerLoadFact {
    std::string node_name;
    std::string instance_id;
    std::string canonical_pose_topic;
    std::string local_pose_topic;
    std::string world_pose_topic;
    std::string tracking_backend;
    double position_distance_limit_metres;
    std::uint64_t ros_time_ns;
    std::uint64_t unix_time_ns;
    std::uint64_t monotonic_time_ns;
};

// K producer wire v1. The caller supplies clocks captured after setConfig
// returned and the exact typed configuration read back from that controller.
std::string controllerLoadFactJSON(const std::optional<WorldBoundary>& boundary,
                                   const ControllerLoadFact& fact);

}  // namespace px4_multirotor_controller
