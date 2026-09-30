#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "multirotor_reference_trajectory/time.h"

namespace multirotor_reference_trajectory {
namespace reference {

// The runtime's inputs and outputs, without ROS. They mirror the
// multirotor_reference_trajectory_msgs messages field for field (the ROS
// edge converts in ros_reference_conversion.h and pins the constants), so
// the runtime reads and writes exactly the values the messages carried.
// Defaults are zero, like ROS message defaults.

struct Point {
    double x{0.0}, y{0.0}, z{0.0};
};
struct Vector3 {
    double x{0.0}, y{0.0}, z{0.0};
};
struct Quaternion {
    double x{0.0}, y{0.0}, z{0.0}, w{0.0};
};
struct Pose {
    Point position;
    Quaternion orientation;
};
struct Header {
    uint32_t seq{0};
    Time stamp;
    std::string frame_id;
};

struct AnalyticReference {
    static constexpr uint16_t ANALYTIC_HOLD = 0;
    static constexpr uint16_t ANALYTIC_CIRCLE = 1;
    static constexpr uint16_t ANALYTIC_HEIGHT_CIRCLE = 2;
    static constexpr uint16_t ANALYTIC_CIRCLE_ENTRY = 3;
    static constexpr uint16_t ANALYTIC_FIGURE_EIGHT = 4;
    static constexpr uint16_t ANALYTIC_LINE = 5;
    static constexpr uint16_t ANALYTIC_LEMNISCATE = 6;
    static constexpr uint16_t ANALYTIC_HELIX_YZ = 7;
    static constexpr uint16_t ANALYTIC_HELIX_XY = 8;
    static constexpr uint16_t ANALYTIC_TORUS_KNOT = 9;

    Header header;
    uint32_t request_id{0};
    uint32_t trajectory_id{0};
    uint32_t revision{0};
    uint16_t analytic_type{0};
    uint32_t flags{0};
    Time start_time;
    double duration{0.0};
    Pose origin;
    std::vector<double> params;
};

struct FlatReferencePoint {
    double t_from_start{0.0};
    Point position;
    Vector3 velocity, acceleration, jerk, snap;
    double yaw{0.0}, yaw_rate{0.0}, yaw_accel{0.0};
};

struct SampledReference {
    Header header;
    uint32_t trajectory_id{0};
    uint32_t revision{0};
    uint32_t flags{0};
    Time start_time;
    double sample_dt{0.0};
    std::vector<FlatReferencePoint> points;
};

struct ReferenceStatus {
    static constexpr uint8_t STATE_SELF_CHECK = 1;
    static constexpr uint8_t STATE_READY = 2;
    static constexpr uint8_t STATE_ACTIVE = 4;
    static constexpr uint8_t STATE_FAULT = 9;
    static constexpr uint8_t TYPE_NONE = 0;
    static constexpr uint8_t TYPE_ANALYTIC = 1;
    static constexpr uint8_t TYPE_SAMPLED = 3;

    Header header;
    uint8_t state{0};
    uint32_t flags{0};
    uint32_t active_trajectory_id{0};
    uint32_t active_revision{0};
    uint8_t active_type{0};
};

}  // namespace reference
}  // namespace multirotor_reference_trajectory
