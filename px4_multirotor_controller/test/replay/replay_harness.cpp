// Deterministic replay of the controller core, for refactoring it safely.
//
// Drives DroneController + its state machine from a recorded input stream
// (test/replay/bag_to_stream.py) with explicit time, exactly as
// DroneRosNode does at run time: each message fills SensorData the way the
// input producers do and posts the same input event; topic stats follow
// ros1_utils::TopicStatsManager (is_active/is_new on arrival, a 0.1 s timer
// with the 2.5 s timeout); the control loop ticks at 1 kHz (main.cpp).
// Every output event and the setpoint snapshots the ROS consumers publish
// are written bit-exact (doubles as hex bits), so two builds of the core can
// be compared with `cmp`.
//
// Usage: replay_harness STREAM OUT.txt
//
// The harness never touches the ROS network: rostime runs in simulated time
// (ros::Time::init + setNow) and messages are deserialized from bytes.

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/serialization.h>
#include <ros/time.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include "px4_multirotor_controller/common/time.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace pmc = px4_multirotor_controller;
namespace sm = state_machine;

namespace {

struct Record {
    uint64_t t_ns;
    uint8_t kind;
    std::vector<uint8_t> data;
};

std::vector<Record> readStream(const char* path) {
    std::ifstream in(path, std::ios::binary);
    char magic[8];
    if (!in.read(magic, 8) || std::memcmp(magic, "PMCRPLY1", 8) != 0) {
        throw std::runtime_error("not a replay stream");
    }
    std::vector<Record> records;
    for (;;) {
        uint64_t t = 0;
        uint8_t kind = 0;
        uint32_t len = 0;
        if (!in.read(reinterpret_cast<char*>(&t), 8)) break;
        in.read(reinterpret_cast<char*>(&kind), 1);
        in.read(reinterpret_cast<char*>(&len), 4);
        Record r{t, kind, std::vector<uint8_t>(len)};
        if (!in.read(reinterpret_cast<char*>(r.data.data()), len)) throw std::runtime_error("truncated stream");
        records.push_back(std::move(r));
    }
    return records;
}

template <typename M>
M decode(const std::vector<uint8_t>& d) {
    M m;
    ros::serialization::IStream s(const_cast<uint8_t*>(d.data()), static_cast<uint32_t>(d.size()));
    ros::serialization::deserialize(s, m);
    return m;
}

// ros1_utils::TopicStatsManager, reduced to what the core reads.
struct Track {
    pmc::SensorData::TopicStats* stats{nullptr};
    std::deque<double> times;
};

const std::map<std::string, sm::EventId> kCommands = {
    {"takeoff", pmc::event_type::TAKEOFF_REQUESTED}, {"Takeoff", pmc::event_type::TAKEOFF_REQUESTED},
    {"TAKEOFF", pmc::event_type::TAKEOFF_REQUESTED}, {"land", pmc::event_type::LANDING_REQUESTED},
    {"Land", pmc::event_type::LANDING_REQUESTED},       {"LAND", pmc::event_type::LANDING_REQUESTED},
    {"hover", pmc::event_type::HOVER_REQUESTED},         {"Hover", pmc::event_type::HOVER_REQUESTED},
    {"HOVER", pmc::event_type::HOVER_REQUESTED},         {"custom1", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"Custom1", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED}, {"CUSTOM1", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"start", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},   {"Start", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"START", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},   {"track", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"Track", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},   {"TRACK", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
};

uint64_t bits(double v) {
    uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

void writeDoubles(FILE* out, const char* tag, std::initializer_list<double> values) {
    std::fprintf(out, " %s", tag);
    for (double v : values) std::fprintf(out, " %016" PRIx64, bits(v));
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: replay_harness STREAM OUT.txt\n");
        return 2;
    }
    const auto records = readStream(argv[1]);
    if (records.empty()) throw std::runtime_error("empty stream");
    FILE* out = std::fopen(argv[2], "w");
    if (!out) throw std::runtime_error("cannot open output");

    ros::Time::init();  // simulated time: this harness sets now explicitly
    pmc::SensorData sensor;
    pmc::DroneController controller(sensor);
    pmc::ControllerConfig config;  // values of config/uav_nmpc.yaml that drive the px4_local path
    config.takeoff_altitude = 2.3;
    config.tracking_backend = pmc::TrackingBackend::PX4_LOCAL;
    config.local_type_mask = 3072;
    controller.setConfig(config);

    std::map<uint8_t, Track> tracks = {
        {1, {&sensor.uav_state_estimate_stats, {}}}, {2, {&sensor.local_pos_stats, {}}},
        {3, {&sensor.local_velocity_stats, {}}},     {4, {&sensor.imu_stats, {}}},
        {5, {&sensor.state_stats, {}}},              {6, {&sensor.battery_stats, {}}},
        {7, {&sensor.vrpn_pose_stats, {}}},
    };
    auto post = [&](sm::EventId id, double now, const char* source) {
        sm::Event event(id, sm::EventTimestamp{now});
        event.source = source;
        controller.getStateMachine().postEvent(std::move(event));
    };

    const double t0 = std::floor(records.front().t_ns * 1e-9 * 1000.0) / 1000.0;
    const double t_end = records.back().t_ns * 1e-9;
    double next_stats = t0 + 0.1;
    auto runStatsTimer = [&](double until) {
        for (; next_stats <= until; next_stats += 0.1) {
            for (auto& [kind, track] : tracks) {
                if (track.times.size() >= 2) track.stats->is_active = (next_stats - track.times.back()) <= 2.5;
            }
        }
    };

    size_t next = 0;
    std::string last_state;
    uint64_t events_written = 0;
    for (uint64_t k = 0;; ++k) {
        const double t = t0 + static_cast<double>(k) * 0.001;
        if (t > t_end + 0.5) break;
        while (next < records.size() && records[next].t_ns * 1e-9 <= t) {
            const Record& r = records[next++];
            const double now = r.t_ns * 1e-9;
            runStatsTimer(now);
            ros::Time::setNow(ros::Time().fromNSec(r.t_ns));
            if (auto it = tracks.find(r.kind); it != tracks.end()) {
                Track& tr = it->second;
                tr.times.push_back(now);
                if (tr.times.size() > 10) tr.times.pop_front();
                tr.stats->last_message_time = pmc::Time().fromNSec(r.t_ns);
                tr.stats->is_active = true;
                tr.stats->is_new = true;
            }
            switch (r.kind) {
                case 1: {
                    const auto m = decode<rigid_state_estimator_msgs::RigidStateEstimate>(r.data);
                    sensor.x = m.position.x; sensor.y = m.position.y; sensor.z = m.position.z;
                    sensor.vx = m.velocity.x; sensor.vy = m.velocity.y; sensor.vz = m.velocity.z;
                    sensor.qx = m.orientation.x; sensor.qy = m.orientation.y; sensor.qz = m.orientation.z; sensor.qw = m.orientation.w;
                    sensor.wx = m.angular_velocity.x; sensor.wy = m.angular_velocity.y; sensor.wz = m.angular_velocity.z;
                    sensor.ax = m.linear_acceleration.x; sensor.ay = m.linear_acceleration.y; sensor.az = m.linear_acceleration.z;
                    sensor.gx = m.gravity.x; sensor.gy = m.gravity.y; sensor.gz = m.gravity.z;
                    sensor.accel_bias_x = m.accel_bias.x; sensor.accel_bias_y = m.accel_bias.y; sensor.accel_bias_z = m.accel_bias.z;
                    sensor.uav_state_estimator_state = m.estimator_state;
                    sensor.uav_state_estimator_flags = m.flags;
                    sensor.uav_state_estimate_stamp = m.header.stamp.toSec();
                    sensor.uav_state_filter_inertial_stamp = m.filter_inertial_stamp_sec;
                    sensor.uav_state_filter_pose_stamp = m.filter_pose_stamp_sec;
                    sensor.uav_state_last_vrpn_pose_stamp = m.last_vrpn_pose_stamp_sec;
                    post(pmc::event_type::INPUT_UAV_STATE_ESTIMATE_UPDATED, now, "alg/state_estimator/state");
                    break;
                }
                case 2: {
                    const auto m = decode<geometry_msgs::PoseStamped>(r.data);
                    sensor.local_x = m.pose.position.x; sensor.local_y = m.pose.position.y; sensor.local_z = m.pose.position.z;
                    sensor.local_qx = m.pose.orientation.x; sensor.local_qy = m.pose.orientation.y;
                    sensor.local_qz = m.pose.orientation.z; sensor.local_qw = m.pose.orientation.w;
                    post(pmc::event_type::INPUT_LOCAL_POSITION_UPDATED, now, "mavros/local_position/pose");
                    break;
                }
                case 3: {
                    const auto m = decode<geometry_msgs::TwistStamped>(r.data);
                    sensor.local_vx = m.twist.linear.x; sensor.local_vy = m.twist.linear.y; sensor.local_vz = m.twist.linear.z;
                    post(pmc::event_type::INPUT_LOCAL_VELOCITY_UPDATED, now, "mavros/local_position/velocity_local");
                    break;
                }
                case 4:
                    post(pmc::event_type::INPUT_IMU_UPDATED, now, "mavros/imu/data");
                    break;
                case 5: {
                    const auto m = decode<mavros_msgs::State>(r.data);
                    sensor.fcu_connected = m.connected; sensor.fcu_armed = m.armed; sensor.fcu_guided = m.guided;
                    sensor.fcu_manual_input = m.manual_input; sensor.fcu_mode = m.mode; sensor.fcu_system_status = m.system_status;
                    post(pmc::event_type::INPUT_FCU_STATE_UPDATED, now, "mavros/state");
                    break;
                }
                case 6: {
                    const auto m = decode<sensor_msgs::BatteryState>(r.data);
                    sensor.battery_percentage = m.percentage;
                    post(pmc::event_type::INPUT_BATTERY_UPDATED, now, "mavros/battery");
                    break;
                }
                case 7: {
                    const auto m = decode<geometry_msgs::PoseStamped>(r.data);
                    sensor.vrpn_x = m.pose.position.x; sensor.vrpn_y = m.pose.position.y; sensor.vrpn_z = m.pose.position.z;
                    sensor.vrpn_qx = m.pose.orientation.x; sensor.vrpn_qy = m.pose.orientation.y;
                    sensor.vrpn_qz = m.pose.orientation.z; sensor.vrpn_qw = m.pose.orientation.w;
                    post(pmc::event_type::INPUT_VRPN_POSE_UPDATED, now, "pose");
                    break;
                }
                case 8: {
                    const auto m = decode<std_msgs::String>(r.data);
                    if (auto it = kCommands.find(m.data); it != kCommands.end()) post(it->second, now, "command");
                    break;
                }
                default:
                    throw std::runtime_error("unknown record kind");
            }
        }
        runStatsTimer(t);
        ros::Time::setNow(ros::Time(t));
        controller.update(t);
        for (const auto& e : controller.getStateMachine().currentOutputEvents()) {
            std::fprintf(out, "%" PRIu64 " ev %u ts %016" PRIx64 " seq %" PRIu64 " cat %d src %s", k, static_cast<unsigned>(e.id),
                         bits(e.timestamp), e.sequence, static_cast<int>(e.category), e.source.c_str());
            for (const auto& [key, value] : e.payload) {
                std::fprintf(out, " %s=", key.c_str());
                std::visit(
                    [&](const auto& v) {
                        using V = std::decay_t<decltype(v)>;
                        if constexpr (std::is_same_v<V, double>) std::fprintf(out, "d:%016" PRIx64, bits(v));
                        else if constexpr (std::is_same_v<V, int64_t>) std::fprintf(out, "i:%" PRId64, v);
                        else if constexpr (std::is_same_v<V, bool>) std::fprintf(out, "b:%d", v ? 1 : 0);
                        else std::fprintf(out, "s:%s", v.c_str());
                    },
                    value);
            }
            if (e.id == pmc::output_event_type::PUBLISH_SETPOINT) {
                const auto& s = controller.getSetpoint();
                writeDoubles(out, "sp", {s.x, s.y, s.z, s.vx, s.vy, s.vz, s.ax, s.ay, s.az, s.qx, s.qy, s.qz, s.qw, s.yaw_rate});
                std::fprintf(out, " mask %u", static_cast<unsigned>(s.type_mask));
            }
            if (e.id == pmc::output_event_type::PUBLISH_ATTITUDE_RATE_TARGET) {
                const auto& a = controller.getAttitudeRateTarget();
                writeDoubles(out, "art", {a.body_rate_x, a.body_rate_y, a.body_rate_z, a.thrust});
            }
            std::fputc('\n', out);
            ++events_written;
        }
        const std::string state = controller.getStateMachine().currentStateName(pmc::region_type::CONTROL);
        if (state != last_state) {
            std::fprintf(out, "%" PRIu64 " state %s\n", k, state.c_str());
            std::fprintf(stderr, "t=%.3f %s\n", t - t0, state.c_str());
            last_state = state;
        }
        for (auto& [kind, track] : tracks) track.stats->is_new = false;
    }
    std::fclose(out);
    std::fprintf(stderr, "records %zu, output events %" PRIu64 "\n", records.size(), events_written);
    return 0;
}
