// Deterministic replay of the controller core, for refactoring it safely.
//
// Drives DroneController + its state machine from a recorded input stream (bag_to_stream.py) with
// explicit time, exactly as DroneRosNode does at run time (controller_node_path.h): each message
// fills SensorData the way the input producers do and posts the same input event; topic stats
// follow ros1_utils::TopicStatsManager (is_active/is_new on arrival, a 0.1 s timer with the 2.5 s
// timeout); the control loop ticks at 1 kHz (main.cpp). Every output event and the setpoint
// snapshots the ROS consumers publish are written bit-exact (doubles as hex bits), so two builds of
// the core can be compared with `cmp`.
//
// Time is an integer nanosecond grid: tick k is t0 + k * 1 ms, t0 the first record's time rounded
// down to 1 ms, and a record is delivered at the first tick not before its receipt time.
//
// Usage: replay_harness STREAM OUT.txt [px4_local|dfbc|nmpc|smc [reference_analytic_type=N]]
//
// The configuration is config/uav_nmpc.yaml's, with the tracking backend (default px4_local) and,
// optionally, nmpc/reference_analytic_type from the command line. With nmpc, each
// REQUEST_NMPC_SOLVE is solved inline, exactly as NmpcOutputConsumer's worker solves it; the result
// arrives at the next update, as if the worker had finished at once. The reference activation
// request (PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION) is written too.
//
// The harness never touches the ROS network: messages are deserialized from bytes.

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "controller_node_path.h"

namespace pmc = px4_multirotor_controller;
using controller_replay::OutputEvent;
using controller_replay::Record;

namespace {

constexpr uint64_t kTickNs = 1000000;  // the node's hard-coded 1 kHz loop

uint64_t bits(double v) {
    uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

void writeDoubles(FILE* out, const char* tag, std::initializer_list<double> values) {
    std::fprintf(out, " %s", tag);
    for (double v : values)
        std::fprintf(out, " %016" PRIx64, bits(v));
}

void writeEvent(FILE* out, uint64_t k, const OutputEvent& o) {
    const auto& e = o.event;
    std::fprintf(out, "%" PRIu64 " ev %u ts %016" PRIx64 " seq %" PRIu64 " cat %d src %s", k,
                 static_cast<unsigned>(e.id), bits(e.timestamp), e.sequence,
                 static_cast<int>(e.category), e.source.c_str());
    for (const auto& [key, value] : e.payload) {
        std::fprintf(out, " %s=", key.c_str());
        std::visit(
            [&](const auto& v) {
                using V = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<V, double>)
                    std::fprintf(out, "d:%016" PRIx64, bits(v));
                else if constexpr (std::is_same_v<V, int64_t>)
                    std::fprintf(out, "i:%" PRId64, v);
                else if constexpr (std::is_same_v<V, bool>)
                    std::fprintf(out, "b:%d", v ? 1 : 0);
                else
                    std::fprintf(out, "s:%s", v.c_str());
            },
            value);
    }
    if (e.id == pmc::output_event_type::PUBLISH_SETPOINT) {
        const auto& s = o.setpoint;
        writeDoubles(out, "sp",
                     {s.x, s.y, s.z, s.vx, s.vy, s.vz, s.ax, s.ay, s.az, s.qx, s.qy, s.qz, s.qw,
                      s.yaw_rate});
        std::fprintf(out, " mask %u", static_cast<unsigned>(s.type_mask));
    }
    if (e.id == pmc::output_event_type::PUBLISH_ATTITUDE_RATE_TARGET) {
        const auto& a = o.attitude;
        writeDoubles(out, "art", {a.body_rate_x, a.body_rate_y, a.body_rate_z, a.thrust});
    }
    if (e.id == pmc::output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION) {
        const auto& m = o.activation;
        std::fprintf(out, " activation %u.%09u req %u id %u rev %u type %u", m.header.stamp.sec,
                     m.header.stamp.nsec, m.request_id, m.trajectory_id, m.revision,
                     m.analytic_type);
        std::fprintf(out, " start %u.%09u", m.start_time.sec, m.start_time.nsec);
        writeDoubles(out, "o",
                     {m.duration, m.origin.position.x, m.origin.position.y, m.origin.position.z,
                      m.origin.orientation.x, m.origin.orientation.y, m.origin.orientation.z,
                      m.origin.orientation.w});
        std::fprintf(out, " params");
        for (double v : m.params)
            writeDoubles(out, "", {v});
    }
    if (o.solved) {
        std::fprintf(out, " nmpc %d status %d", o.solve.success ? 1 : 0, o.solve.solver_status);
        writeDoubles(out, "t",
                     {o.solve.target.body_rate_x, o.solve.target.body_rate_y,
                      o.solve.target.body_rate_z, o.solve.target.thrust});
    }
    std::fputc('\n', out);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 5) {
        std::fprintf(stderr,
                     "usage: replay_harness STREAM OUT.txt [px4_local|dfbc|nmpc|smc "
                     "[reference_analytic_type=N]]\n");
        return 2;
    }
    const auto records = controller_replay::readStream(argv[1]);
    if (records.empty())
        throw std::runtime_error("empty stream");
    FILE* out = std::fopen(argv[2], "w");
    if (!out)
        throw std::runtime_error("cannot open output");

    const std::string backend = argc >= 4 ? argv[3] : "px4_local";
    pmc::TrackingBackend tracking;
    if (backend == "px4_local")
        tracking = pmc::TrackingBackend::PX4_LOCAL;
    else if (backend == "dfbc")
        tracking = pmc::TrackingBackend::DFBC;
    else if (backend == "nmpc")
        tracking = pmc::TrackingBackend::NMPC;
    else if (backend == "smc")
        tracking = pmc::TrackingBackend::SMC;
    else
        throw std::runtime_error("unknown tracking backend");
    pmc::ControllerConfig config = controller_replay::NodePath::profileConfig(tracking);
    if (argc == 5) {
        const std::string arg = argv[4];
        const std::string key = "reference_analytic_type=";
        if (arg.rfind(key, 0) != 0)
            throw std::runtime_error("unknown option " + arg);
        config.nmpc.reference_analytic_type = std::stoi(arg.substr(key.size()));
    }

    const uint64_t t0_ns = records.front().t_ns / kTickNs * kTickNs;
    const uint64_t t_end_ns = records.back().t_ns + 500000000ULL;
    controller_replay::NodePath node(config, pmc::Time().fromNSec(t0_ns).toSec());

    size_t next = 0;
    std::string last_state;
    uint64_t events_written = 0;
    for (uint64_t k = 0;; ++k) {
        const uint64_t tick_ns = t0_ns + k * kTickNs;
        if (tick_ns > t_end_ns)
            break;
        // ros::spinOnce(): the callbacks for everything received so far.
        while (next < records.size() && records[next].t_ns <= tick_ns)
            node.receive(records[next++]);
        for (const OutputEvent& o : node.update(pmc::Time().fromNSec(tick_ns).toSec())) {
            writeEvent(out, k, o);
            ++events_written;
        }
        const std::string state = node.state();
        if (state != last_state) {
            std::fprintf(out, "%" PRIu64 " state %s\n", k, state.c_str());
            std::fprintf(stderr, "t=%.3f %s\n", static_cast<double>(tick_ns - t0_ns) * 1e-9,
                         state.c_str());
            last_state = state;
        }
    }
    std::fclose(out);
    std::fprintf(stderr, "records %zu, output events %" PRIu64 "\n", records.size(),
                 events_written);
    return 0;
}
