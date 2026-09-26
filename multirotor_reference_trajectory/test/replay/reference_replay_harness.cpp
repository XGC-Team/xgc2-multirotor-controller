// Deterministic replay of the reference trajectory runtime, for refactoring
// it safely.
//
// Drives ReferenceTrajectoryRuntime from a request stream
// (test/replay/make_reference_stream.py) with explicit time, exactly as
// ReferenceTrajectoryNode does at run time: each message goes through the
// input producer's accept + post, then the 100 Hz main loop updates the
// runtime. Waypoint plans are solved inline (config inline_planning), so a
// plan result arrives at the next update. Every output event and the
// message the output consumer would publish for it are written bit-exact
// (doubles as hex bits); when the active trajectory changes, its evaluator
// is sampled. Two builds of the runtime can then be compared with `cmp`.
//
// Usage: reference_replay_harness STREAM OUT.txt
//
// The harness never touches the ROS network: rostime runs in simulated time
// (ros::Time::init + setNow) and messages are deserialized from bytes.

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <multirotor_reference_trajectory_msgs/WaypointReferenceRequest.h>
#include <ros/serialization.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>

#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"
#include "multirotor_reference_trajectory/ros_reference_conversion.h"

namespace mrt = multirotor_reference_trajectory;
namespace msgs = multirotor_reference_trajectory_msgs;
namespace sm = state_machine;
namespace trajectory = xgc2_math::trajectory;

namespace {

struct Record {
    uint64_t t_ns;
    uint8_t kind;
    std::vector<uint8_t> data;
};

std::vector<Record> readStream(const char* path) {
    std::ifstream in(path, std::ios::binary);
    char magic[8];
    if (!in.read(magic, 8) || std::memcmp(magic, "MRTRPLY1", 8) != 0) {
        throw std::runtime_error("not a reference replay stream");
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
        if (len > 0 && !in.read(reinterpret_cast<char*>(r.data.data()), len)) {
            throw std::runtime_error("truncated stream");
        }
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

uint64_t bits(double v) {
    uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

FILE* out = nullptr;

void d(double v) { std::fprintf(out, " %016" PRIx64, bits(v)); }
void ds(const std::vector<double>& v) {
    std::fprintf(out, " [%zu", v.size());
    for (double x : v) d(x);
    std::fputc(']', out);
}
void t(const ros::Time& v) { std::fprintf(out, " %u.%09u", v.sec, v.nsec); }
void vec(const geometry_msgs::Vector3& v) { d(v.x); d(v.y); d(v.z); }
void point(const geometry_msgs::Point& v) { d(v.x); d(v.y); d(v.z); }
void header(const std_msgs::Header& h) {
    std::fprintf(out, " hdr %u", h.seq);
    t(h.stamp);
    std::fprintf(out, " %s", h.frame_id.empty() ? "-" : h.frame_id.c_str());
}

void writeStatus(const msgs::ReferenceStatus& m) {
    std::fprintf(out, " status");
    header(m.header);
    std::fprintf(out, " st %u fl %u id %u rev %u type %u", m.state, m.flags, m.active_trajectory_id,
                 m.active_revision, m.active_type);
}

void writeAnalytic(const msgs::AnalyticReference& m) {
    std::fprintf(out, " analytic");
    header(m.header);
    std::fprintf(out, " req %u id %u rev %u type %u fl %u", m.request_id, m.trajectory_id, m.revision,
                 m.analytic_type, m.flags);
    t(m.start_time);
    d(m.duration);
    point(m.origin.position);
    d(m.origin.orientation.x); d(m.origin.orientation.y); d(m.origin.orientation.z); d(m.origin.orientation.w);
    ds(m.params);
}

void writePolynomial(const msgs::ActivePolynomialReference& m) {
    std::fprintf(out, " polynomial");
    header(m.header);
    std::fprintf(out, " id %u rev %u fl %u", m.trajectory_id, m.revision, m.flags);
    t(m.start_time);
    d(m.duration);
    std::fprintf(out, " order %u", m.order);
    ds(m.segment_durations);
    ds(m.coeff_x);
    ds(m.coeff_y);
    ds(m.coeff_z);
    ds(m.coeff_yaw);
}

void writeSampled(const msgs::SampledReference& m) {
    std::fprintf(out, " sampled");
    header(m.header);
    std::fprintf(out, " id %u rev %u fl %u", m.trajectory_id, m.revision, m.flags);
    t(m.start_time);
    d(m.sample_dt);
    std::fprintf(out, " [%zu", m.points.size());
    for (const auto& p : m.points) {
        d(p.t_from_start);
        point(p.position);
        vec(p.velocity); vec(p.acceleration); vec(p.jerk); vec(p.snap);
        d(p.yaw); d(p.yaw_rate); d(p.yaw_accel);
    }
    std::fputc(']', out);
}

// The active evaluator, sampled every 0.05 s over (at most) its first 20 s,
// as the reference path preview would read it.
void writeEvaluator(uint64_t k, const mrt::ReferenceTrajectoryRuntime& runtime) {
    const auto* evaluator = runtime.evaluator();
    std::fprintf(out, "%" PRIu64 " evaluator", k);
    if (evaluator == nullptr) {
        std::fprintf(out, " none\n");
        return;
    }
    std::fprintf(out, " type %d", static_cast<int>(evaluator->type()));
    d(evaluator->duration());
    std::fprintf(out, " fl %u\n", evaluator->flags());
    const double span = std::isfinite(evaluator->duration()) ? std::min(evaluator->duration(), 20.0) : 20.0;
    for (int i = 0; static_cast<double>(i) * 0.05 <= span + 1e-9; ++i) {
        const double at = static_cast<double>(i) * 0.05;
        trajectory::FlatOutput3 f;
        const bool ok = evaluator->evaluate(at, f);
        std::fprintf(out, "  %d %d", i, ok ? 1 : 0);
        for (const auto* v : {&f.position, &f.velocity, &f.acceleration, &f.jerk, &f.snap}) {
            d(v->x()); d(v->y()); d(v->z());
        }
        d(f.yaw); d(f.yaw_rate); d(f.yaw_accel);
        std::fprintf(out, " fl %u\n", f.flags);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: reference_replay_harness STREAM OUT.txt\n");
        return 2;
    }
    const auto records = readStream(argv[1]);
    if (records.empty()) throw std::runtime_error("empty stream");
    out = std::fopen(argv[2], "w");
    if (!out) throw std::runtime_error("cannot open output");

    ros::Time::init();  // simulated time: this harness sets now explicitly
    mrt::ReferenceTrajectoryRuntime runtime;
    mrt::ReferenceTrajectoryConfig config;  // config/multirotor_reference_trajectory.yaml
    config.status_rate_hz = 10.0;
    config.active_publish_rate_hz = 10.0;
    config.validation_sample_dt = 0.02;
    config.trajectory_timeout = 0.5;
    config.min_lead_time = 0.2;
    config.limits.min_specific_thrust = 0.1;
    config.inline_planning = true;
    runtime.setConfig(config);

    // ReferenceInputProducer::post
    auto post = [&](uint32_t id, const char* source) {
        sm::Event event(id, sm::EventTimestamp{ros::Time::now().toSec()});
        event.source = source;
        const auto status = runtime.postEvent(std::move(event));
        std::fprintf(out, "  post %u %s %s\n", id, source, status.ok() ? "ok" : status.message.c_str());
    };

    const double t0 = std::floor(records.front().t_ns * 1e-9 * 100.0) / 100.0;
    const double t_end = records.back().t_ns * 1e-9 + 1.0;
    size_t next = 0;
    uint8_t last_state = 0xFF;
    uint32_t last_flags = 0xFFFFFFFFu;
    std::tuple<int, uint32_t, uint32_t, uint64_t> last_active{-1, 0, 0, 0};
    uint64_t events_written = 0;
    for (uint64_t k = 0;; ++k) {
        const double now_target = t0 + static_cast<double>(k) * 0.01;  // main_frequency 100 Hz
        if (now_target > t_end) break;
        // ros::spinOnce(): the callbacks for everything received so far.
        while (next < records.size() && records[next].t_ns * 1e-9 <= now_target) {
            const Record& r = records[next++];
            ros::Time::setNow(ros::Time().fromNSec(r.t_ns));
            std::fprintf(out, "%" PRIu64 " in %u\n", k, r.kind);
            switch (r.kind) {
                case 1:
                    if (runtime.acceptAnalytic(mrt::toCore(decode<msgs::AnalyticReference>(r.data)))) {
                        post(mrt::event_type::ANALYTIC_RECEIVED, "analytic_reference");
                    } else {
                        std::fprintf(out, "  rejected\n");
                    }
                    break;
                case 2:
                    if (runtime.acceptWaypoint(mrt::toCore(decode<msgs::WaypointReferenceRequest>(r.data)))) {
                        post(mrt::event_type::WAYPOINT_RECEIVED, "waypoint_reference");
                    } else {
                        std::fprintf(out, "  rejected\n");
                    }
                    break;
                case 3:
                    if (runtime.acceptSampled(mrt::toCore(decode<msgs::SampledReference>(r.data)))) {
                        post(mrt::event_type::SAMPLED_RECEIVED, "sampled_reference");
                    } else {
                        std::fprintf(out, "  rejected\n");
                    }
                    break;
                case 4:
                    runtime.reset();
                    post(mrt::event_type::RESET_REQUESTED, "reset");
                    break;
                default:
                    throw std::runtime_error("unknown record kind");
            }
        }
        ros::Time::setNow(ros::Time(now_target));
        runtime.update(ros::Time::now().toSec());
        for (const auto& e : runtime.stateMachine().currentOutputEvents()) {
            std::fprintf(out, "%" PRIu64 " ev %u ts %016" PRIx64 " seq %" PRIu64 " cat %d src %s", k,
                         static_cast<unsigned>(e.id), bits(e.timestamp), e.sequence,
                         static_cast<int>(e.category), e.source.c_str());
            // ReferenceOutputConsumer::handle
            if (e.id == mrt::output_event_type::PUBLISH_STATUS) {
                writeStatus(mrt::toRos(runtime.makeStatus(e.timestamp > 0.0 ? e.timestamp : ros::Time::now().toSec())));
            } else if (e.id == mrt::output_event_type::PUBLISH_ACTIVE_ANALYTIC) {
                writeAnalytic(mrt::toRos(runtime.activeAnalyticMessage()));
            } else if (e.id == mrt::output_event_type::PUBLISH_ACTIVE_POLYNOMIAL) {
                writePolynomial(mrt::toRos(runtime.activePolynomialMessage()));
            } else if (e.id == mrt::output_event_type::PUBLISH_ACTIVE_SAMPLED) {
                writeSampled(mrt::toRos(runtime.activeSampledMessage()));
            }
            std::fputc('\n', out);
            ++events_written;
        }
        if (runtime.currentState() != last_state) {
            std::fprintf(out, "%" PRIu64 " state %u\n", k, runtime.currentState());
            std::fprintf(stderr, "t=%.2f state %u\n", now_target - t0, runtime.currentState());
            last_state = runtime.currentState();
        }
        if (runtime.flags() != last_flags) {
            std::fprintf(out, "%" PRIu64 " flags %u\n", k, runtime.flags());
            last_flags = runtime.flags();
        }
        mrt::Time start;
        if (runtime.activeType() == trajectory::TrajectoryModelType::kAnalytic) {
            start = runtime.activeAnalyticMessage().start_time;
        } else if (runtime.activeType() == trajectory::TrajectoryModelType::kSampled) {
            start = runtime.activeSampledMessage().start_time;
        } else if (runtime.activeType() == trajectory::TrajectoryModelType::kPolynomial) {
            start = runtime.activePolynomialMessage().start_time;
        }
        const std::tuple<int, uint32_t, uint32_t, uint64_t> active{
            static_cast<int>(runtime.activeType()), runtime.activeTrajectoryId(), runtime.activeRevision(),
            start.toNSec()};
        if (active != last_active) {
            writeEvaluator(k, runtime);
            last_active = active;
        }
    }
    std::fclose(out);
    std::fprintf(stderr, "records %zu, output events %" PRIu64 "\n", records.size(), events_written);
    return 0;
}
