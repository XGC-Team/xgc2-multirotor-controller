// Deterministic replay of the reference trajectory runtime, for refactoring it safely.
//
// Drives ReferenceTrajectoryRuntime from a request stream (make_reference_stream.py) with explicit
// time, exactly as ReferenceTrajectoryNode does at run time (reference_node_path.h): each message
// goes through the input producer's accept + post, then the 100 Hz main loop updates the runtime.
// Analytic and sampled references run on the update thread, so a plan result arrives at the next
// update. Every output event and the message the output consumer would publish for it are written
// bit-exact (doubles as hex bits); when the active trajectory changes, its evaluator is sampled.
// Two builds of the runtime can then be compared with `cmp`.
//
// Time is an integer nanosecond grid: tick k is t0 + k * 10 ms, t0 the first request's time rounded
// down to 10 ms, and a request is delivered at the first tick not before its receipt time.
//
// Usage: reference_replay_harness STREAM OUT.txt
//
// The harness never touches the ROS network: messages are deserialized from bytes.

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "reference_node_path.h"
#include "reference_trace.h"

namespace mrt = multirotor_reference_trajectory;
namespace trajectory = xgc2_math::trajectory;
using reference_replay::Published;
using reference_replay::Record;

namespace {

constexpr uint64_t kTickNs = 10000000;  // main_frequency 100 Hz

FILE* out = nullptr;

void d(double v) {
    std::fprintf(out, " %016" PRIx64, reference_replay::trace::bits(v));
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
    const double span =
        std::isfinite(evaluator->duration()) ? std::min(evaluator->duration(), 20.0) : 20.0;
    for (int i = 0; static_cast<double>(i) * 0.05 <= span + 1e-9; ++i) {
        const double at = static_cast<double>(i) * 0.05;
        trajectory::FlatOutput3 f;
        const bool ok = evaluator->evaluate(at, f);
        std::fprintf(out, "  %d %d", i, ok ? 1 : 0);
        for (const auto* v : {&f.position, &f.velocity, &f.acceleration, &f.jerk, &f.snap}) {
            d(v->x());
            d(v->y());
            d(v->z());
        }
        d(f.yaw);
        d(f.yaw_rate);
        d(f.yaw_accel);
        std::fprintf(out, " fl %u\n", f.flags);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: reference_replay_harness STREAM OUT.txt\n");
        return 2;
    }
    const auto records = reference_replay::readStream(argv[1]);
    if (records.empty())
        throw std::runtime_error("empty stream");
    out = std::fopen(argv[2], "w");
    if (!out)
        throw std::runtime_error("cannot open output");

    mrt::ReferenceTrajectoryConfig config;  // config/multirotor_reference_trajectory.yaml
    config.status_rate_hz = 10.0;
    config.active_publish_rate_hz = 10.0;
    config.validation_sample_dt = 0.02;
    config.trajectory_timeout = 0.5;
    config.min_lead_time = 0.2;
    config.limits.min_specific_thrust = 0.1;
    reference_replay::NodePath node(config);
    const mrt::ReferenceTrajectoryRuntime& runtime = node.runtime();

    const uint64_t t0_ns = records.front().t_ns / kTickNs * kTickNs;
    const uint64_t t_end_ns = records.back().t_ns + 1000000000ULL;
    size_t next = 0;
    uint8_t last_state = 0xFF;
    uint32_t last_flags = 0xFFFFFFFFu;
    std::tuple<int, uint32_t, uint32_t, uint64_t> last_active{-1, 0, 0, 0};
    uint64_t events_written = 0;
    for (uint64_t k = 0;; ++k) {
        const uint64_t tick_ns = t0_ns + k * kTickNs;
        if (tick_ns > t_end_ns)
            break;
        // ros::spinOnce(): the callbacks for everything received so far.
        while (next < records.size() && records[next].t_ns <= tick_ns) {
            const Record& r = records[next++];
            std::fprintf(out, "%" PRIu64 " in %u\n", k, r.kind);
            std::fputs(node.receive(r, mrt::Time().fromNSec(r.t_ns).toSec()).c_str(), out);
        }
        const double now = mrt::Time().fromNSec(tick_ns).toSec();
        for (const Published& p : node.update(now)) {
            const auto& e = p.event;
            std::fprintf(out, "%" PRIu64 " ev %u ts %016" PRIx64 " seq %" PRIu64 " cat %d src %s",
                         k, static_cast<unsigned>(e.id), reference_replay::trace::bits(e.timestamp),
                         e.sequence, static_cast<int>(e.category), e.source.c_str());
            if (e.id == mrt::output_event_type::PUBLISH_STATUS) {
                std::fputs(reference_replay::trace::text(p.status).c_str(), out);
            } else if (e.id == mrt::output_event_type::PUBLISH_ACTIVE_ANALYTIC) {
                std::fputs(reference_replay::trace::text(p.analytic).c_str(), out);
            } else if (e.id == mrt::output_event_type::PUBLISH_ACTIVE_SAMPLED) {
                std::fputs(reference_replay::trace::text(p.sampled).c_str(), out);
            }
            std::fputc('\n', out);
            ++events_written;
        }
        if (runtime.currentState() != last_state) {
            std::fprintf(out, "%" PRIu64 " state %u\n", k, runtime.currentState());
            std::fprintf(stderr, "t=%.2f state %u\n", static_cast<double>(tick_ns - t0_ns) * 1e-9,
                         runtime.currentState());
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
        }
        const std::tuple<int, uint32_t, uint32_t, uint64_t> active{
            static_cast<int>(runtime.activeType()), runtime.activeTrajectoryId(),
            runtime.activeRevision(), start.toNSec()};
        if (active != last_active) {
            writeEvaluator(k, runtime);
            last_active = active;
        }
    }
    std::fclose(out);
    std::fprintf(stderr, "records %zu, output events %" PRIu64 "\n", records.size(),
                 events_written);
    return 0;
}
