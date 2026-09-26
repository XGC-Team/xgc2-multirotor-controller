// Golden test for ActiveTrajectoryCache: deterministic analytic (every
// type, including an unknown one), polynomial and sampled references,
// sampled and horizon-sampled at many times. Every result is printed with
// doubles as hex bits, so the output of two builds compares with `cmp`.
// The recorded flight never sends a reference, so this is the gate for
// changes to the cache's input types.
//
// Usage: reference_cache_golden OUT.txt

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include <multirotor_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>

#include "px4_multirotor_controller/uav/active_trajectory_cache.h"
#if __has_include("px4_multirotor_controller/ros_reference_conversion.h")
#include "px4_multirotor_controller/ros_reference_conversion.h"
#define PMC_REF(msg) ::px4_multirotor_controller::toCoreReference(msg)
#else
#define PMC_REF(msg) (msg)
#endif

namespace pmc = px4_multirotor_controller;
namespace mrt = multirotor_reference_trajectory_msgs;

namespace {

FILE* out = nullptr;

void d(double v) {
    uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    std::fprintf(out, " %016" PRIx64, b);
}

void v3(const Eigen::Vector3d& v) { d(v.x()); d(v.y()); d(v.z()); }

void probe(pmc::ActiveTrajectoryCache& cache, const pmc::Time& start, const char* label) {
    std::fprintf(out, "%s valid %d id %u rev %u seq %" PRIu64 "\n", label, cache.valid() ? 1 : 0, cache.trajectoryId(),
                 cache.revision(), cache.sequence());
    for (double dt : {-0.5, 0.0, 0.013, 0.37, 1.0, 2.5, 7.9, 30.0, 400.0}) {
        const pmc::Time now = start + pmc::Duration(dt);
        pmc::UavReferencePoint p;
        const bool ok = cache.sample(now, p);
        std::fprintf(out, "  sample %d flags %u", ok ? 1 : 0, p.flags);
        d(p.t_from_start); v3(p.position); v3(p.velocity); v3(p.acceleration); v3(p.jerk); v3(p.snap);
        d(p.yaw); d(p.yaw_rate); d(p.yaw_accel);
        double remaining = -1.0;
        const bool fin = cache.finiteTimeRemaining(now, remaining);
        std::fprintf(out, " finite %d", fin ? 1 : 0);
        d(remaining);
        std::vector<xgc2_math::control::Se3Reference> refs;
        const bool hz = cache.sampleHorizon(now, 0.05, 20, 9.8066, refs);
        std::fprintf(out, " horizon %d n %zu", hz ? 1 : 0, refs.size());
        for (const auto& r : refs) {
            v3(r.state.position); v3(r.state.velocity);
            d(r.state.attitude.w()); d(r.state.attitude.x()); d(r.state.attitude.y()); d(r.state.attitude.z());
            v3(r.state.body_rate);
            d(r.control.body_z_specific_force); v3(r.control.angular_acceleration);
            std::fprintf(out, " f%u", r.flags);
        }
        std::fputc('\n', out);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    out = std::fopen(argv[1], "w");
    std::mt19937_64 rng(0xe0a266e);
    std::uniform_real_distribution<double> u(0.2, 2.0);
    pmc::ActiveTrajectoryCache cache;
    const pmc::Time received(1790380000.0);
    const ros::Time ros_received(1790380000.0);

    for (int type = 0; type <= 10; ++type) {
        for (int variant = 0; variant < 3; ++variant) {
            mrt::AnalyticReference msg;
            msg.header.stamp = ros_received;
            msg.request_id = 100 + type;
            msg.trajectory_id = type + 1;
            msg.revision = variant;
            msg.analytic_type = static_cast<uint16_t>(type);
            msg.flags = variant == 2 ? 0x2u : 0u;
            msg.start_time = variant == 1 ? ros::Time() : ros_received + ros::Duration(0.25);
            msg.duration = variant == 0 ? 0.0 : 5.0 + u(rng);
            msg.origin.position.x = u(rng); msg.origin.position.y = -u(rng); msg.origin.position.z = 1.0 + u(rng);
            const double yaw = u(rng) - 1.0;
            msg.origin.orientation.z = std::sin(yaw / 2.0); msg.origin.orientation.w = std::cos(yaw / 2.0);
            for (int i = 0; i < 8; ++i) msg.params.push_back(variant == 2 && i == 1 ? NAN : u(rng));
            const bool accepted = cache.updateAnalytic(PMC_REF(msg), received);
            std::fprintf(out, "analytic type %d variant %d accepted %d\n", type, variant, accepted ? 1 : 0);
            probe(cache, received, "  after");
        }
    }
    for (int variant = 0; variant < 3; ++variant) {
        mrt::ActivePolynomialReference msg;
        msg.header.stamp = ros_received;
        msg.trajectory_id = 50 + variant;
        msg.revision = variant;
        msg.start_time = variant == 1 ? ros::Time() : ros_received;
        msg.order = 5;
        const int segments = 3 + variant;
        for (int s = 0; s < segments; ++s) msg.segment_durations.push_back(u(rng));
        if (variant == 2) msg.segment_durations.back() = -1.0;  // invalid
        for (auto* c : {&msg.coeff_x, &msg.coeff_y, &msg.coeff_z, &msg.coeff_yaw}) {
            for (int i = 0; i < segments * 6; ++i) c->push_back((u(rng) - 1.0) * 0.5);
        }
        msg.duration = 0.0;
        for (double s : msg.segment_durations) msg.duration += s;
        const bool accepted = cache.updatePolynomial(PMC_REF(msg), received);
        std::fprintf(out, "polynomial variant %d accepted %d\n", variant, accepted ? 1 : 0);
        probe(cache, received, "  after");
    }
    for (int variant = 0; variant < 3; ++variant) {
        mrt::SampledReference msg;
        msg.header.stamp = ros_received;
        msg.trajectory_id = 70 + variant;
        msg.revision = variant;
        msg.start_time = variant == 1 ? ros::Time() : ros_received + ros::Duration(0.1);
        msg.sample_dt = variant == 2 ? 0.0 : 0.1;
        for (int i = 0; i < 25; ++i) {
            mrt::FlatReferencePoint p;
            const double t = i * 0.1;
            p.t_from_start = t;
            p.position.x = std::sin(t); p.position.y = std::cos(t); p.position.z = 1.5 + 0.1 * t;
            p.velocity.x = std::cos(t); p.velocity.y = -std::sin(t); p.velocity.z = 0.1;
            p.acceleration.x = -std::sin(t); p.acceleration.y = -std::cos(t);
            p.jerk.x = -std::cos(t); p.jerk.y = std::sin(t);
            p.snap.x = std::sin(t); p.snap.y = std::cos(t);
            p.yaw = 0.1 * t; p.yaw_rate = 0.1;
            msg.points.push_back(p);
        }
        const bool accepted = cache.updateSampled(PMC_REF(msg), received);
        std::fprintf(out, "sampled variant %d accepted %d\n", variant, accepted ? 1 : 0);
        probe(cache, received, "  after");
    }
    cache.clear();
    probe(cache, received, "cleared");
    std::fclose(out);
    return 0;
}
