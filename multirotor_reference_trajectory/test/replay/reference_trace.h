#pragma once

// Bit-exact text of the reference messages: doubles as hex bits, times as sec.nsec. The replay
// harness writes it, and the module test compares it, so "equal" means every bit.

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "multirotor_reference_trajectory/reference_types.h"

namespace reference_replay {

namespace trace {

namespace ref = multirotor_reference_trajectory::reference;

inline uint64_t bits(double v) {
    uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

class Text {
   public:
    const std::string& str() const {
        return s_;
    }

    void raw(const char* text) {
        s_ += text;
    }
    void u(unsigned v) {
        appendf(" %u", v);
    }
    void d(double v) {
        appendf(" %016" PRIx64, bits(v));
    }
    void time(const multirotor_reference_trajectory::Time& v) {
        appendf(" %u.%09u", v.sec, v.nsec);
    }
    template <typename V>
    void vec(const V& v) {
        d(v.x);
        d(v.y);
        d(v.z);
    }
    void header(const ref::Header& h) {
        appendf(" hdr %u", h.seq);
        time(h.stamp);
        appendf(" %s", h.frame_id.empty() ? "-" : h.frame_id.c_str());
    }

    void status(const ref::ReferenceStatus& m) {
        raw(" status");
        header(m.header);
        appendf(" st %u fl %u id %u rev %u type %u", m.state, m.flags, m.active_trajectory_id,
                m.active_revision, m.active_type);
    }

    void analytic(const ref::AnalyticReference& m) {
        raw(" analytic");
        header(m.header);
        appendf(" req %u id %u rev %u type %u fl %u", m.request_id, m.trajectory_id, m.revision,
                m.analytic_type, m.flags);
        time(m.start_time);
        d(m.duration);
        vec(m.origin.position);
        d(m.origin.orientation.x);
        d(m.origin.orientation.y);
        d(m.origin.orientation.z);
        d(m.origin.orientation.w);
        appendf(" [%zu", m.params.size());
        for (double x : m.params)
            d(x);
        raw("]");
    }

    void sampled(const ref::SampledReference& m) {
        raw(" sampled");
        header(m.header);
        appendf(" id %u rev %u fl %u", m.trajectory_id, m.revision, m.flags);
        time(m.start_time);
        d(m.sample_dt);
        appendf(" [%zu", m.points.size());
        for (const auto& p : m.points) {
            d(p.t_from_start);
            vec(p.position);
            vec(p.velocity);
            vec(p.acceleration);
            vec(p.jerk);
            vec(p.snap);
            d(p.yaw);
            d(p.yaw_rate);
            d(p.yaw_accel);
        }
        raw("]");
    }

   private:
    template <typename... Args>
    void appendf(const char* format, Args... args) {
        char buffer[160];
        const int n = std::snprintf(buffer, sizeof buffer, format, args...);
        if (n > 0)
            s_.append(buffer, static_cast<size_t>(n) < sizeof buffer ? n : sizeof buffer - 1);
    }

    std::string s_;
};

inline std::string text(const ref::ReferenceStatus& m) {
    Text t;
    t.status(m);
    return t.str();
}
inline std::string text(const ref::AnalyticReference& m) {
    Text t;
    t.analytic(m);
    return t.str();
}
inline std::string text(const ref::SampledReference& m) {
    Text t;
    t.sampled(m);
    return t.str();
}

}  // namespace trace

}  // namespace reference_replay
