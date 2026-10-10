// The px4_multirotor_reference module against the ROS node's behavior
// (replay/reference_node_path.h) on the scripted request stream: the same requests at the same
// times must give the same published status and active references, bit for bit.
//
// Each 10 ms tick delivers the requests received since the previous tick to the node path (the
// callbacks of ros::spinOnce()) and, as payloads, to the module's ports, then updates both at the
// same nanosecond time. Every message the node's output consumer would publish must appear on the
// module's output ports, in the same order.

#include <gtest/gtest.h>

#include <module_support/test_host.hpp>
#include <string>
#include <vector>

#include "payload_conversion.h"
#include "replay/reference_node_path.h"
#include "replay/reference_trace.h"

namespace {

using module_support::ModuleLibrary;
using module_support::TestHost;
using reference_replay::Published;
using reference_replay::Record;
namespace mrt = multirotor_reference_trajectory;
namespace msgs = multirotor_reference_trajectory_msgs;
namespace conv = multirotor_reference_trajectory::module;
namespace trace = reference_replay::trace;

constexpr uint64_t kTickNs = 10000000;

// The ROS edge's mapping of a request message to the module's payload.
void deliver(TestHost& host, const Record& r) {
    switch (r.kind) {
        case reference_replay::kAnalyticRequest: {
            xgc2_px4_reference_analytic_v1 payload;
            ASSERT_TRUE(conv::toPayload(
                mrt::toCore(reference_replay::decode<msgs::AnalyticReference>(r.data)), payload));
            ASSERT_TRUE(host.push("analytic", payload, static_cast<int64_t>(r.t_ns)));
            break;
        }
        case reference_replay::kSampledRequest: {
            auto payload = std::make_unique<xgc2_px4_reference_sampled_v1>();
            ASSERT_TRUE(conv::toPayload(
                mrt::toCore(reference_replay::decode<msgs::SampledReference>(r.data)), *payload));
            ASSERT_TRUE(host.push("sampled", *payload, static_cast<int64_t>(r.t_ns)));
            break;
        }
        case reference_replay::kResetRequest: {
            xgc2_px4_reference_reset_v1 payload{};
            ASSERT_TRUE(host.push("reset", payload, static_cast<int64_t>(r.t_ns)));
            break;
        }
        default:
            FAIL() << "unknown record kind";
    }
}

TEST(ReferenceModuleParity, PublishesWhatTheRosNodeCorePublishes) {
    const auto records = reference_replay::readStream(REFERENCE_REPLAY_STREAM);
    ASSERT_FALSE(records.empty());

    ModuleLibrary library(REFERENCE_MODULE_PATH);
    TestHost host(library.desc());
    // config/multirotor_reference_trajectory.yaml
    mrt::ReferenceTrajectoryConfig config;
    config.limits.min_specific_thrust = 0.1;
    reference_replay::NodePath node(config);

    const uint64_t t0_ns = records.front().t_ns / kTickNs * kTickNs;
    const uint64_t t_end_ns = records.back().t_ns + 1000000000ULL;
    host.setNow(static_cast<int64_t>(t0_ns));
    ASSERT_EQ(host.create("{}"), XGC2_OK);
    ASSERT_EQ(host.start(), XGC2_OK);

    size_t next = 0;
    size_t statuses = 0, analytics = 0, sampleds = 0, requests = 0;
    for (uint64_t k = 0;; ++k) {
        const uint64_t tick_ns = t0_ns + k * kTickNs;
        if (tick_ns > t_end_ns)
            break;
        while (next < records.size() && records[next].t_ns <= tick_ns) {
            const Record& r = records[next++];
            node.receive(r, mrt::Time().fromNSec(r.t_ns).toSec());
            deliver(host, r);
            ++requests;
        }
        host.setNow(static_cast<int64_t>(tick_ns));
        ASSERT_EQ(host.step(), XGC2_OK) << "tick " << k;
        const std::vector<Published> expected = node.update(mrt::Time().fromNSec(tick_ns).toSec());

        // The same messages, in the order of the node's output events, per output port.
        std::vector<std::string> want_status, want_analytic, want_sampled;
        for (const Published& p : expected) {
            if (p.event.id == mrt::output_event_type::PUBLISH_STATUS)
                want_status.push_back(trace::text(p.status));
            if (p.event.id == mrt::output_event_type::PUBLISH_ACTIVE_ANALYTIC)
                want_analytic.push_back(trace::text(p.analytic));
            if (p.event.id == mrt::output_event_type::PUBLISH_ACTIVE_SAMPLED)
                want_sampled.push_back(trace::text(p.sampled));
        }
        std::vector<std::string> got_status, got_analytic, got_sampled;
        for (const auto& s : host.outputs("status")) {
            mrt::reference::ReferenceStatus m;
            conv::toCore(s.as<xgc2_px4_reference_status_v1>(), m);
            got_status.push_back(trace::text(m));
        }
        for (const auto& s : host.outputs("active_analytic")) {
            mrt::reference::AnalyticReference m;
            ASSERT_TRUE(conv::toCore(s.as<xgc2_px4_reference_analytic_v1>(), m));
            got_analytic.push_back(trace::text(m));
        }
        for (const auto& s : host.outputs("active_sampled")) {
            mrt::reference::SampledReference m;
            ASSERT_TRUE(conv::toCore(s.as<xgc2_px4_reference_sampled_v1>(), m));
            got_sampled.push_back(trace::text(m));
        }
        ASSERT_EQ(got_status, want_status) << "status at tick " << k;
        ASSERT_EQ(got_analytic, want_analytic) << "active analytic at tick " << k;
        ASSERT_EQ(got_sampled, want_sampled) << "active sampled at tick " << k;
        statuses += got_status.size();
        analytics += got_analytic.size();
        sampleds += got_sampled.size();
        host.clearOutputs();
        ASSERT_EQ(host.lastReportDetail().empty(), false);
    }
    // The scenario exercises every output: guard against a vacuous comparison.
    EXPECT_EQ(requests, records.size());
    EXPECT_GT(statuses, 100U);
    EXPECT_GT(analytics, 30U);
    EXPECT_GT(sampleds, 5U);
    EXPECT_TRUE(
        host.logged(2, "rejected sampled reference"));  // the scripted invalid sampled request
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
