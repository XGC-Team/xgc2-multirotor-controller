// The px4_multirotor_reference module, loaded as a shared library and driven through the module ABI
// by an in-test host.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <module_support/test_host.hpp>
#include <string>

#include "multirotor_reference_trajectory/payloads.h"
#include "multirotor_reference_trajectory/reference_types.h"
#include "payload_conversion.h"

namespace {

using module_support::ModuleLibrary;
using module_support::TestHost;
namespace mrt = multirotor_reference_trajectory;
namespace conv = multirotor_reference_trajectory::module;

constexpr int64_t kTickNs = 10000000;  // 100 Hz
constexpr int64_t kT0 = 1000000000000LL;

class ReferenceModuleTest : public ::testing::Test {
   protected:
    void SetUp() override {
        library_ = std::make_unique<ModuleLibrary>(REFERENCE_MODULE_PATH);
    }

    void TearDown() override {
        if (!HasFailure() || !host_)
            return;
        for (const auto& entry : host_->logs()) {
            std::fprintf(stderr, "module log [%d] %s\n", entry.first, entry.second.c_str());
        }
    }

    void open(const std::string& config = "{}") {
        host_ = std::make_unique<TestHost>(library_->desc());
        host_->setNow(kT0);
        ASSERT_EQ(host_->create(config), XGC2_OK);
        ASSERT_EQ(host_->start(), XGC2_OK);
    }

    // Runs `count` 10 ms ticks; the first is one tick after the current clock.
    void run(int count) {
        for (int i = 0; i < count; ++i) {
            host_->setNow(host_->now() + kTickNs);
            ASSERT_EQ(host_->step(), XGC2_OK);
        }
    }

    static xgc2_px4_reference_analytic_v1 analytic(uint16_t type, std::vector<double> params,
                                                   double duration = 6.0) {
        mrt::reference::AnalyticReference m;
        m.header.stamp = mrt::Time(1000.0);
        m.header.frame_id = "map";
        m.request_id = 7;
        m.trajectory_id = 11;
        m.revision = 3;
        m.analytic_type = type;
        m.duration = duration;
        m.origin.position = {0.5, -0.5, 1.5};
        m.origin.orientation = {0.0, 0.0, 0.0, 1.0};
        m.params = std::move(params);
        xgc2_px4_reference_analytic_v1 payload;
        std::memset(&payload, 0, sizeof payload);
        EXPECT_TRUE(conv::toPayload(m, payload));
        return payload;
    }

    static std::unique_ptr<xgc2_px4_reference_sampled_v1> sampled(int points, double dt = 0.1) {
        mrt::reference::SampledReference m;
        m.header.stamp = mrt::Time(1000.0);
        m.trajectory_id = 21;
        m.revision = 2;
        m.sample_dt = dt;
        for (int i = 0; i < points; ++i) {
            mrt::reference::FlatReferencePoint p;
            const double s = i * dt;
            p.t_from_start = s;
            p.position = {std::cos(0.5 * s), std::sin(0.5 * s), 1.0 + 0.1 * s};
            p.velocity = {-0.5 * std::sin(0.5 * s), 0.5 * std::cos(0.5 * s), 0.1};
            p.acceleration = {-0.25 * std::cos(0.5 * s), -0.25 * std::sin(0.5 * s), 0.0};
            p.jerk = {0.125 * std::sin(0.5 * s), -0.125 * std::cos(0.5 * s), 0.0};
            p.snap = {0.0625 * std::cos(0.5 * s), 0.0625 * std::sin(0.5 * s), 0.0};
            p.yaw = 0.2 * s;
            p.yaw_rate = 0.2;
            m.points.push_back(p);
        }
        auto payload = std::make_unique<xgc2_px4_reference_sampled_v1>();
        std::memset(payload.get(), 0, sizeof *payload);
        EXPECT_TRUE(conv::toPayload(m, *payload));
        return payload;
    }

    void pushAnalytic(const xgc2_px4_reference_analytic_v1& payload) {
        ASSERT_TRUE(host_->push("analytic", payload, host_->now()));
    }

    std::unique_ptr<ModuleLibrary> library_;
    std::unique_ptr<TestHost> host_;
};

TEST_F(ReferenceModuleTest, DescriptorDeclaresTheAbiAndTheTypedPorts) {
    const xgc2_module_desc* desc = library_->desc();
    ASSERT_NE(desc, nullptr);
    EXPECT_EQ(desc->abi_major, XGC2_MODULE_ABI_MAJOR);
    EXPECT_EQ(desc->abi_minor, XGC2_MODULE_ABI_MINOR);
    EXPECT_STREQ(desc->name, "px4_multirotor_reference");
    EXPECT_GT(std::strlen(desc->version), 0U);
    ASSERT_NE(desc->create, nullptr);
    ASSERT_NE(desc->configure, nullptr);
    ASSERT_NE(desc->start, nullptr);
    ASSERT_NE(desc->step, nullptr);
    ASSERT_NE(desc->stop, nullptr);
    ASSERT_NE(desc->destroy, nullptr);

    struct Expected {
        const char* name;
        uint32_t direction;
        uint32_t kind;
        const char* schema;
        uint32_t size;
        uint32_t align;
        bool event;
    };
    const Expected expected[] = {
        {"analytic", XGC2_PORT_IN, XGC2_PORT_EVENT, "xgc2.px4.reference_analytic.v1",
         sizeof(xgc2_px4_reference_analytic_v1), alignof(xgc2_px4_reference_analytic_v1), true},
        {"sampled", XGC2_PORT_IN, XGC2_PORT_EVENT, "xgc2.px4.reference_sampled.v1",
         sizeof(xgc2_px4_reference_sampled_v1), alignof(xgc2_px4_reference_sampled_v1), true},
        {"reset", XGC2_PORT_IN, XGC2_PORT_EVENT, "xgc2.px4.reference_reset.v1",
         sizeof(xgc2_px4_reference_reset_v1), alignof(xgc2_px4_reference_reset_v1), true},
        {"status", XGC2_PORT_OUT, XGC2_PORT_STATE, "xgc2.px4.reference_status.v1",
         sizeof(xgc2_px4_reference_status_v1), alignof(xgc2_px4_reference_status_v1), false},
        {"active_analytic", XGC2_PORT_OUT, XGC2_PORT_STATE, "xgc2.px4.reference_analytic.v1",
         sizeof(xgc2_px4_reference_analytic_v1), alignof(xgc2_px4_reference_analytic_v1), false},
        {"active_sampled", XGC2_PORT_OUT, XGC2_PORT_STATE, "xgc2.px4.reference_sampled.v1",
         sizeof(xgc2_px4_reference_sampled_v1), alignof(xgc2_px4_reference_sampled_v1), false},
    };
    ASSERT_EQ(desc->port_count, sizeof expected / sizeof expected[0]);
    for (uint32_t i = 0; i < desc->port_count; ++i) {
        const xgc2_port_desc& p = desc->ports[i];
        EXPECT_STREQ(p.name, expected[i].name);
        EXPECT_EQ(p.direction, expected[i].direction) << p.name;
        EXPECT_EQ(p.kind, expected[i].kind) << p.name;
        EXPECT_STREQ(p.schema_id, expected[i].schema) << p.name;
        EXPECT_EQ(p.size, expected[i].size) << p.name;
        EXPECT_EQ(p.align, expected[i].align) << p.name;
        EXPECT_EQ(p.queue_depth >= 1, expected[i].event) << p.name;
        EXPECT_EQ(p.flags, 0U) << p.name;
    }
}

TEST_F(ReferenceModuleTest, RejectsAnInvalidConfiguration) {
    for (const char* bad : {"[]", "{", "{\"main_frequency\": 0}", "{\"main_frequency\": \"fast\"}",
                            "{\"status_rte\": 10}", "{\"max_jerk\": null}"}) {
        TestHost host(library_->desc());
        EXPECT_EQ(host.create(bad), XGC2_ERR_INVALID) << bad;
        EXPECT_EQ(host.instance(), nullptr) << bad;
        EXPECT_FALSE(host.logs().empty()) << bad;
    }
}

TEST_F(ReferenceModuleTest, NamesTheUnknownKeyInTheError) {
    TestHost host(library_->desc());
    EXPECT_EQ(host.create("{\"status_rte\": 10}"), XGC2_ERR_INVALID);
    EXPECT_TRUE(host.logged(3, "unknown configuration key 'status_rte'"));
}

TEST_F(ReferenceModuleTest, StartAsksForTheNodePeriodAndReportsItsState) {
    open();
    EXPECT_EQ(host_->periodNs(), 10000000);  // main_frequency 100 Hz, as the ROS node
    EXPECT_EQ(host_->lastReportDetail(), "SelfCheck");
    run(2);
    EXPECT_EQ(host_->lastReportDetail(), "Ready");
    const auto status = host_->outputsAs<xgc2_px4_reference_status_v1>("status");
    ASSERT_FALSE(status.empty());
    EXPECT_EQ(status.back().state, mrt::reference::ReferenceStatus::STATE_READY);
    EXPECT_EQ(status.back().active_type, mrt::reference::ReferenceStatus::TYPE_NONE);
    EXPECT_EQ(host_->reports().front().first, 0);
}

TEST_F(ReferenceModuleTest, ConfigurationKeysOfTheNodeAreAccepted) {
    open(
        "{\"main_frequency\": 50.0, \"status_rate\": 5.0, \"active_publish_rate\": 2.0, "
        "\"validation_sample_dt\": 0.05, \"trajectory_timeout\": 1.0, \"min_lead_time\": 0.5, "
        "\"max_velocity\": 4.0, \"max_acceleration\": 6.0, \"max_jerk\": 0.0, \"max_snap\": 0.0, "
        "\"min_specific_thrust\": 0.1}");
    EXPECT_EQ(host_->periodNs(), 20000000);
}

TEST_F(ReferenceModuleTest, AnalyticRequestBecomesTheActiveReference) {
    open();
    run(5);
    pushAnalytic(analytic(mrt::reference::AnalyticReference::ANALYTIC_CIRCLE_ENTRY,
                          {1.5, 1.0, 1.2, 0.2, 0.3, 2.0, 0.5, -0.5}, 8.0));
    run(15);  // the active reference is published at 10 Hz
    EXPECT_EQ(host_->lastReportDetail(), "Active");

    const auto active = host_->outputsAs<xgc2_px4_reference_analytic_v1>("active_analytic");
    ASSERT_FALSE(active.empty());
    const auto& a = active.back();
    EXPECT_EQ(a.request_id, 7U);
    EXPECT_EQ(a.trajectory_id, 11U);
    EXPECT_EQ(a.revision, 3U);
    EXPECT_STREQ(a.header.frame_id, "map");
    ASSERT_EQ(a.params_len, 8U);
    EXPECT_EQ(a.params[0], 1.5);
    EXPECT_EQ(a.params[7], -0.5);
    EXPECT_EQ(a.duration, 8.0);
    // Start time zero: the runtime starts it after the lead time (0.2 s) from the request.
    const double start = a.start_sec + 1e-9 * a.start_nsec;
    const double requested_at = kT0 * 1e-9 + 5 * 0.01;
    EXPECT_GE(start, requested_at + 0.2 - 1e-6);
    EXPECT_LE(start, requested_at + 0.2 + 0.02);

    const auto status = host_->outputsAs<xgc2_px4_reference_status_v1>("status");
    EXPECT_EQ(status.back().state, mrt::reference::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(status.back().active_type, mrt::reference::ReferenceStatus::TYPE_ANALYTIC);
    EXPECT_EQ(status.back().active_trajectory_id, 11U);
    EXPECT_EQ(status.back().active_revision, 3U);
    EXPECT_TRUE(host_->outputs("active_sampled").empty());
}

TEST_F(ReferenceModuleTest, AStepWithOnlyRequestsDeliversThemAndLeavesTheUpdateToThePeriod) {
    open();
    run(5);
    host_->clearOutputs();
    host_->setNow(host_->now() + 200000000);  // the status is due
    auto bad = analytic(mrt::reference::AnalyticReference::ANALYTIC_HOLD, {}, 6.0);
    bad.origin_position[0] = std::nan("");  // a request the runtime refuses
    pushAnalytic(bad);
    // Like the node's callback between two iterations of its loop: the request is delivered ...
    ASSERT_EQ(host_->step(XGC2_STEP_INPUT), XGC2_OK);
    EXPECT_TRUE(host_->logged(2, "rejected analytic reference"));
    // ... without an update of the runtime ...
    EXPECT_TRUE(host_->outputs("status").empty());
    // ... which the next period makes.
    ASSERT_EQ(host_->step(XGC2_STEP_TIMER), XGC2_OK);
    EXPECT_EQ(host_->outputs("status").size(), 1U);
}

TEST_F(ReferenceModuleTest, StepsOnDifferentWorkerThreadsDriveTheSameRuntime) {
    // The in-test host calls the module on a different worker thread each time, as the host's pool
    // may; the runtime's state machine accepts updates from one thread only.
    open();
    run(5);
    pushAnalytic(analytic(mrt::reference::AnalyticReference::ANALYTIC_HOLD, {}, 30.0));
    run(15);
    EXPECT_EQ(host_->lastReportDetail(), "Active");
    EXPECT_GT(host_->workerThreadsUsed(), 1U);
}

TEST_F(ReferenceModuleTest, ActiveReferenceIsPublishedAtTheConfiguredRate) {
    open("{\"active_publish_rate\": 10.0, \"status_rate\": 10.0}");
    run(3);
    pushAnalytic(analytic(mrt::reference::AnalyticReference::ANALYTIC_HOLD, {}, 30.0));
    run(100);  // 1 s
    const size_t active = host_->outputs("active_analytic").size();
    EXPECT_GE(active, 9U);
    EXPECT_LE(active, 12U);
    // The state ports carry the step's time as the sample's stamp.
    for (const auto& s : host_->outputs("active_analytic")) {
        EXPECT_GE(s.stamp_ns, kT0);
        EXPECT_EQ((s.stamp_ns - kT0) % kTickNs, 0);
    }
}

TEST_F(ReferenceModuleTest, SampledRequestIsPublishedWithItsUsedPoints) {
    open();
    run(3);
    const auto request = sampled(40);
    ASSERT_TRUE(host_->push("sampled", *request, host_->now()));
    run(15);
    const auto active = host_->outputs("active_sampled");
    ASSERT_FALSE(active.empty());
    const auto& bytes = active.back().bytes;
    ASSERT_EQ(bytes.size(), sizeof(xgc2_px4_reference_sampled_v1));
    xgc2_px4_reference_sampled_v1 head;
    std::memcpy(&head, bytes.data(), sizeof head);
    EXPECT_EQ(head.trajectory_id, 21U);
    EXPECT_EQ(head.points_len, 40U);
    EXPECT_EQ(head.points[39].t_from_start, 39 * 0.1);
    EXPECT_EQ(head.points[39].position[2], request->points[39].position[2]);
    const auto status = host_->outputsAs<xgc2_px4_reference_status_v1>("status");
    EXPECT_EQ(status.back().active_type, mrt::reference::ReferenceStatus::TYPE_SAMPLED);
}

TEST_F(ReferenceModuleTest, RejectedRequestsLeaveTheRuntimeReady) {
    open();
    run(5);
    // Not finite: the runtime's validation refuses it.
    auto bad = analytic(mrt::reference::AnalyticReference::ANALYTIC_HOLD, {}, 6.0);
    bad.origin_position[0] = std::nan("");
    pushAnalytic(bad);
    // A length beyond the payload capacity is refused before the runtime sees it.
    auto too_long = analytic(mrt::reference::AnalyticReference::ANALYTIC_HOLD, {}, 6.0);
    too_long.params_len = XGC2_PX4_REFERENCE_MAX_PARAMS + 1;
    pushAnalytic(too_long);
    // A sampled reference whose times do not increase.
    auto unordered = sampled(10);
    unordered->points[5].t_from_start = 0.0;
    ASSERT_TRUE(host_->push("sampled", *unordered, host_->now()));
    run(5);
    EXPECT_EQ(host_->lastReportDetail(), "Ready");
    EXPECT_TRUE(host_->outputs("active_analytic").empty());
    EXPECT_TRUE(host_->outputs("active_sampled").empty());
    EXPECT_TRUE(host_->logged(2, "rejected analytic reference"));
    EXPECT_TRUE(host_->logged(2, "rejected sampled reference"));
}

TEST_F(ReferenceModuleTest, ResetReturnsToSelfCheckAndDropsTheReference) {
    open();
    run(3);
    pushAnalytic(analytic(mrt::reference::AnalyticReference::ANALYTIC_HOLD, {}, 30.0));
    run(5);
    ASSERT_EQ(host_->lastReportDetail(), "Active");
    xgc2_px4_reference_reset_v1 reset{};
    ASSERT_TRUE(host_->push("reset", reset, host_->now()));
    host_->clearOutputs();
    run(30);
    // The reset starts the runtime over: it becomes Ready again, with nothing active.
    EXPECT_EQ(host_->lastReportDetail(), "Ready");
    const auto status = host_->outputsAs<xgc2_px4_reference_status_v1>("status");
    ASSERT_FALSE(status.empty());
    EXPECT_EQ(status.back().state, mrt::reference::ReferenceStatus::STATE_READY);
    EXPECT_EQ(status.back().active_type, mrt::reference::ReferenceStatus::TYPE_NONE);
    EXPECT_EQ(status.back().active_trajectory_id, 0U);
    EXPECT_TRUE(host_->outputs("active_analytic").empty());
}

TEST_F(ReferenceModuleTest, StopRefusesStepsAndStartBeginsFresh) {
    open();
    run(3);
    pushAnalytic(analytic(mrt::reference::AnalyticReference::ANALYTIC_HOLD, {}, 30.0));
    run(5);
    ASSERT_EQ(host_->lastReportDetail(), "Active");
    ASSERT_EQ(host_->stop(), XGC2_OK);
    host_->setNow(host_->now() + kTickNs);
    EXPECT_EQ(host_->step(), XGC2_ERR_STATE);
    ASSERT_EQ(host_->start(), XGC2_OK);
    EXPECT_EQ(host_->lastReportDetail(), "SelfCheck");
    host_->clearOutputs();
    run(5);
    EXPECT_TRUE(host_->outputs("active_analytic").empty());
}

TEST_F(ReferenceModuleTest, ConfigureWhileRunningAppliesAndRestartsTheRuntime) {
    open();
    run(3);
    pushAnalytic(analytic(mrt::reference::AnalyticReference::ANALYTIC_HOLD, {}, 30.0));
    run(5);
    ASSERT_EQ(host_->lastReportDetail(), "Active");
    ASSERT_EQ(host_->configure("{\"main_frequency\": 200.0}"), XGC2_OK);
    EXPECT_EQ(host_->periodNs(), 5000000);
    EXPECT_EQ(host_->lastReportDetail(), "SelfCheck");
    EXPECT_EQ(host_->configure("{\"main_frequency\": -1}"), XGC2_ERR_INVALID);
    EXPECT_EQ(host_->periodNs(), 5000000);  // the refused configuration changed nothing
}

TEST_F(ReferenceModuleTest, AFailedOutputDegradesTheHealth) {
    open();
    run(3);
    host_->limitOutput("status", 0);
    run(30);
    EXPECT_GT(host_->refusedWrites("status"), 0U);
    EXPECT_EQ(host_->reports().back().first,
              1);  // degraded, and it stays so until a write succeeds
    EXPECT_TRUE(host_->logged(3, "output status is not writable"));
}

TEST_F(ReferenceModuleTest, RequestsAreProcessedInArrivalOrderAcrossPorts) {
    open();
    run(3);
    const auto request = analytic(mrt::reference::AnalyticReference::ANALYTIC_HOLD, {}, 30.0);
    xgc2_px4_reference_reset_v1 reset{};
    // The reset arrived after the request: nothing is activated.
    ASSERT_TRUE(host_->push("reset", reset, host_->now() + 2000));
    ASSERT_TRUE(host_->push("analytic", request, host_->now() + 1000));
    run(5);
    EXPECT_EQ(host_->lastReportDetail(), "Ready");
    EXPECT_TRUE(host_->outputs("active_analytic").empty());
    // The request arrives in a later step than the reset: it is activated. (A request in the same
    // step as a reset meets the runtime in SelfCheck, which does not take requests.)
    ASSERT_TRUE(host_->push("reset", reset, host_->now()));
    run(3);
    pushAnalytic(request);
    run(15);
    EXPECT_EQ(host_->lastReportDetail(), "Active");
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
