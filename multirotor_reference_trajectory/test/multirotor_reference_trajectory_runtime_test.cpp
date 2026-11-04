#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "multirotor_reference_trajectory/ros_reference_conversion.h"

namespace {

geometry_msgs::Pose pose(double x, double y, double z) {
    geometry_msgs::Pose out;
    out.position.x = x;
    out.position.y = y;
    out.position.z = z;
    out.orientation.w = 1.0;
    return out;
}

multirotor_reference_trajectory_msgs::AnalyticReference makeAnalyticCurveReference(
    uint16_t analytic_type) {
    multirotor_reference_trajectory_msgs::AnalyticReference msg;
    msg.header.stamp = ros::Time(1.0);
    msg.trajectory_id = 100U + analytic_type;
    msg.revision = 1U;
    msg.analytic_type = analytic_type;
    msg.start_time = ros::Time(1.0);
    msg.duration = 6.0;
    msg.origin = pose(0.0, 0.0, 1.5);
    switch (analytic_type) {
        case multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_LINE:
            msg.params = {1.0, 0.5, 2.0};
            break;
        case multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_LEMNISCATE:
            msg.params = {1.0, 0.7, 1.0};
            break;
        case multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HELIX_YZ:
            msg.params = {0.5, 0.6, 10.0};
            break;
        case multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HELIX_XY:
            msg.params = {0.5, 0.6, 10.0};
            break;
        case multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_TORUS_KNOT:
            msg.params = {0.5, 0.25};
            break;
        default:
            break;
    }
    return msg;
}

}  // namespace

TEST(ReferenceTrajectoryRuntime, AnalyticCurveReferencesActivate) {
    const uint16_t analytic_types[] = {
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_LINE,
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_LEMNISCATE,
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HELIX_YZ,
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HELIX_XY,
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_TORUS_KNOT,
    };

    for (const auto analytic_type : analytic_types) {
        multirotor_reference_trajectory::ReferenceTrajectoryRuntime runtime;
        multirotor_reference_trajectory::ReferenceTrajectoryConfig config;
        config.min_lead_time = 0.0;
        config.validation_sample_dt = 0.05;
        runtime.setConfig(config);
        runtime.update(0.0);

        const auto msg = makeAnalyticCurveReference(analytic_type);
        ASSERT_TRUE(runtime.acceptAnalytic(multirotor_reference_trajectory::toCore(msg), 0.01))
            << "analytic_type=" << analytic_type;
        runtime.update(0.01);
        ASSERT_NE(runtime.evaluator(), nullptr) << "analytic_type=" << analytic_type;
        EXPECT_EQ(runtime.activeType(),
                  multirotor_reference_trajectory::trajectory::TrajectoryModelType::kAnalytic);
        EXPECT_EQ(runtime.activeTrajectoryId(), msg.trajectory_id);

        multirotor_reference_trajectory::trajectory::FlatOutput3 output;
        ASSERT_TRUE(runtime.evaluator()->evaluate(0.5, output))
            << "analytic_type=" << analytic_type;
        EXPECT_TRUE(
            multirotor_reference_trajectory::trajectory::TrajectoryValidator3::finite(output))
            << "analytic_type=" << analytic_type;
    }
}

TEST(ReferenceTrajectoryRuntime, ActiveAnalyticCanSwitchToAnotherAnalytic) {
    multirotor_reference_trajectory::ReferenceTrajectoryRuntime runtime;
    multirotor_reference_trajectory::ReferenceTrajectoryConfig config;
    config.min_lead_time = 0.0;
    config.validation_sample_dt = 0.05;
    runtime.setConfig(config);

    runtime.update(0.0);
    runtime.update(0.01);
    ASSERT_EQ(runtime.currentState(),
              multirotor_reference_trajectory_msgs::ReferenceStatus::STATE_READY);

    const auto line = makeAnalyticCurveReference(
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_LINE);
    ASSERT_TRUE(runtime.acceptAnalytic(multirotor_reference_trajectory::toCore(line), 0.02));
    runtime.update(0.02);

    ASSERT_EQ(runtime.currentState(),
              multirotor_reference_trajectory_msgs::ReferenceStatus::STATE_ACTIVE);
    ASSERT_EQ(runtime.activeTrajectoryId(), line.trajectory_id);

    auto lemniscate = makeAnalyticCurveReference(
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_LEMNISCATE);
    lemniscate.trajectory_id = line.trajectory_id + 100U;
    lemniscate.revision = line.revision + 1U;
    ASSERT_TRUE(runtime.acceptAnalytic(multirotor_reference_trajectory::toCore(lemniscate), 0.03));
    runtime.update(0.03);

    EXPECT_EQ(runtime.currentState(),
              multirotor_reference_trajectory_msgs::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(runtime.activeTrajectoryId(), lemniscate.trajectory_id);
    EXPECT_EQ(runtime.activeRevision(), lemniscate.revision);
}

TEST(ReferenceTrajectoryRuntime, ExpiredReferenceIdentityRemainsUntilReset) {
    namespace mrt = multirotor_reference_trajectory;
    mrt::ReferenceTrajectoryRuntime runtime;
    mrt::ReferenceTrajectoryConfig config;
    config.min_lead_time = 0.0;
    config.trajectory_timeout = 0.5;
    runtime.setConfig(config);
    runtime.update(0.0);

    auto message = makeAnalyticCurveReference(
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_LINE);
    message.start_time = ros::Time(1.0);
    message.duration = 1.0;
    ASSERT_TRUE(runtime.acceptAnalytic(mrt::toCore(message), 0.01));
    runtime.update(0.01);
    ASSERT_EQ(runtime.currentState(), mrt::reference::ReferenceStatus::STATE_ACTIVE);

    runtime.update(2.51);
    runtime.update(2.52);
    const auto ready = runtime.makeStatus(2.52);
    EXPECT_EQ(ready.state, mrt::reference::ReferenceStatus::STATE_READY);
    EXPECT_EQ(ready.active_type, mrt::reference::ReferenceStatus::TYPE_ANALYTIC);
    EXPECT_EQ(ready.active_trajectory_id, message.trajectory_id);
    EXPECT_EQ(ready.active_revision, message.revision);

    runtime.reset();
    const auto reset = runtime.makeStatus(2.53);
    EXPECT_EQ(reset.state, mrt::reference::ReferenceStatus::STATE_SELF_CHECK);
    EXPECT_EQ(reset.active_type, mrt::reference::ReferenceStatus::TYPE_NONE);
    EXPECT_EQ(reset.active_trajectory_id, 0U);
    EXPECT_EQ(reset.active_revision, 0U);
}

TEST(ReferenceTrajectoryRuntime, ReceiptLeadFutureStartAndStrictExpiry) {
    namespace mrt = multirotor_reference_trajectory;
    mrt::ReferenceTrajectoryRuntime runtime;
    mrt::ReferenceTrajectoryConfig config;
    config.min_lead_time = 0.2;
    config.trajectory_timeout = 0.5;
    runtime.setConfig(config);
    runtime.update(100.0);
    auto message = makeAnalyticCurveReference(
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_LINE);
    message.duration = 2.0;
    message.start_time = ros::Time(100.05);
    ASSERT_TRUE(runtime.acceptAnalytic(mrt::toCore(message), 100.01));
    runtime.update(100.01);
    EXPECT_NEAR(runtime.activeAnalyticMessage().start_time.toSec(), 100.21, 1e-9);

    message.start_time = ros::Time(105.0);
    message.trajectory_id += 1;
    ASSERT_TRUE(runtime.acceptAnalytic(mrt::toCore(message), 100.02));
    runtime.update(100.02);
    ASSERT_DOUBLE_EQ(runtime.activeAnalyticMessage().start_time.toSec(), 105.0);
    ASSERT_NE(runtime.evaluator(), nullptr);
    EXPECT_DOUBLE_EQ(runtime.evaluator()->duration(), 2.0);

    runtime.update(107.5);  // Equal to start + duration + timeout: still active.
    EXPECT_EQ(runtime.currentState(), mrt::reference::ReferenceStatus::STATE_ACTIVE);
    runtime.update(107.500001);
    EXPECT_EQ(runtime.currentState(), mrt::reference::ReferenceStatus::STATE_READY);
    EXPECT_EQ(runtime.activeTrajectoryId(), message.trajectory_id);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
