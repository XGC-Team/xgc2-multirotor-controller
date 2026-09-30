#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "multirotor_reference_trajectory/ros_reference_conversion.h"

namespace {

namespace sm = ::state_machine;

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

void post(multirotor_reference_trajectory::ReferenceTrajectoryRuntime& runtime, uint32_t event_id,
          double timestamp) {
    sm::Event event(event_id, sm::EventTimestamp{timestamp});
    event.source = "runtime_test";
    const auto status = runtime.postEvent(std::move(event));
    ASSERT_TRUE(status.ok()) << status.message;
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
        ASSERT_TRUE(runtime.acceptAnalytic(multirotor_reference_trajectory::toCore(msg)))
            << "analytic_type=" << analytic_type;
        ASSERT_TRUE(runtime.activatePending()) << "analytic_type=" << analytic_type;
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
    ASSERT_TRUE(runtime.acceptAnalytic(multirotor_reference_trajectory::toCore(line)));
    post(runtime, multirotor_reference_trajectory::event_type::ANALYTIC_RECEIVED, 0.02);
    runtime.update(0.02);

    ASSERT_EQ(runtime.currentState(),
              multirotor_reference_trajectory_msgs::ReferenceStatus::STATE_ACTIVE);
    ASSERT_EQ(runtime.activeTrajectoryId(), line.trajectory_id);

    auto lemniscate = makeAnalyticCurveReference(
        multirotor_reference_trajectory_msgs::AnalyticReference::ANALYTIC_LEMNISCATE);
    lemniscate.trajectory_id = line.trajectory_id + 100U;
    lemniscate.revision = line.revision + 1U;
    ASSERT_TRUE(runtime.acceptAnalytic(multirotor_reference_trajectory::toCore(lemniscate)));
    post(runtime, multirotor_reference_trajectory::event_type::ANALYTIC_RECEIVED, 0.03);
    runtime.update(0.03);

    EXPECT_EQ(runtime.currentState(),
              multirotor_reference_trajectory_msgs::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(runtime.activeTrajectoryId(), lemniscate.trajectory_id);
    EXPECT_EQ(runtime.activeRevision(), lemniscate.revision);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
