#include "multirotor_reference_trajectory/multirotor_reference_trajectory_runtime.h"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

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

multirotor_reference_trajectory::WaypointReferenceRequest makeWaypointRequest() {
    multirotor_reference_trajectory::WaypointReferenceRequest msg;
    msg.header.stamp = ros::Time(1.0);
    msg.trajectory_id = 42U;
    msg.revision = 7U;
    msg.objective = multirotor_reference_trajectory::WaypointReferenceRequest::OBJECTIVE_MINCO;
    msg.waypoints.push_back(pose(0.0, 0.0, 1.0));
    msg.waypoints.push_back(pose(1.0, 0.5, 1.2));
    msg.waypoints.push_back(pose(2.0, 0.0, 1.0));
    msg.segment_times = {1.0, 1.0};
    msg.desired_speed = 1.0;
    msg.time_weight = 0.1;
    msg.max_iterations = 80U;
    msg.rel_cost_tol = 1.0e-5;
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

TEST(ReferenceTrajectoryRuntime, WaypointPlanningUsesAsyncWorkerEvent) {
    multirotor_reference_trajectory::ReferenceTrajectoryRuntime runtime;
    multirotor_reference_trajectory::ReferenceTrajectoryConfig config;
    config.min_lead_time = 0.0;
    config.validation_sample_dt = 0.05;
    runtime.setConfig(config);

    runtime.update(0.0);
    runtime.update(0.01);
    ASSERT_EQ(runtime.currentState(),
              multirotor_reference_trajectory::ReferenceStatus::STATE_READY);

    const auto request = makeWaypointRequest();
    ASSERT_TRUE(runtime.acceptWaypoint(request));
    post(runtime, multirotor_reference_trajectory::event_type::WAYPOINT_RECEIVED, 0.02);

    runtime.update(0.02);
    ASSERT_EQ(runtime.currentState(),
              multirotor_reference_trajectory::ReferenceStatus::STATE_PLANNING);

    bool activated = false;
    for (int i = 0; i < 200; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        runtime.update(0.03 + static_cast<double>(i) * 0.01);
        if (runtime.currentState() ==
            multirotor_reference_trajectory::ReferenceStatus::STATE_ACTIVE) {
            activated = true;
            break;
        }
    }

    ASSERT_TRUE(activated) << "state=" << static_cast<int>(runtime.currentState())
                           << " flags=" << runtime.flags();
    EXPECT_EQ(runtime.activeType(),
              multirotor_reference_trajectory::trajectory::TrajectoryModelType::kPolynomial);
    EXPECT_EQ(runtime.activeTrajectoryId(), request.trajectory_id);
    EXPECT_EQ(runtime.activeRevision(), request.revision);
    EXPECT_NE(runtime.evaluator(), nullptr);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
