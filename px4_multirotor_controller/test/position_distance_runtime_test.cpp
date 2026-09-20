#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/time.h>

#include <memory>

#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {
namespace {

// Real controller, state implementations and state-machine runtime. Outputs are
// deliberately NOT consumed: no ROS node, MAVLink command or vehicle is used.
class PositionDistanceRuntimeTest : public ::testing::Test {
   protected:
    void SetUp() override {
        ros::Time::init();
        sensor_.local_pos_stats.is_active = true;
        sensor_.local_velocity_stats.is_active = true;
        sensor_.imu_stats.is_active = true;
        sensor_.state_stats.is_active = true;
        sensor_.uav_state_estimate_stats.is_active = true;
        sensor_.vrpn_pose_stats.is_active = true;
        sensor_.uav_state_estimator_state =
            rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
        sensor_.fcu_connected = true;
        sensor_.local_z = sensor_.vrpn_z = sensor_.z = 1.0;
        sensor_.local_pos_stats.is_new = true;
        sensor_.vrpn_pose_stats.is_new = true;
    }

    void start(TrackingBackend backend = TrackingBackend::PX4_LOCAL) {
        controller_ = std::make_unique<DroneController>(sensor_);
        ControllerConfig config;
        config.tracking_backend = backend;
        controller_->setConfig(config);
        tick();
    }

    void tick() {
        controller_->update(now_ += 0.01);
        // Same consumption boundary as DroneRosNode::run / resetNewFlags.
        sensor_.local_pos_stats.is_new = false;
        sensor_.vrpn_pose_stats.is_new = false;
        sensor_.uav_state_estimate_stats.is_new = false;
        sensor_.state_stats.is_new = false;
        sensor_.imu_stats.is_new = false;
    }

    uint32_t state() {
        return controller_->getStateMachine().currentState(region_type::CONTROL);
    }

    void request(uint32_t event_id) {
        const auto status = controller_->getStateMachine().postEvent(
            ::state_machine::Event(event_id, ::state_machine::EventTimestamp{now_}));
        ASSERT_TRUE(status.ok()) << status.message;
        tick();
    }

    void enterFlightState(uint32_t target) {
        ASSERT_EQ(state(), state_type::Ready);
        request(event_type::TAKEOFF_REQUESTED);
        ASSERT_EQ(state(), state_type::TakeoffInit);
        if (target == state_type::TakeoffInit) {
            return;
        }
        request(event_type::ALTCTL_READY);
        ASSERT_EQ(state(), state_type::TakeoffOffboardRequest);
        if (target == state_type::TakeoffOffboardRequest) {
            return;
        }
        request(event_type::OFFBOARD_READY);
        ASSERT_EQ(state(), state_type::TakeoffArmRequest);
        if (target == state_type::TakeoffArmRequest) {
            return;
        }
        sensor_.fcu_armed = true;
        request(event_type::ARM_READY);
        ASSERT_EQ(state(), state_type::TakeoffAscending);
        if (target == state_type::TakeoffAscending) {
            return;
        }
        request(event_type::ALTITUDE_REACHED);
        ASSERT_EQ(state(), state_type::Hover);
        if (target == state_type::Hover) {
            return;
        }
        request(event_type::TRAJECTORY_TRACKING_REQUESTED);
        ASSERT_EQ(state(), state_type::Custom1);
    }

    SensorData sensor_;
    std::unique_ptr<DroneController> controller_;
    double now_{100.0};
};

TEST_F(PositionDistanceRuntimeTest, SameTickInputBlocksTakeoffAndBoundaryRecoveryAllowsIt) {
    start();
    ASSERT_EQ(state(), state_type::Ready);
    sensor_.vrpn_x = 1.001;
    sensor_.vrpn_pose_stats.is_new = true;
    request(event_type::TAKEOFF_REQUESTED);
    EXPECT_EQ(state(), state_type::Ready);
    EXPECT_TRUE(controller_->getPositionDistance().exceeded);

    // Clean ticks and unrelated commands must not recompute the distance.
    sensor_.vrpn_x = 1.0;
    for (int i = 0; i < 20; ++i) {
        tick();
        EXPECT_EQ(state(), state_type::Ready);
        EXPECT_DOUBLE_EQ(controller_->getPositionDistance().metres, 1.001);
    }
    sensor_.vrpn_pose_stats.is_new = true;
    request(event_type::TAKEOFF_REQUESTED);
    EXPECT_EQ(state(), state_type::TakeoffInit);
    EXPECT_DOUBLE_EQ(controller_->getPositionDistance().metres, 1.0);
    EXPECT_FALSE(controller_->getPositionDistance().exceeded);
}

TEST_F(PositionDistanceRuntimeTest, GroundStatesNeverLandBecauseOfPositionDistance) {
    sensor_.state_stats.is_active = false;
    sensor_.vrpn_x = 2.0;
    start();
    ASSERT_EQ(state(), state_type::SelfCheck);
    ASSERT_TRUE(controller_->getPositionDistance().exceeded);
    for (int i = 0; i < 20; ++i) {
        tick();
        EXPECT_EQ(state(), state_type::SelfCheck);
    }
    sensor_.state_stats.is_active = true;
    tick();
    ASSERT_EQ(state(), state_type::Ready);
    request(event_type::TAKEOFF_REQUESTED);
    EXPECT_EQ(state(), state_type::Ready);
    for (int i = 0; i < 20; ++i) {
        tick();
        EXPECT_EQ(state(), state_type::Ready);
    }
}

TEST_F(PositionDistanceRuntimeTest, UnobservedPairCannotApproveTakeoff) {
    sensor_.vrpn_pose_stats.is_new = false;
    start();
    ASSERT_EQ(state(), state_type::Ready);
    request(event_type::TAKEOFF_REQUESTED);
    EXPECT_EQ(state(), state_type::Ready);
    EXPECT_FALSE(controller_->getPositionDistance().available);
    sensor_.vrpn_pose_stats.is_new = true;
    request(event_type::TAKEOFF_REQUESTED);
    EXPECT_EQ(state(), state_type::TakeoffInit);
}

class PositionDistanceFlightTest : public PositionDistanceRuntimeTest,
                                   public ::testing::WithParamInterface<uint32_t> {};

TEST_P(PositionDistanceFlightTest, OnlyFlightStatesEnterExistingLandingOnce) {
    const auto target = GetParam();
    start(target == state_type::Custom1 ? TrackingBackend::DFBC : TrackingBackend::PX4_LOCAL);
    enterFlightState(target);
    ASSERT_EQ(state(), target);
    tick();
    ASSERT_EQ(state(), target);  // Normal distance does not interrupt flight.

    // In the early Takeoff children, also prove the parent safety rule wins
    // over the existing child-level armed/Ready transition.
    sensor_.fcu_armed = true;
    sensor_.vrpn_x = 1.01;
    sensor_.vrpn_pose_stats.is_new = true;
    tick();
    ASSERT_EQ(state(), state_type::Landing);

    // Landing::onEnter clears this latch; onTick does not. Its survival proves
    // no Landing re-entry, without timing sleeps or an added production counter.
    controller_->requestCustom1Tracking();
    for (int i = 0; i < 100; ++i) {
        tick();
        ASSERT_EQ(state(), state_type::Landing);
        ASSERT_TRUE(controller_->custom1Requested());
        for (const auto& record : controller_->getStateMachine().currentEvents()) {
            EXPECT_NE(record.event.id, event_type::LANDING_REQUESTED);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    ExistingFlightStates, PositionDistanceFlightTest,
    ::testing::Values(state_type::TakeoffInit, state_type::TakeoffOffboardRequest,
                      state_type::TakeoffArmRequest, state_type::TakeoffAscending,
                      state_type::Hover, state_type::Custom1));

}  // namespace
}  // namespace px4_multirotor_controller
