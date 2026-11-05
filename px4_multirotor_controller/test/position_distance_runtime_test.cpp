#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <limits>
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
        config.takeoff_altitude = 1.0;
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
        return static_cast<uint32_t>(controller_->flightState());
    }

    void request(uint32_t event_id) {
        if (event_id == event_type::ALTCTL_READY) {
            sensor_.fcu_mode = "ALTCTL";
            for (int i = 0; i < 2; ++i) { sensor_.state_stats.is_new = true; tick(); }
            return;
        }
        if (event_id == event_type::OFFBOARD_READY) {
            sensor_.fcu_mode = "OFFBOARD";
            sensor_.state_stats.is_new = true; tick(); return;
        }
        if (event_id == event_type::ARM_READY) {
            sensor_.fcu_armed = true;
            sensor_.state_stats.is_new = true; tick(); return;
        }
        if (event_id == event_type::ALTITUDE_REACHED) {
            sensor_.local_z = sensor_.vrpn_z = sensor_.z = controller_->getSetpoint().z;
            for (int i = 0; i < 5; ++i) {
                sensor_.local_pos_stats.is_new = sensor_.vrpn_pose_stats.is_new = true;
                sensor_.uav_state_estimate_stats.is_new = true;
                tick();
            }
            return;
        }
        const auto command = event_id == event_type::TAKEOFF_REQUESTED ? FlightCommand::Takeoff
            : event_id == event_type::HOVER_REQUESTED ? FlightCommand::Hover
            : event_id == event_type::LANDING_REQUESTED ? FlightCommand::Land : FlightCommand::Track;
        ASSERT_TRUE(controller_->postCommand(command, now_));
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

TEST_F(PositionDistanceRuntimeTest, ReadySelectsTakeoffBeforeLocalSensorLoss) {
    start();
    ASSERT_EQ(state(), state_type::Ready);
    // For PX4_LOCAL the existing health path does not emit a local-position
    // timeout edge. Both Ready rules are true on this turn.
    sensor_.local_pos_stats.is_active = false;
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Takeoff, now_));
    tick();
    EXPECT_EQ(state(), state_type::TakeoffInit);
}

TEST_F(PositionDistanceRuntimeTest, ReadySelectsAirborneBeforeTakeoff) {
    start();
    ASSERT_EQ(state(), state_type::Ready);
    sensor_.fcu_armed = true;
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Takeoff, now_));
    tick();
    EXPECT_EQ(state(), state_type::Landing);
}

TEST_F(PositionDistanceRuntimeTest, ReadySelectsFcuTimeoutBeforeTakeoff) {
    start();
    ASSERT_EQ(state(), state_type::Ready);
    sensor_.state_stats.is_active = false;
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Takeoff, now_));
    tick();
    EXPECT_EQ(state(), state_type::Landing);
}

TEST_F(PositionDistanceRuntimeTest, TransitionKeepsBothDueAndLandingSetpointsInOrder) {
    start();
    enterFlightState(state_type::Hover);
    now_ += 0.04;  // Hover's existing 30 Hz publication is due on the next turn.
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Land, now_));
    tick();
    ASSERT_EQ(state(), state_type::Landing);
    const auto& outputs = controller_->flightEffects();
    ASSERT_GE(outputs.size(), 3U);
    EXPECT_TRUE(std::holds_alternative<FlightSetpoint>(outputs[0]));
    EXPECT_TRUE(std::holds_alternative<FlightSetpoint>(outputs[1]));
    EXPECT_TRUE(std::holds_alternative<FlightEvents>(outputs[2]));
}

TEST_F(PositionDistanceRuntimeTest, TakeoffPublishesSetpointBeforeInitialDisarmRequest) {
    start();
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Takeoff, now_));
    tick();
    ASSERT_EQ(state(), state_type::TakeoffInit);
    const auto& outputs = controller_->flightEffects();
    ASSERT_GE(outputs.size(), 2U);
    EXPECT_TRUE(std::holds_alternative<FlightSetpoint>(outputs[0]));
    const auto* disarm = std::get_if<FlightArmRequest>(&outputs[1]);
    ASSERT_NE(disarm, nullptr);
    EXPECT_FALSE(disarm->arm);
}

TEST_F(PositionDistanceRuntimeTest, CommandAfterFullTurnBatchRemainsForNextTurn) {
    start();
    for (int i = 0; i < 64; ++i)
        ASSERT_TRUE(controller_->postCommand(FlightCommand::Hover, now_));
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Takeoff, now_));
    tick();
    EXPECT_EQ(state(), state_type::Ready);
    tick();
    EXPECT_EQ(state(), state_type::TakeoffInit);
}

TEST_F(PositionDistanceRuntimeTest, FullOriginalCommandQueueReportsFailure) {
    start();
    for (int i = 0; i < 4096; ++i)
        ASSERT_TRUE(controller_->postCommand(FlightCommand::Hover, now_));
    const auto result = controller_->postCommand(FlightCommand::Land, now_);
    EXPECT_FALSE(result);
}

class SameTurnCommandTest : public PositionDistanceRuntimeTest,
                            public ::testing::WithParamInterface<bool> {};

TEST_P(SameTurnCommandTest, LandWinsOverTrackingRegardlessOfArrivalOrder) {
    start(TrackingBackend::DFBC);
    enterFlightState(state_type::Hover);
    const FlightCommand first = GetParam() ? FlightCommand::Land : FlightCommand::Track;
    const FlightCommand second = GetParam() ? FlightCommand::Track : FlightCommand::Land;
    ASSERT_TRUE(controller_->postCommand(first, now_));
    ASSERT_TRUE(controller_->postCommand(second, now_));
    tick();
    ASSERT_EQ(state(), state_type::Landing);
    for (const auto& output : controller_->flightEffects()) {
        EXPECT_FALSE(std::holds_alternative<FlightReferenceActivation>(output));
    }
}

TEST_P(SameTurnCommandTest, HoverWinsOverTrackingDuringAscentInEitherArrivalOrder) {
    start(TrackingBackend::DFBC);
    enterFlightState(state_type::TakeoffAscending);
    const FlightCommand first = GetParam() ? FlightCommand::Hover : FlightCommand::Track;
    const FlightCommand second = GetParam() ? FlightCommand::Track : FlightCommand::Hover;
    ASSERT_TRUE(controller_->postCommand(first, now_));
    ASSERT_TRUE(controller_->postCommand(second, now_));
    tick();
    EXPECT_EQ(state(), state_type::Hover);
    EXPECT_FALSE(controller_->custom1Requested());
    tick();
    EXPECT_EQ(state(), state_type::Hover);
}

INSTANTIATE_TEST_SUITE_P(BothArrivalOrders, SameTurnCommandTest, ::testing::Bool());

TEST_F(PositionDistanceRuntimeTest, SameTurnPositionSafetyWinsOverTracking) {
    start(TrackingBackend::DFBC);
    enterFlightState(state_type::Hover);
    sensor_.vrpn_x = 1.01;
    sensor_.vrpn_pose_stats.is_new = true;
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Track, now_));
    tick();
    ASSERT_TRUE(controller_->getPositionDistance().exceeded);
    ASSERT_EQ(state(), state_type::Landing);
    for (const auto& output : controller_->flightEffects()) {
        EXPECT_FALSE(std::holds_alternative<FlightReferenceActivation>(output));
    }
}

TEST_F(PositionDistanceRuntimeTest, LandWinsOverArmedReturnDuringTakeoffInit) {
    start();
    enterFlightState(state_type::TakeoffInit);
    sensor_.fcu_armed = true;
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Land, now_));
    tick();
    EXPECT_EQ(state(), state_type::Landing);
}

TEST_F(PositionDistanceRuntimeTest, DeferredCompletionIsDiagnosedOnlyWhenItActuallyOccurs) {
    start();
    enterFlightState(state_type::TakeoffAscending);
    sensor_.local_z = sensor_.vrpn_z = sensor_.z = controller_->getSetpoint().z;
    for (int i = 0; i < 4; ++i) {
        sensor_.local_pos_stats.is_new = sensor_.vrpn_pose_stats.is_new = true;
        tick();
        ASSERT_EQ(state(), state_type::TakeoffAscending);
    }
    sensor_.local_pos_stats.is_new = sensor_.vrpn_pose_stats.is_new = true;
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Land, now_));
    tick();
    ASSERT_EQ(state(), state_type::Landing);
    EXPECT_EQ(controller_->flightDiagnosticEvents(),
              (std::vector<uint32_t>{event_type::LANDING_REQUESTED, event_type::ALTITUDE_REACHED}));

    // The old engine makes the unselected completion visible to the next
    // control turn; that is the same completion, not a second occurrence.
    tick();
    EXPECT_TRUE(controller_->flightDiagnosticEvents().empty());
}

TEST_F(PositionDistanceRuntimeTest, GroundReconfigureRetainsExistingCustom1PublishCadence) {
    start(TrackingBackend::PVA_SMC);
    enterFlightState(state_type::Custom1);
    request(event_type::LANDING_REQUESTED);
    ASSERT_EQ(state(), state_type::Landing);

    sensor_.local_z = sensor_.vrpn_z = sensor_.z = 0.0;
    sensor_.fcu_armed = false;
    for (int i = 0; i < 5; ++i) {
        sensor_.local_pos_stats.is_new = sensor_.vrpn_pose_stats.is_new = true;
        sensor_.state_stats.is_new = true;
        tick();
    }
    ASSERT_EQ(state(), state_type::SelfCheck);
    tick();
    ASSERT_EQ(state(), state_type::Ready);

    auto config = controller_->getConfig();
    config.tracking_backend = TrackingBackend::SMC;
    controller_->setConfig(config);
    enterFlightState(state_type::Custom1);
    tick();  // First waiting-reference setpoint.
    ASSERT_TRUE(std::any_of(controller_->flightEffects().begin(), controller_->flightEffects().end(),
                            [](const FlightEffect& effect) {
                                return std::holds_alternative<FlightSetpoint>(effect);
                            }));
    now_ += 0.01;
    tick();  // The previously configured 10 ms PVA cadence still applies.
    EXPECT_TRUE(std::any_of(controller_->flightEffects().begin(), controller_->flightEffects().end(),
                            [](const FlightEffect& effect) {
                                return std::holds_alternative<FlightSetpoint>(effect);
                            }));
}

TEST_F(PositionDistanceRuntimeTest, RealSetpointDeadlineDoesNotMoveOnAnEarlyTurn) {
    start();
    EXPECT_FALSE(controller_->update(now_));
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Takeoff, now_));
    now_ += 0.01;
    EXPECT_TRUE(controller_->update(now_));  // Continue the entered state once.
    ASSERT_EQ(state(), state_type::TakeoffInit);
    EXPECT_FALSE(controller_->update(now_));

    const auto due = controller_->nextWake();
    ASSERT_TRUE(due);
    EXPECT_NEAR(*due - now_, kLocalSetpointPublishInterval, 1e-9);
    controller_->update(std::nextafter(*due, -std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(std::any_of(controller_->flightEffects().begin(), controller_->flightEffects().end(),
                             [](const FlightEffect& effect) {
                                 return std::holds_alternative<FlightSetpoint>(effect);
                             }));
    ASSERT_TRUE(controller_->nextWake());
    EXPECT_DOUBLE_EQ(*controller_->nextWake(), *due);
    controller_->update(*due);
    EXPECT_TRUE(std::any_of(controller_->flightEffects().begin(), controller_->flightEffects().end(),
                            [](const FlightEffect& effect) {
                                return std::holds_alternative<FlightSetpoint>(effect);
                            }));
}

TEST_F(PositionDistanceRuntimeTest, ExistingTakeoffTimeoutRemainsStrictAtTheDeadline) {
    start();
    ASSERT_TRUE(controller_->postCommand(FlightCommand::Takeoff, 101.0));
    controller_->update(101.0);
    ASSERT_EQ(state(), state_type::TakeoffInit);
    controller_->update(111.0);
    ASSERT_EQ(state(), state_type::TakeoffInit);
    const auto due = controller_->nextWake();
    ASSERT_TRUE(due);
    EXPECT_GT(*due, 111.0);
    EXPECT_LT(*due, 111.000001);
    controller_->update(*due);
    EXPECT_EQ(state(), state_type::Landing);
}

TEST_F(PositionDistanceRuntimeTest, FreshImuObservationsDoNotDependOnEmptyTurns) {
    sensor_.local_pos_stats.is_active = false;
    controller_ = std::make_unique<DroneController>(sensor_);
    ControllerConfig config;
    config.tracking_backend = TrackingBackend::PX4_LOCAL;
    config.safety.acc_saturation_xy = 1.0;
    config.safety.acc_saturation_z = 1.0;
    controller_->setConfig(config);
    controller_->getSetpoint().ax = 2.0;

    controller_->onFreshImu();
    tick();
    EXPECT_EQ(state(), state_type::SelfCheck);
    sensor_.local_pos_stats.is_active = true;
    controller_->onFreshImu();
    tick();
    ASSERT_EQ(state(), state_type::Ready);
    controller_->onFreshImu();
    tick();
    EXPECT_EQ(state(), state_type::Landing);
    EXPECT_EQ(controller_->flightDiagnosticEvents(),
              (std::vector<uint32_t>{event_type::SAFE_CONTROL_SATURATION_XY}));
    tick();  // A wake without another observation must not replay the same sample.
    EXPECT_TRUE(controller_->flightDiagnosticEvents().empty());
}

TEST_F(PositionDistanceRuntimeTest, DistinctImuSamplesEachProduceTheirOwnSaturationFacts) {
    start();
    auto config = controller_->getConfig();
    config.safety.acc_saturation_xy = 1.0;
    config.safety.acc_saturation_z = 1.0;
    controller_->setConfig(config);
    controller_->onFreshImu();  // Original zero setpoint is not saturated.
    tick();
    EXPECT_EQ(state(), state_type::Ready);
    EXPECT_TRUE(controller_->flightDiagnosticEvents().empty());

    controller_->getSetpoint().ax = 2.0;
    controller_->getSetpoint().az = -2.0;
    controller_->onFreshImu();
    controller_->onFreshImu();  // Two distinct received samples in the same turn.
    tick();
    EXPECT_EQ(state(), state_type::Landing);
    EXPECT_EQ(controller_->flightDiagnosticEvents(),
              (std::vector<uint32_t>{event_type::SAFE_CONTROL_SATURATION_XY,
                                     event_type::SAFE_CONTROL_SATURATION_Z,
                                     event_type::SAFE_CONTROL_SATURATION_XY,
                                     event_type::SAFE_CONTROL_SATURATION_Z}));
    sensor_.imu_stats.is_new = true;  // No new ingress: flags cannot replay a sample.
    tick();
    EXPECT_TRUE(controller_->flightDiagnosticEvents().empty());
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
        for (const auto event : controller_->flightDiagnosticEvents()) {
            EXPECT_NE(event, event_type::LANDING_REQUESTED);
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
