#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/time.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>

#include "px4_multirotor_controller/common/core_log.h"
#include "px4_multirotor_controller/common/world_boundary.h"
#include "px4_multirotor_controller/driver/controller_config.h"
#include "px4_multirotor_controller/driver/pose_ingress.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {
namespace {

std::atomic<int> g_counted_error_logs{0};
void countingErrorLogSink(LogLevel level, const char*) {
    if (level == LogLevel::kError) {
        g_counted_error_logs.fetch_add(1, std::memory_order_relaxed);
    }
}

void pushLocal(SensorData& sd, double stamp_s, double x, double y, double z, double qx = 0.0,
               double qy = 0.0, double qz = 0.0, double qw = 1.0) {
    const uint64_t ns = static_cast<uint64_t>(std::llround(stamp_s * 1e9));
    ingestLocalPoseSample(sd, static_cast<uint32_t>(ns / 1000000000ULL),
                          static_cast<uint32_t>(ns % 1000000000ULL), stamp_s, x, y, z, qx, qy, qz,
                          qw);
    sd.local_pos_stats.is_new = true;
}

void pushVrpn(SensorData& sd, double stamp_s, double x, double y, double z, double qx = 0.0,
              double qy = 0.0, double qz = 0.0, double qw = 1.0) {
    const uint64_t ns = static_cast<uint64_t>(std::llround(stamp_s * 1e9));
    ingestVrpnPoseSample(sd, static_cast<uint32_t>(ns / 1000000000ULL),
                         static_cast<uint32_t>(ns % 1000000000ULL), stamp_s, x, y, z, qx, qy, qz,
                         qw);
    sd.vrpn_pose_stats.is_new = true;
}

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
        sensor_.z = 1.0;
        pushLocal(sensor_, 100.0, 0.0, 0.0, 1.0);
        pushVrpn(sensor_, 100.0, 0.0, 0.0, 1.0);
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
        now_ += 0.01;
        pushLocal(sensor_, now_, sensor_.local_x, sensor_.local_y, sensor_.local_z,
                  sensor_.local_qx, sensor_.local_qy, sensor_.local_qz, sensor_.local_qw);
        pushVrpn(sensor_, now_, sensor_.vrpn_x, sensor_.vrpn_y, sensor_.vrpn_z, sensor_.vrpn_qx,
                 sensor_.vrpn_qy, sensor_.vrpn_qz, sensor_.vrpn_qw);
        controller_->update(now_);
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

TEST_F(PositionDistanceRuntimeTest, ReadyAllowsTakeoffWithDistanceAndGroundFaultsReturnToSelfCheck) {
    start();
    ASSERT_EQ(state(), state_type::Ready);
    // Ground Ready must NOT block takeoff due to legacy 1.0 m distance refusal
    // and must NOT preempt to Landing on fault.
    sensor_.vrpn_x = 1.001;
    tick();
    ASSERT_EQ(state(), state_type::Ready);
    EXPECT_TRUE(controller_->getPositionDistance().exceeded);

    sensor_.vrpn_x = 0.0;
    tick();
    sensor_.state_stats.is_active = false;
    tick();
    EXPECT_EQ(state(), state_type::SelfCheck);

    sensor_.state_stats.is_active = true;
    tick();
    ASSERT_EQ(state(), state_type::Ready);
    const double qnan = std::numeric_limits<double>::quiet_NaN();
    ingestVrpnPoseSample(sensor_, 200, 0, now_ + 0.005, qnan, qnan, qnan, qnan, qnan, qnan, qnan);
    controller_->update(now_ += 0.01);
    EXPECT_EQ(state(), state_type::SelfCheck);
    tick();
    ASSERT_EQ(state(), state_type::Ready);

    // Takeoff is admitted from Ready once canonical & local poses are active/valid.
    request(event_type::TAKEOFF_REQUESTED);
    EXPECT_EQ(state(), state_type::TakeoffInit);
}

TEST_F(PositionDistanceRuntimeTest, GroundStatesNeverLandBecauseOfPositionDistance) {
    sensor_.state_stats.is_active = false;
    now_ += 0.005;
    pushVrpn(sensor_, now_, 2.0, 0.0, 1.0);
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
    for (int i = 0; i < 20; ++i) {
        tick();
        EXPECT_EQ(state(), state_type::Ready);
    }
}

TEST_F(PositionDistanceRuntimeTest, ReadyRequiresActiveValidLocalAndVrpnPoses) {
    sensor_.vrpn_pose_stats.is_active = false;
    start();
    EXPECT_EQ(state(), state_type::SelfCheck);
    request(event_type::TAKEOFF_REQUESTED);
    EXPECT_EQ(state(), state_type::SelfCheck);

    sensor_.vrpn_pose_stats.is_active = true;
    tick();
    ASSERT_EQ(state(), state_type::Ready);
    request(event_type::TAKEOFF_REQUESTED);
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
    pushLocal(sensor_, 101.0, sensor_.local_x, sensor_.local_y, sensor_.local_z);
    pushVrpn(sensor_, 101.0, sensor_.vrpn_x, sensor_.vrpn_y, sensor_.vrpn_z);
    controller_->update(101.0);
    ASSERT_EQ(state(), state_type::TakeoffInit);
    // Keep pose streams fresher than the 2.0 s stale deadline while TakeoffInit
    // runs toward its strict 10.0 s timeout at 111.0 s.
    for (double t = 102.5; t < 110.5; t += 1.5) {
        pushLocal(sensor_, t, sensor_.local_x, sensor_.local_y, sensor_.local_z);
        pushVrpn(sensor_, t, sensor_.vrpn_x, sensor_.vrpn_y, sensor_.vrpn_z);
        controller_->update(t);
        sensor_.local_pos_stats.is_new = sensor_.vrpn_pose_stats.is_new = false;
        ASSERT_EQ(state(), state_type::TakeoffInit);
    }
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
    config.takeoff_altitude = 1.0;
    config.safety.acc_saturation_xy = 1.0;
    config.safety.acc_saturation_z = 1.0;
    controller_->setConfig(config);
    controller_->getSetpoint().ax = 2.0;

    controller_->onFreshImu();
    tick();
    EXPECT_EQ(state(), state_type::SelfCheck);
    sensor_.local_pos_stats.is_active = true;
    tick();
    ASSERT_EQ(state(), state_type::Ready);
    enterFlightState(state_type::Hover);
    controller_->getSetpoint().ax = 2.0;
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
    enterFlightState(state_type::Hover);
    controller_->onFreshImu();  // Hover zero setpoint is not saturated.
    tick();
    EXPECT_EQ(state(), state_type::Hover);
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

TEST_F(PositionDistanceRuntimeTest, AllActivePhasesLandImmediatelyOnSingleInvalidSe3Pose) {
    for (const uint32_t target : {state_type::TakeoffInit, state_type::TakeoffOffboardRequest,
                                  state_type::TakeoffArmRequest, state_type::TakeoffAscending,
                                  state_type::Hover, state_type::Custom1}) {
        SensorData sd;
        sd.local_pos_stats.is_active = true;
        sd.local_velocity_stats.is_active = true;
        sd.imu_stats.is_active = true;
        sd.state_stats.is_active = true;
        sd.uav_state_estimate_stats.is_active = true;
        sd.vrpn_pose_stats.is_active = true;
        sd.uav_state_estimator_state =
            rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
        sd.fcu_connected = true;
        double t = 100.0;
        ingestLocalPoseSample(sd, 100, 0, t, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        ingestVrpnPoseSample(sd, 100, 0, t, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0);

        DroneController ctrl(sd);
        ControllerConfig cfg;
        cfg.tracking_backend =
            (target == state_type::Custom1) ? TrackingBackend::DFBC : TrackingBackend::PX4_LOCAL;
        cfg.takeoff_altitude = 1.0;
        ctrl.setConfig(cfg);

        auto step = [&](double dt = 0.01) {
            t += dt;
            sd.z = sd.local_z;
            ctrl.update(t);
            sd.local_pos_stats.is_new = false;
            sd.vrpn_pose_stats.is_new = false;
            sd.state_stats.is_new = false;
        };
        step();
        ASSERT_EQ(static_cast<uint32_t>(ctrl.flightState()), state_type::Ready);
        ASSERT_TRUE(ctrl.postCommand(FlightCommand::Takeoff, t));
        step();
        if (target != state_type::TakeoffInit) {
            sd.fcu_mode = "ALTCTL";
            sd.state_stats.is_new = true;
            step();
            sd.state_stats.is_new = true;
            step();
            if (target != state_type::TakeoffOffboardRequest) {
                sd.fcu_mode = "OFFBOARD";
                sd.state_stats.is_new = true;
                step();
                if (target != state_type::TakeoffArmRequest) {
                    sd.fcu_armed = true;
                    sd.state_stats.is_new = true;
                    step();
                    if (target == state_type::Hover || target == state_type::Custom1) {
                        for (int i = 0; i < 6; ++i) {
                            t += 0.02;
                            const uint64_t ns = static_cast<uint64_t>(std::llround(t * 1e9));
                            ingestLocalPoseSample(sd, static_cast<uint32_t>(ns / 1000000000ULL),
                                                  static_cast<uint32_t>(ns % 1000000000ULL), t, 1.0,
                                                  2.0, 1.0, 0.0, 0.0, 0.0, 1.0);
                            ingestVrpnPoseSample(sd, static_cast<uint32_t>(ns / 1000000000ULL),
                                                 static_cast<uint32_t>(ns % 1000000000ULL), t, 1.0,
                                                 2.0, 1.0, 0.0, 0.0, 0.0, 1.0);
                            sd.local_pos_stats.is_new = true;
                            sd.vrpn_pose_stats.is_new = true;
                            sd.uav_state_estimate_stats.is_new = true;
                            sd.z = sd.local_z;
                            ctrl.update(t);
                            sd.local_pos_stats.is_new = false;
                            sd.vrpn_pose_stats.is_new = false;
                            sd.uav_state_estimate_stats.is_new = false;
                            sd.state_stats.is_new = false;
                        }
                        if (target == state_type::Custom1) {
                            ASSERT_TRUE(ctrl.postCommand(FlightCommand::Track, t));
                            step();
                        }
                    }
                }
            }
        }
        ASSERT_EQ(static_cast<uint32_t>(ctrl.flightState()), target);

        const double qnan = std::numeric_limits<double>::quiet_NaN();
        const auto invalid_res =
            ingestVrpnPoseSample(sd, 101, 0, t + 0.01, qnan, qnan, qnan, qnan, qnan, qnan, qnan);
        EXPECT_EQ(invalid_res, PoseIngressResult::RejectedInvalid);
        EXPECT_TRUE(sd.vrpn_pose_invalid);
        EXPECT_DOUBLE_EQ(sd.vrpn_x, 1.0);
        EXPECT_DOUBLE_EQ(sd.vrpn_y, 2.0);
        EXPECT_DOUBLE_EQ(sd.vrpn_z, 1.0);
        step();
        EXPECT_EQ(static_cast<uint32_t>(ctrl.flightState()), state_type::Landing);
        EXPECT_TRUE(ctrl.getLastSafetyTrigger().triggered);
        EXPECT_EQ(ctrl.getLastSafetyTrigger().event_id, event_type::SAFE_VRPN_POSE_INVALID);
    }
}

TEST_F(PositionDistanceRuntimeTest, Aligned50MsDelayPairingAndUnpairableNonFault) {
    SensorData sd;
    sd.local_pos_stats.is_active = sd.local_velocity_stats.is_active = true;
    sd.imu_stats.is_active = sd.state_stats.is_active = sd.vrpn_pose_stats.is_active = true;
    sd.fcu_connected = true;
    double t = 100.0;
    ingestLocalPoseSample(sd, 100, 0, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    ingestVrpnPoseSample(sd, 100, 0, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    DroneController ctrl(sd);
    ControllerConfig cfg;
    cfg.tracking_backend = TrackingBackend::PX4_LOCAL;
    cfg.takeoff_altitude = 1.0;
    cfg.safety.localization_delay_s = 0.050;
    cfg.safety.max_aligned_position_error_m = 0.50;
    ctrl.setConfig(cfg);
    ctrl.update(t);
    ASSERT_EQ(ctrl.flightState(), FlightState::Ready);

    ASSERT_TRUE(ctrl.postCommand(FlightCommand::Takeoff, t += 0.01));
    ctrl.update(t);
    sd.fcu_mode = "ALTCTL";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_mode = "OFFBOARD";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_armed = true;
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    for (int i = 0; i < 6; ++i) {
        const uint32_t nsec = static_cast<uint32_t>((i + 1) * 50000000);
        ingestLocalPoseSample(sd, 100, nsec, t += 0.05, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        ingestVrpnPoseSample(sd, 100, nsec, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        sd.local_pos_stats.is_new = sd.vrpn_pose_stats.is_new = true;
        ctrl.update(t);
    }
    ASSERT_EQ(ctrl.flightState(), FlightState::Hover);

    // Move at 12 m/s only along local & vrpn consistently at t_vrpn = t_local + 50 ms.
    // Naive same-time diff = 12 * 0.05 = 0.60 m (> 0.50 m!), aligned 50 ms diff = 0.00 m.
    t = 101.0;
    ingestLocalPoseSample(sd, 101, 0, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    ingestLocalPoseSample(sd, 101, 40000000, t + 0.04, 0.48, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    ingestLocalPoseSample(sd, 101, 80000000, t + 0.08, 0.96, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    ingestVrpnPoseSample(sd, 101, 90000000, t + 0.09, 0.48, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    ctrl.update(t + 0.09);
    EXPECT_EQ(ctrl.flightState(), FlightState::Hover);
    ASSERT_TRUE(ctrl.getPositionDistance().available);
    EXPECT_NEAR(ctrl.getPositionDistance().metres, 0.0, 1e-6);
    EXPECT_FALSE(ctrl.getPositionDistance().exceeded);

    // Gap > 100 ms in local_history must mark available=false and NEVER fault.
    ingestLocalPoseSample(sd, 101, 250000000, t + 0.25, 0.96, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    ingestVrpnPoseSample(sd, 101, 200000000, t + 0.26, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    ctrl.update(t + 0.26);
    EXPECT_EQ(ctrl.flightState(), FlightState::Hover);
    EXPECT_FALSE(ctrl.getPositionDistance().available);

    // Duplicate/regressing stamp increments anomaly count, never faults & never refreshes rx.
    const double saved_vrpn_rx = *sd.last_valid_vrpn_rx_s;
    const auto dup = ingestVrpnPoseSample(sd, 101, 200000000, t + 0.50, 2.0, 0.0, 1.0, 0.0, 0.0,
                                          0.0, 1.0);
    EXPECT_EQ(dup, PoseIngressResult::AcceptedNonFreshStamp);
    EXPECT_DOUBLE_EQ(*sd.last_valid_vrpn_rx_s, saved_vrpn_rx);
    ctrl.update(t + 0.50);
    EXPECT_GE(ctrl.getMocapStampAnomalyCount(), 1U);
    EXPECT_EQ(ctrl.flightState(), FlightState::Hover);
}

TEST_F(PositionDistanceRuntimeTest, RealDtJumpToleranceS3DoubleCoverGrossBoundsAndWakeTimeout) {
    SensorData sd;
    sd.local_pos_stats.is_active = sd.local_velocity_stats.is_active = true;
    sd.imu_stats.is_active = sd.state_stats.is_active = sd.vrpn_pose_stats.is_active = true;
    sd.fcu_connected = true;
    double t = 100.0;
    ingestLocalPoseSample(sd, 100, 0, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    ingestVrpnPoseSample(sd, 100, 0, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    DroneController ctrl(sd);
    ControllerConfig cfg;
    cfg.tracking_backend = TrackingBackend::PX4_LOCAL;
    cfg.takeoff_altitude = 1.0;
    cfg.safety.max_aligned_position_error_m = 5.0;
    ctrl.setConfig(cfg);
    ctrl.update(t);
    ASSERT_TRUE(ctrl.postCommand(FlightCommand::Takeoff, t += 0.01));
    ctrl.update(t);
    sd.fcu_mode = "ALTCTL";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_mode = "OFFBOARD";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_armed = true;
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    for (int i = 0; i < 6; ++i) {
        pushLocal(sd, t += 0.01, 0.0, 0.0, 1.0);
        pushVrpn(sd, t, 0.0, 0.0, 1.0);
        ctrl.update(t);
    }
    ASSERT_EQ(ctrl.flightState(), FlightState::Hover);

    // 0.2 s gap at 6 m/s (1.2 m <= 1.7 m limit) and 50 deg (<= 75 deg limit): NO fault.
    const double a50 = 50.0 * M_PI / 180.0;
    ingestVrpnPoseSample(sd, 100, 200000000, t + 0.2, 1.2, 0.0, 1.0, 0.0, 0.0,
                         std::sin(a50 * 0.5), std::cos(a50 * 0.5));
    ctrl.update(t + 0.2);
    EXPECT_EQ(ctrl.flightState(), FlightState::Hover);

    // S3 double-cover: q -> -q has shortest angle 0 rad, NO jump fault.
    ingestVrpnPoseSample(sd, 100, 250000000, t + 0.25, 1.2, 0.0, 1.0, 0.0, 0.0,
                         -std::sin(a50 * 0.5), -std::cos(a50 * 0.5));
    ctrl.update(t + 0.25);
    EXPECT_EQ(ctrl.flightState(), FlightState::Hover);

    // 0.4 s gap at 6 m/s (2.4 m <= 2.9 m limit) and 100 deg total rotation (+50 deg <= 125 deg):
    const double a100 = 100.0 * M_PI / 180.0;
    ingestVrpnPoseSample(sd, 100, 650000000, t + 0.65, 3.6, 0.0, 1.0, 0.0, 0.0,
                         std::sin(a100 * 0.5), std::cos(a100 * 0.5));
    ctrl.update(t + 0.65);
    EXPECT_EQ(ctrl.flightState(), FlightState::Hover);

    // Single teleport jump > real-dt limit triggers immediate 1-sample Landing.
    ingestVrpnPoseSample(sd, 100, 750000000, t + 0.75, 6.0, 0.0, 1.0, 0.0, 0.0,
                         std::sin(a100 * 0.5), std::cos(a100 * 0.5));
    ctrl.update(t + 0.75);
    EXPECT_EQ(ctrl.flightState(), FlightState::Landing);
    EXPECT_EQ(ctrl.getLastSafetyTrigger().event_id, event_type::SAFE_VRPN_POSITION_JUMP);
}

TEST_F(PositionDistanceRuntimeTest, PerAxisJumpDiagonalVsAxisAndNormalizedQuaternionDot) {
    SensorData sd;
    sd.local_pos_stats.is_active = sd.local_velocity_stats.is_active = true;
    sd.imu_stats.is_active = sd.state_stats.is_active = sd.vrpn_pose_stats.is_active = true;
    sd.fcu_connected = true;
    double t = 100.0;
    pushLocal(sd, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    pushVrpn(sd, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    DroneController ctrl(sd);
    ControllerConfig cfg;
    cfg.tracking_backend = TrackingBackend::PX4_LOCAL;
    cfg.takeoff_altitude = 1.0;
    cfg.safety.max_aligned_position_error_m = 5.0;
    ctrl.setConfig(cfg);
    ctrl.update(t);
    ASSERT_TRUE(ctrl.postCommand(FlightCommand::Takeoff, t += 0.01));
    ctrl.update(t);
    sd.fcu_mode = "ALTCTL";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_mode = "OFFBOARD";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_armed = true;
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    pushLocal(sd, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    pushVrpn(sd, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    for (int i = 0; i < 6; ++i) {
        pushLocal(sd, t += 0.01, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
        pushVrpn(sd, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
        ctrl.update(t);
    }
    ASSERT_EQ(ctrl.flightState(), FlightState::Hover);

    // Repeated non-unit quaternion q=(0,0,0,0.96) at dt=0.004 s -> 0 rad angular jump (NO Land).
    t += 0.004;
    pushLocal(sd, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    pushVrpn(sd, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.96);
    ctrl.update(t);
    EXPECT_EQ(ctrl.flightState(), FlightState::Hover);

    // Diagonal dx=dy=dz=0.70 m in dt=0.033 s: hypot=1.212 > 1.198, but max_axis=0.70 <= 1.198!
    t += 0.033;
    pushLocal(sd, t, 0.70, 0.70, 1.70, 0.0, 0.0, 0.0, 0.96);
    pushVrpn(sd, t, 0.70, 0.70, 1.70, 0.0, 0.0, 0.0, 0.96);
    ctrl.update(t);
    EXPECT_EQ(ctrl.flightState(), FlightState::Hover);

    // Single-axis jump dx=1.25 m (> 1.198 m) at dt=0.033 s DOES trigger immediate Landing.
    t += 0.033;
    pushVrpn(sd, t, 1.95, 0.70, 1.70, 0.0, 0.0, 0.0, 0.96);
    ctrl.update(t);
    EXPECT_EQ(ctrl.flightState(), FlightState::Landing);
    EXPECT_EQ(ctrl.getLastSafetyTrigger().event_id, event_type::SAFE_VRPN_POSITION_JUMP);
}

TEST_F(PositionDistanceRuntimeTest, LocalJumpAndGrossBoundsFreezeDescentAndBlockTouchdown) {
    SensorData sd;
    sd.local_pos_stats.is_active = sd.local_velocity_stats.is_active = true;
    sd.imu_stats.is_active = sd.state_stats.is_active = sd.vrpn_pose_stats.is_active = true;
    sd.fcu_connected = true;
    double t = 100.0;
    pushLocal(sd, t, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0);
    pushVrpn(sd, t, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0);
    DroneController ctrl(sd);
    ControllerConfig cfg;
    cfg.tracking_backend = TrackingBackend::PX4_LOCAL;
    cfg.takeoff_altitude = 0.1;
    cfg.safety.max_aligned_position_error_m = 50.0;
    ctrl.setConfig(cfg);
    ctrl.update(t);
    ASSERT_TRUE(ctrl.postCommand(FlightCommand::Takeoff, t += 0.01));
    ctrl.update(t);
    sd.fcu_mode = "ALTCTL";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_mode = "OFFBOARD";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_armed = true;
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    for (int i = 0; i < 6; ++i) {
        pushLocal(sd, t += 0.01, 0.0, 0.0, 0.1);
        pushVrpn(sd, t, 0.0, 0.0, 0.1);
        ctrl.update(t);
    }
    ASSERT_EQ(ctrl.flightState(), FlightState::Hover);

    // Local position jump triggers SAFE_POSITION_JUMP, Landing, and faulted local descent (-0.2).
    pushLocal(sd, t += 0.02, 2.5, 0.0, 0.05);
    ctrl.update(t);
    EXPECT_EQ(ctrl.flightState(), FlightState::Landing);
    EXPECT_EQ(ctrl.getLastSafetyTrigger().event_id, event_type::SAFE_POSITION_JUMP);
    EXPECT_TRUE(ctrl.isLocalPositionFaulted());
    EXPECT_DOUBLE_EQ(ctrl.getSetpoint().vz, -0.2);

    // Even near ground (z=0.05, vz=0) with FCU armed=false, faulted local must NOT confirm disarm!
    sd.fcu_armed = false;
    for (int i = 0; i < 15; ++i) {
        sd.state_stats.is_new = true;
        pushLocal(sd, t += 0.05, 2.5, 0.0, 0.05);
        ctrl.update(t);
        EXPECT_EQ(ctrl.flightState(), FlightState::Landing);
        EXPECT_DOUBLE_EQ(ctrl.getSetpoint().vz, -0.2);
    }

    // Separate check for local gross bounds (x = 120.0 -> SAFE_LOCAL_GROSS_BOUNDS_VIOLATION).
    SensorData sd2;
    sd2.local_pos_stats.is_active = sd2.local_velocity_stats.is_active = true;
    sd2.imu_stats.is_active = sd2.state_stats.is_active = sd2.vrpn_pose_stats.is_active = true;
    sd2.fcu_connected = true;
    t = 200.0;
    pushLocal(sd2, t, 99.5, 0.0, 1.0);
    pushVrpn(sd2, t, 99.5, 0.0, 1.0);
    DroneController ctrl2(sd2);
    cfg.takeoff_altitude = 1.0;
    ctrl2.setConfig(cfg);
    ctrl2.update(t);
    ASSERT_TRUE(ctrl2.postCommand(FlightCommand::Takeoff, t += 0.01));
    ctrl2.update(t);
    pushLocal(sd2, t += 0.2, 100.2, 0.0, 1.0);
    ctrl2.update(t);
    EXPECT_EQ(ctrl2.flightState(), FlightState::Landing);
    EXPECT_EQ(ctrl2.getLastSafetyTrigger().event_id, event_type::SAFE_LOCAL_GROSS_BOUNDS_VIOLATION);
    EXPECT_TRUE(ctrl2.isLocalPositionFaulted());
}

TEST_F(PositionDistanceRuntimeTest, InvalidThenValidBurstCoalescingTripsSafetyAnd130HzInvalidBurstBoundsLogs) {
    SensorData sd;
    sd.local_pos_stats.is_active = sd.local_velocity_stats.is_active = true;
    sd.imu_stats.is_active = sd.state_stats.is_active = sd.vrpn_pose_stats.is_active = true;
    sd.fcu_connected = true;
    double t = 100.0;
    pushLocal(sd, t, 0.0, 0.0, 1.0);
    pushVrpn(sd, t, 0.0, 0.0, 1.0);

    DroneController ctrl(sd);
    ControllerConfig cfg;
    cfg.tracking_backend = TrackingBackend::PX4_LOCAL;
    cfg.takeoff_altitude = 1.0;
    ctrl.setConfig(cfg);
    ctrl.update(t);
    ASSERT_TRUE(ctrl.postCommand(FlightCommand::Takeoff, t += 0.01));
    ctrl.update(t);
    ASSERT_EQ(ctrl.flightState(), FlightState::TakeoffInit);

    // Coalesced burst: ONE invalid qNaN frame followed immediately by a valid frame BEFORE one update().
    const double qnan = std::numeric_limits<double>::quiet_NaN();
    ingestVrpnPoseSample(sd, 100, 15000000, t + 0.005, qnan, qnan, qnan, qnan, qnan, qnan, qnan);
    pushVrpn(sd, t + 0.008, 0.0, 0.0, 1.0);
    EXPECT_FALSE(sd.vrpn_pose_invalid);
    EXPECT_EQ(sd.vrpn_pose_invalid_count, 1U);
    ctrl.update(t + 0.01);
    EXPECT_EQ(ctrl.flightState(), FlightState::Landing);
    EXPECT_EQ(ctrl.getLastSafetyTrigger().event_id, event_type::SAFE_VRPN_POSE_INVALID);

    // Feed 200 consecutive invalid frames at 130 Hz in Landing: Landing holds idempotently
    // and emitted error log entries stay bounded (<= 2).
    g_counted_error_logs.store(0, std::memory_order_relaxed);
    setLogSink(&countingErrorLogSink);
    double burst_t = t + 0.01;
    for (int i = 0; i < 200; ++i) {
        burst_t += 1.0 / 130.0;
        const uint64_t ns = static_cast<uint64_t>(std::llround(burst_t * 1e9));
        ingestVrpnPoseSample(sd, static_cast<uint32_t>(ns / 1000000000ULL),
                             static_cast<uint32_t>(ns % 1000000000ULL), burst_t, qnan, qnan, qnan,
                             qnan, qnan, qnan, qnan);
        ctrl.update(burst_t);
        ASSERT_EQ(ctrl.flightState(), FlightState::Landing);
    }
    setLogSink(nullptr);
    EXPECT_LE(g_counted_error_logs.load(std::memory_order_relaxed), 2);
}

TEST_F(PositionDistanceRuntimeTest, ControlVelocityBoundsAndSilentTimeoutWakeAndRequireFlag) {
    // require_control_velocity_bounds=true without controlVelocityBounds must throw.
    EXPECT_THROW(readControllerConfig(ControllerParameters([](const std::string& key,
                                                              ControllerParameterValue& out) {
                     if (key == "world_boundary_json") {
                         out = std::string(
                             R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-5,"xMax":5,"yMin":-5,"yMax":5,"zMin":0,"zMax":3},"groundZ":0})");
                         return true;
                     }
                     if (key == "require_control_velocity_bounds") {
                         out = true;
                         return true;
                     }
                     return false;
                 })),
                 std::invalid_argument);

    auto wb = parseWorldBoundary(
        R"({"schemaVersion":1,"frameId":"world","unit":"m","controlBounds":{"xMin":-5,"xMax":5,"yMin":-5,"yMax":5,"zMin":0,"zMax":3},"groundZ":0,"controlVelocityBounds":{"vxMin":-2,"vxMax":2,"vyMin":-2,"vyMax":2,"vzMin":-1,"vzMax":1.5},"planningBounds":{"xMin":-4,"xMax":4,"yMin":-4,"yMax":4,"zMin":0.5,"zMax":2.5},"planningVelocityBounds":{"vxMin":-1,"vxMax":1,"vyMin":-1,"vyMax":1,"vzMin":-0.5,"vzMax":0.5}})");
    SensorData sd;
    sd.local_pos_stats.is_active = sd.local_velocity_stats.is_active = true;
    sd.imu_stats.is_active = sd.state_stats.is_active = sd.vrpn_pose_stats.is_active = true;
    sd.fcu_connected = true;
    double t = 100.0;
    ingestLocalPoseSample(sd, 100, 0, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    ingestVrpnPoseSample(sd, 100, 0, t, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    DroneController ctrl(sd);
    ControllerConfig cfg;
    cfg.tracking_backend = TrackingBackend::PX4_LOCAL;
    cfg.takeoff_altitude = 1.0;
    cfg.safety.world_boundary = wb;
    cfg.safety.require_control_velocity_bounds = true;
    ctrl.setConfig(cfg);
    ctrl.update(t);
    ASSERT_TRUE(ctrl.postCommand(FlightCommand::Takeoff, t += 0.01));
    ctrl.update(t);
    sd.fcu_mode = "ALTCTL";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_mode = "OFFBOARD";
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.fcu_armed = true;
    sd.state_stats.is_new = true;
    ctrl.update(t += 0.01);
    for (int i = 0; i < 6; ++i) {
        pushLocal(sd, t += 0.01, 0.0, 0.0, 1.0);
        pushVrpn(sd, t, 0.0, 0.0, 1.0);
        ctrl.update(t);
    }
    ASSERT_EQ(ctrl.flightState(), FlightState::Hover);

    // vx = 1.9, vy = 1.9 => speed = 2.687 > 2.0, but both axes inside [-2, 2]: MUST NOT fault!
    sd.local_vx = 1.9;
    sd.local_vy = 1.9;
    sd.local_velocity_stats.is_new = true;
    ctrl.update(t += 0.01);
    sd.local_velocity_stats.is_new = false;
    EXPECT_EQ(ctrl.flightState(), FlightState::Hover);

    // Silent pose stream stop wakes via nextWake() up to last_valid_vrpn_rx_s + 2.0 s (102.11) -> TimeoutVrpnPose -> Landing.
    for (int step = 0; step < 100 && ctrl.flightState() == FlightState::Hover; ++step) {
        const auto wake = ctrl.nextWake();
        ASSERT_TRUE(wake);
        t = *wake;
        if (!sd.last_valid_local_rx_s || (t - *sd.last_valid_local_rx_s) > 0.5) {
            pushLocal(sd, t, 0.0, 0.0, 1.0);
        }
        ctrl.update(t);
    }
    EXPECT_NEAR(t, 102.11, 1e-3);
    EXPECT_EQ(ctrl.flightState(), FlightState::Landing);
    EXPECT_EQ(ctrl.getLastSafetyTrigger().event_id, event_type::SAFE_TIMEOUT_VRPN_POSE);
}

}  // namespace
}  // namespace px4_multirotor_controller
