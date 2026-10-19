#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

#include "px4_multirotor_controller/common/state_estimate_status.h"
#include "px4_multirotor_controller/driver/trajectory_ingress.h"

namespace px4_multirotor_controller {
namespace {

class DfbcStreamReferenceTest : public ::testing::Test {
   protected:
    void SetUp() override {
        sensor_.local_pos_stats.is_active = sensor_.local_velocity_stats.is_active = true;
        sensor_.imu_stats.is_active = sensor_.state_stats.is_active = true;
        sensor_.uav_state_estimate_stats.is_active = sensor_.vrpn_pose_stats.is_active = true;
        sensor_.uav_state_estimator_state = state_estimate::STATE_RUNNING;
        sensor_.local_pos_stats.is_new = sensor_.vrpn_pose_stats.is_new = true;
        sensor_.fcu_connected = true;
        sensor_.fcu_mode = "OFFBOARD";
        sensor_.local_z = sensor_.vrpn_z = sensor_.z = 1.0;
        sensor_.hover_thrust_estimate_available = true;
        sensor_.hover_thrust_estimate = .4;
        config_.tracking_backend = TrackingBackend::DFBC;
        config_.dfbc.reference_source = DfbcReferenceSource::PLANNER_SETPOINT;
        config_.nmpc.hover_thrust_enabled = true;
        config_.nmpc.enable_timing_log = false;
        controller_ = std::make_unique<DroneController>(sensor_);
        controller_->setConfig(config_);
        tick();
    }

    void tick(double dt = .02) {
        now_ = Time(now_ + dt).toSec();
        sensor_.hover_thrust_estimate_stamp = now_;
        controller_->update(now_);
    }

    uint32_t state() {
        return controller_->getStateMachine().currentState(region_type::CONTROL);
    }

    void request(uint32_t event) {
        ASSERT_TRUE(
            controller_->getStateMachine()
                .postEvent(::state_machine::Event(event, ::state_machine::EventTimestamp{now_}))
                .ok());
        tick();
    }

    void enterCustom1() {
        ASSERT_EQ(state(), state_type::Ready);
        request(event_type::TAKEOFF_REQUESTED);
        ASSERT_EQ(state(), state_type::TakeoffInit);
        request(event_type::ALTCTL_READY);
        ASSERT_EQ(state(), state_type::TakeoffOffboardRequest);
        request(event_type::OFFBOARD_READY);
        ASSERT_EQ(state(), state_type::TakeoffArmRequest);
        sensor_.fcu_armed = true;
        request(event_type::ARM_READY);
        ASSERT_EQ(state(), state_type::TakeoffAscending);
        request(event_type::ALTITUDE_REACHED);
        ASSERT_EQ(state(), state_type::Hover);
        request(event_type::TRAJECTORY_TRACKING_REQUESTED);
        ASSERT_EQ(state(), state_type::Custom1);
    }

    PositionTargetIngress sample(double x = 0) {
        PositionTargetIngress input;
        input.position << x, 0, 1;
        input.header_stamp = input.receipt_time = Time(now_);
        return input;
    }

    bool emitted(uint32_t event) {
        for (const auto& output : controller_->getStateMachine().currentOutputEvents())
            if (output.id == event)
                return true;
        return false;
    }

    void send(double x = 0) {
        ASSERT_EQ(ingestPlannerSetpoint(*controller_, sample(x)), PlannerSetpointResult::kCached);
        tick();
    }

    ControllerConfig config_;
    SensorData sensor_;
    std::unique_ptr<DroneController> controller_;
    double now_{100.0};
};

TEST_F(DfbcStreamReferenceTest, OptInDoesNotChangeDefaultFiniteReferencePath) {
    config_.dfbc.reference_source = DfbcReferenceSource::TRAJECTORY_CACHE;
    controller_->setConfig(config_);
    EXPECT_EQ(ingestPlannerSetpoint(*controller_, sample()), PlannerSetpointResult::kNotUsed);
    enterCustom1();
    EXPECT_TRUE(emitted(output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION));
    EXPECT_FALSE(controller_->mpcTrajectoryBuffer().hasPending());
}

TEST_F(DfbcStreamReferenceTest, RequiresCompleteFiniteWorldPvaAndEffectiveStamp) {
    auto input = sample();
    input.header_stamp = Time{};
    EXPECT_NE(ingestPlannerSetpoint(*controller_, input), PlannerSetpointResult::kCached);
    input = sample();
    input.coordinate_frame = 0;
    EXPECT_NE(ingestPlannerSetpoint(*controller_, input), PlannerSetpointResult::kCached);
    input = sample();
    input.type_mask = kIgnoreAfxBit;
    EXPECT_NE(ingestPlannerSetpoint(*controller_, input), PlannerSetpointResult::kCached);
    input = sample();
    input.type_mask = kForceBit;
    EXPECT_NE(ingestPlannerSetpoint(*controller_, input), PlannerSetpointResult::kCached);
    input = sample();
    input.velocity.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_NE(ingestPlannerSetpoint(*controller_, input), PlannerSetpointResult::kCached);
    EXPECT_FALSE(controller_->mpcTrajectoryBuffer().hasPending());
    input = sample();
    EXPECT_EQ(ingestPlannerSetpoint(*controller_, input), PlannerSetpointResult::kCached);
}

TEST_F(DfbcStreamReferenceTest, DefaultFiniteTrajectoryKeepsOriginalOutputPeriod) {
    config_.dfbc.reference_source = DfbcReferenceSource::TRAJECTORY_CACHE;
    config_.nmpc.control_period = .02;
    controller_->setConfig(config_);
    enterCustom1();
    reference::AnalyticReference trajectory;
    trajectory.header.stamp = trajectory.start_time = Time(now_ + 1.0);
    trajectory.duration = 10.0;
    trajectory.analytic_type = reference::AnalyticReference::ANALYTIC_HOLD;
    trajectory.origin.position.z = 1.0;
    trajectory.origin.orientation.w = 1.0;
    ASSERT_TRUE(controller_->activeTrajectoryCache().updateAnalytic(trajectory, Time(now_)));
    unsigned outputs = 0;
    for (int i = 0; i < 100; ++i) {
        tick(.001);
        if (emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET))
            ++outputs;
    }
    EXPECT_GT(outputs, 0U);
    EXPECT_LE(outputs, 6U);
    EXPECT_EQ(state(), state_type::Custom1);
}

TEST_F(DfbcStreamReferenceTest, WaitsForDueAlignedSampleThenTracksContinuousEndpointAndNextGoal) {
    enterCustom1();
    EXPECT_FALSE(emitted(output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION));
    EXPECT_FALSE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));

    // A future segment remains pending; it cannot take over early.
    auto future = sample();
    future.header_stamp = Time(now_ + .1);
    ASSERT_EQ(ingestPlannerSetpoint(*controller_, future), PlannerSetpointResult::kCached);
    tick();
    EXPECT_FALSE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));
    EXPECT_TRUE(controller_->mpcTrajectoryBuffer().hasPending());
    tick(.09);
    EXPECT_TRUE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));

    // This is a continuous planner stream, not a finite sampled trajectory.
    // A stationary endpoint remains usable, and another goal needs no reactivation.
    for (int i = 0; i < 150; ++i) {
        send();
        ASSERT_EQ(state(), state_type::Custom1);
        ASSERT_TRUE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));
        EXPECT_TRUE(std::isfinite(controller_->getAttitudeRateTarget().thrust));
        EXPECT_GT(controller_->getAttitudeRateTarget().thrust, 0);
        EXPECT_FALSE(emitted(output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION));
    }
    send(.5);
    EXPECT_TRUE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));
    EXPECT_NE(controller_->getAttitudeRateTarget().body_rate_y, 0.0);
}

TEST_F(DfbcStreamReferenceTest, StaleStreamHoldsOnePoseAndFreshStreamResumes) {
    enterCustom1();
    send();
    ASSERT_TRUE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));
    sensor_.x = sensor_.local_x = sensor_.vrpn_x = .25;
    tick(.2);
    ASSERT_TRUE(emitted(output_event_type::PUBLISH_SETPOINT));
    EXPECT_FALSE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));
    EXPECT_DOUBLE_EQ(controller_->getSetpoint().x, .25);
    sensor_.x = sensor_.local_x = sensor_.vrpn_x = .4;
    tick();
    EXPECT_DOUBLE_EQ(controller_->getSetpoint().x, .25);
    send(.4);
    EXPECT_EQ(state(), state_type::Custom1);
    EXPECT_TRUE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));
}

TEST_F(DfbcStreamReferenceTest, MisalignedFirstReferenceKeepsHover) {
    enterCustom1();
    send(3.0);
    EXPECT_FALSE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));
    EXPECT_TRUE(emitted(output_event_type::PUBLISH_SETPOINT));
    EXPECT_DOUBLE_EQ(controller_->getSetpoint().x, 0);
    send();
    EXPECT_TRUE(emitted(output_event_type::PUBLISH_ATTITUDE_RATE_TARGET));
}

}  // namespace
}  // namespace px4_multirotor_controller
