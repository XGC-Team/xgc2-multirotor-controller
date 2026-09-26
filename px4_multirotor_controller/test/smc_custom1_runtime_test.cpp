#include <gtest/gtest.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <cmath>
#include <memory>

#include <state_machine/runtime/async_task_executor.hpp>

#include "px4_multirotor_controller/output/control_output_consumer.h"

#include <xgc2_math/control/smc_tracking_controller.hpp>

#include "px4_multirotor_controller/common/state_estimate_status.h"
#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/drone_controller.h"

namespace px4_multirotor_controller {
namespace {

struct PublishedCommand {
    bool attitude{false};
    bool position{false};
    AttitudeRateTarget attitude_cmd{};
    Setpoint position_cmd{};
};

// Same decision as ControlOutputConsumer and ctl-px4: an attitude event is a
// body-rate command only while the target is valid. A position event is the
// setpoint that replaces it.
PublishedCommand publishedCommand(DroneController& controller) {
    PublishedCommand out;
    for (const auto& event : controller.getStateMachine().currentOutputEvents()) {
        if (event.id == output_event_type::PUBLISH_ATTITUDE_RATE_TARGET) {
            out.attitude = true;
            out.attitude_cmd = controller.getAttitudeRateTarget();
        }
        if (event.id == output_event_type::PUBLISH_SETPOINT) {
            out.position = true;
            out.position_cmd = controller.getSetpoint();
        }
    }
    return out;
}

Eigen::Vector3d expectedSmcAcceleration(const ControllerConfig& config, const SensorData& sensor,
                                          const Eigen::Vector3d& reference_position,
                                          const Eigen::Vector3d& reference_velocity,
                                          const Eigen::Vector3d& reference_acceleration) {
    xgc2_math::control::SmcTrackingConfig smc_config;
    smc_config.k1 = config.smc.k1;
    smc_config.k2 = config.smc.k2;
    smc_config.boundary_layer = config.smc.boundary_layer;
    const Eigen::Vector3d measured(sensor.local_x, sensor.local_y, sensor.local_z);
    const Eigen::Vector3d velocity(sensor.local_vx, sensor.local_vy, sensor.local_vz);
    const auto smc = xgc2_math::control::computeSmcTracking(
        smc_config, measured, velocity, reference_position, reference_velocity,
        reference_acceleration);
    EXPECT_TRUE(smc.success);
    EXPECT_TRUE(smc.acceleration.isApprox(reference_acceleration + smc.feedback));
    return smc.acceleration;
}

void expectAccelerationSetpoint(const PublishedCommand& command, const Eigen::Vector3d& acceleration) {
    EXPECT_FALSE(command.attitude);
    ASSERT_TRUE(command.position);
    EXPECT_NEAR(command.position_cmd.ax, acceleration.x(), 1e-9);
    EXPECT_NEAR(command.position_cmd.ay, acceleration.y(), 1e-9);
    EXPECT_NEAR(command.position_cmd.az, acceleration.z(), 1e-9);
    EXPECT_EQ(command.position_cmd.type_mask, kSmcAccelerationTypeMask);
    EXPECT_EQ(command.position_cmd.type_mask & kForceBit, 0u);
    EXPECT_EQ(command.position_cmd.coordinate_frame, 1);
}

class SmcCustom1RuntimeTest : public ::testing::Test {
   protected:
    void SetUp() override {
        sensor_.local_pos_stats.is_active = true;
        sensor_.local_velocity_stats.is_active = true;
        sensor_.imu_stats.is_active = true;
        sensor_.state_stats.is_active = true;
        sensor_.uav_state_estimate_stats.is_active = true;
        sensor_.vrpn_pose_stats.is_active = true;
        sensor_.uav_state_estimator_state = state_estimate::STATE_RUNNING;
        sensor_.uav_state_estimator_flags = 0;
        sensor_.fcu_connected = true;
        sensor_.qw = 1.0;
        sensor_.z = sensor_.local_z = sensor_.vrpn_z = 1.0;
        sensor_.hover_thrust_estimate = 0.5;
        sensor_.hover_thrust_estimate_available = true;
        sensor_.local_pos_stats.is_new = true;
        sensor_.vrpn_pose_stats.is_new = true;
        sensor_.uav_state_estimate_stats.is_new = true;
    }

    void start(TrackingBackend backend, Px4LocalLiftMode lift = Px4LocalLiftMode::Legacy) {
        controller_ = std::make_unique<DroneController>(sensor_);
        ControllerConfig config;
        config.tracking_backend = backend;
        config.px4_local_lift = lift;
        config.nmpc.hover_thrust_enabled = true;
        config.planning_period = 0.1;
        controller_->setConfig(config);
        config_ = controller_->getConfig();
        tick();
    }

    void touch() {
        sensor_.hover_thrust_estimate_stamp = now_;
        sensor_.uav_state_estimate_stamp = now_;
    }

    void tick() {
        now_ += 0.01;
        touch();
        controller_->update(now_);
    }

    void at(double t) {
        now_ = t;
        touch();
        controller_->update(now_);
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

    void enterHover() {
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
    }

    void enterCustom1() {
        enterHover();
        request(event_type::TRAJECTORY_TRACKING_REQUESTED);
        ASSERT_EQ(state(), state_type::Custom1);
    }

    void cache(const Time& header, const Time& receipt, const Eigen::Vector3d& position,
               const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration,
               uint16_t type_mask = 0) {
        PositionTargetIngress ingress;
        ingress.position = position;
        ingress.velocity = velocity;
        ingress.acceleration = acceleration;
        ingress.header_stamp = header;
        ingress.receipt_time = receipt;
        ingress.coordinate_frame = 1;
        ingress.type_mask = type_mask;
        const auto traj =
            ingestPositionTarget(ingress, config_.tracking_backend, config_.px4_local_lift);
        ASSERT_TRUE(traj.is_valid);
        controller_->mpcTrajectoryBuffer().cachePending(traj);
    }

    SensorData sensor_;
    ControllerConfig config_;
    std::unique_ptr<DroneController> controller_;
    double now_{100.0};
};

TEST_F(SmcCustom1RuntimeTest, Custom1PublishesWorldAccelerationWithoutHoverThrust) {
    start(TrackingBackend::SMC);
    ControllerConfig config = controller_->getConfig();
    config.nmpc.hover_thrust_enabled = false;
    controller_->setConfig(config);
    config_ = controller_->getConfig();
    sensor_.hover_thrust_estimate_available = false;
    sensor_.uav_state_estimate_stats = {};
    sensor_.uav_state_estimator_state = 0;
    sensor_.uav_state_estimator_flags = 0xffffffffu;
    sensor_.x = 99.0;
    enterCustom1();
    sensor_.local_x = 0.2;
    const double t0 = controller_->getCurrentTime();
    cache(Time(t0), Time(t0 - 0.05), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero());
    at(t0 + 0.02);

    const auto expected = expectedSmcAcceleration(config_, sensor_, Eigen::Vector3d(0.0, 0.0, 1.0),
                                                  Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    const auto command = publishedCommand(*controller_);
    expectAccelerationSetpoint(command, expected);
    EXPECT_GT(std::abs(command.position_cmd.ax), 1e-3);
}

TEST_F(SmcCustom1RuntimeTest, FutureSegmentDoesNotReplaceTheActiveOne) {
    start(TrackingBackend::SMC);
    enterCustom1();
    const double t0 = controller_->getCurrentTime();
    cache(Time(t0), Time(t0 - 0.05), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero());
    at(t0 + 0.02);
    const auto on_first = publishedCommand(*controller_);
    const Eigen::Vector3d first_acceleration = expectedSmcAcceleration(
        config_, sensor_, Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());
    expectAccelerationSetpoint(on_first, first_acceleration);

    cache(Time(t0 + 0.08), Time(t0 + 0.02), Eigen::Vector3d(1.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero());
    at(t0 + 0.04);
    const auto still_first = publishedCommand(*controller_);
    expectAccelerationSetpoint(still_first, first_acceleration);

    at(t0 + 0.08);
    const auto on_second = publishedCommand(*controller_);
    const Eigen::Vector3d second_acceleration = expectedSmcAcceleration(
        config_, sensor_, Eigen::Vector3d(1.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());
    expectAccelerationSetpoint(on_second, second_acceleration);
    EXPECT_GT(std::abs(on_second.position_cmd.ax - still_first.position_cmd.ax), 1e-3);
}

TEST_F(SmcCustom1RuntimeTest, ExpiredSegmentReleasesAttitudeAndPublishesHover) {
    start(TrackingBackend::SMC);
    enterCustom1();
    const double t0 = controller_->getCurrentTime();
    cache(Time(t0), Time(t0 - 0.05), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.4, 0.0, 0.0),
          Eigen::Vector3d::Zero());
    at(t0 + 0.02);
    EXPECT_EQ(publishedCommand(*controller_).position_cmd.type_mask, kSmcAccelerationTypeMask);

    at(t0 + 2.0);
    const auto command = publishedCommand(*controller_);
    EXPECT_FALSE(command.attitude);
    ASSERT_TRUE(command.position);
    EXPECT_NEAR(command.position_cmd.x, sensor_.local_x, 1e-12);
    EXPECT_NEAR(command.position_cmd.y, sensor_.local_y, 1e-12);
    EXPECT_NEAR(command.position_cmd.z, sensor_.local_z, 1e-12);
    EXPECT_DOUBLE_EQ(command.position_cmd.vx, 0.0);
    EXPECT_DOUBLE_EQ(command.position_cmd.vy, 0.0);
    EXPECT_DOUBLE_EQ(command.position_cmd.vz, 0.0);
    EXPECT_GT(std::abs(command.position_cmd.x - 0.8), 0.1);

    const double frozen_x = command.position_cmd.x;
    const double frozen_y = command.position_cmd.y;
    const double frozen_z = command.position_cmd.z;
    sensor_.local_x += 0.35;
    sensor_.local_y -= 0.2;
    sensor_.local_z += 0.15;
    at(t0 + 2.05);
    const auto held = publishedCommand(*controller_);
    EXPECT_FALSE(held.attitude);
    ASSERT_TRUE(held.position);
    EXPECT_NEAR(held.position_cmd.x, frozen_x, 1e-12);
    EXPECT_NEAR(held.position_cmd.y, frozen_y, 1e-12);
    EXPECT_NEAR(held.position_cmd.z, frozen_z, 1e-12);
    EXPECT_GT(std::abs(held.position_cmd.x - sensor_.local_x), 0.2);
}

TEST_F(SmcCustom1RuntimeTest, LegacyStillLiftsFromReceipt) {
    start(TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::Legacy);
    enterHover();
    const double t0 = controller_->getCurrentTime();
    cache(Time(t0 + 50.0), Time(50.0), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d::Zero());
    request(event_type::TRAJECTORY_TRACKING_REQUESTED);
    ASSERT_EQ(state(), state_type::Custom1);
    tick();
    const auto command = publishedCommand(*controller_);
    ASSERT_TRUE(command.position);
    EXPECT_FALSE(command.attitude);
    const double tau = controller_->getCurrentTime() - 50.0;
    EXPECT_NEAR(command.position_cmd.x, tau, 1e-6);
    EXPECT_GT(command.position_cmd.x, 10.0);
}

TEST_F(SmcCustom1RuntimeTest, ZeroOrderHoldWaitsForHeaderAndDoesNotIntegrate) {
    start(TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::ZeroOrderHold);
    enterHover();
    const double t0 = controller_->getCurrentTime();
    cache(Time(t0 + 0.05), Time(50.0), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d::Zero());
    request(event_type::TRAJECTORY_TRACKING_REQUESTED);
    ASSERT_EQ(state(), state_type::Custom1);
    tick();
    const auto early = publishedCommand(*controller_);
    ASSERT_TRUE(early.position);
    EXPECT_NEAR(early.position_cmd.x, 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(early.position_cmd.vx, 0.0);

    at(t0 + 0.08);
    const auto held = publishedCommand(*controller_);
    ASSERT_TRUE(held.position);
    EXPECT_NEAR(held.position_cmd.x, 0.0, 1e-9);
    EXPECT_NEAR(held.position_cmd.vx, 1.0, 1e-9);
    EXPECT_NEAR(held.position_cmd.z, 1.0, 1e-9);
}

TEST(StageEffectiveTime, ZeroOrderHoldAndSmcShareTheWindowLegacyDoesNot) {
    PositionTargetIngress ingress;
    ingress.position = Eigen::Vector3d(0.0, 0.0, 1.0);
    ingress.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    ingress.header_stamp = Time(10.0);
    ingress.receipt_time = Time(9.5);

    const auto legacy =
        ingestPositionTarget(ingress, TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::Legacy);
    EXPECT_EQ(legacy.planning_time, Time(9.5));
    MpcTrajectoryBuffer legacy_buffer;
    legacy_buffer.cachePending(legacy);
    EXPECT_TRUE(promoteTrajectorySample(legacy_buffer, Time(9.7), TrackingBackend::PX4_LOCAL,
                                        Px4LocalLiftMode::Legacy));
    const auto legacy_lift =
        liftForBackend(legacy_buffer.active(), Time(9.7), 0.1, kDefaultPvaLocalTypeMask, false,
                       TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::Legacy);
    ASSERT_TRUE(legacy_lift.success);
    EXPECT_NEAR(legacy_lift.setpoint.x, 0.2, 1e-9);

    const auto held =
        ingestPositionTarget(ingress, TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::ZeroOrderHold);
    const auto smc = ingestPositionTarget(ingress, TrackingBackend::SMC, Px4LocalLiftMode::Legacy);
    EXPECT_EQ(held.planning_time, Time(10.0));
    EXPECT_EQ(smc.planning_time, Time(10.0));

    MpcTrajectoryBuffer hold_buffer;
    hold_buffer.cachePending(held);
    EXPECT_FALSE(promoteTrajectorySample(hold_buffer, Time(9.7), TrackingBackend::PX4_LOCAL,
                                         Px4LocalLiftMode::ZeroOrderHold));
    EXPECT_TRUE(promoteTrajectorySample(hold_buffer, Time(10.05), TrackingBackend::PX4_LOCAL,
                                        Px4LocalLiftMode::ZeroOrderHold));
    const auto hold_lift =
        liftForBackend(hold_buffer.active(), Time(10.05), 0.1, kDefaultPvaLocalTypeMask, false,
                       TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::ZeroOrderHold);
    ASSERT_TRUE(hold_lift.success);
    EXPECT_NEAR(hold_lift.setpoint.x, 0.0, 1e-12);
    EXPECT_NEAR(hold_lift.setpoint.vx, 1.0, 1e-12);

    const auto smc_lift = liftForBackend(smc, Time(10.05), 0.1, kDefaultPvaLocalTypeMask, false,
                                         TrackingBackend::SMC, Px4LocalLiftMode::Legacy);
    ASSERT_TRUE(smc_lift.success);
    EXPECT_NEAR(smc_lift.setpoint.x, 0.05, 1e-9);
    EXPECT_NEAR(smc_lift.setpoint.vx, 1.0, 1e-9);

    EXPECT_FALSE(liftForBackend(smc, Time(10.2), 0.1, kDefaultPvaLocalTypeMask, false,
                                TrackingBackend::SMC, Px4LocalLiftMode::Legacy)
                     .success);
    EXPECT_FALSE(liftForBackend(held, Time(10.2), 0.1, kDefaultPvaLocalTypeMask, false,
                                TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::ZeroOrderHold)
                     .success);
    EXPECT_TRUE(liftForBackend(legacy, Time(10.2), 0.1, kDefaultPvaLocalTypeMask, false,
                               TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::Legacy)
                    .success);
}

PositionTargetIngress finiteWorldTarget(uint16_t type_mask, uint8_t coordinate_frame) {
    PositionTargetIngress ingress;
    ingress.position = Eigen::Vector3d(1.0, 2.0, 1.0);
    ingress.velocity = Eigen::Vector3d(0.3, 0.0, 0.0);
    ingress.acceleration = Eigen::Vector3d(0.2, 0.0, 0.0);
    ingress.header_stamp = Time(10.0);
    ingress.receipt_time = Time(9.5);
    ingress.type_mask = type_mask;
    ingress.coordinate_frame = coordinate_frame;
    return ingress;
}

TEST(SmcPositionTarget, UsesOnlyAvailableWorldPva) {
    const auto ignored = finiteWorldTarget(kIgnorePxBit, 1);
    EXPECT_FALSE(ingestPositionTarget(ignored, TrackingBackend::SMC, Px4LocalLiftMode::Legacy).is_valid);
    EXPECT_TRUE(ingestPositionTarget(ignored, TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::ZeroOrderHold)
                    .is_valid);
    EXPECT_TRUE(
        ingestPositionTarget(ignored, TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::Legacy).is_valid);

    const auto missing_accel = finiteWorldTarget(kIgnoreAfxBit, 1);
    EXPECT_FALSE(
        ingestPositionTarget(missing_accel, TrackingBackend::SMC, Px4LocalLiftMode::Legacy).is_valid);

    const auto force = finiteWorldTarget(static_cast<uint16_t>(kDefaultPvaLocalTypeMask | kForceBit), 1);
    EXPECT_FALSE(ingestPositionTarget(force, TrackingBackend::SMC, Px4LocalLiftMode::Legacy).is_valid);

    for (const uint8_t frame : {uint8_t{7}, uint8_t{8}, uint8_t{9}}) {
        const auto off_world = finiteWorldTarget(kDefaultPvaLocalTypeMask, frame);
        EXPECT_FALSE(
            ingestPositionTarget(off_world, TrackingBackend::SMC, Px4LocalLiftMode::Legacy).is_valid)
            << static_cast<unsigned>(frame);
    }

    const auto unset_frame = finiteWorldTarget(kDefaultPvaLocalTypeMask, 0);
    const auto unset_ingested =
        ingestPositionTarget(unset_frame, TrackingBackend::SMC, Px4LocalLiftMode::Legacy);
    EXPECT_TRUE(unset_ingested.is_valid);
    EXPECT_EQ(unset_ingested.coordinate_frame, 1);

    const auto body = finiteWorldTarget(kDefaultPvaLocalTypeMask, 8);
    const auto legacy_body =
        ingestPositionTarget(body, TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::Legacy);
    EXPECT_TRUE(legacy_body.is_valid);
    EXPECT_EQ(legacy_body.coordinate_frame, 8);
    EXPECT_TRUE(liftForBackend(legacy_body, Time(9.7), 0.1, kDefaultPvaLocalTypeMask, false,
                               TrackingBackend::PX4_LOCAL, Px4LocalLiftMode::Legacy)
                    .success);

    MpcTrajectoryState partial;
    partial.position_k = Eigen::Vector3d(9.0, 8.0, 1.5);
    partial.velocity_k = Eigen::Vector3d(0.2, -0.1, 0.0);
    partial.acceleration_k = Eigen::Vector3d(3.0, 3.0, 3.0);
    partial.planning_time = Time(2.0);
    partial.type_mask = 3523;
    partial.coordinate_frame = 1;
    partial.is_valid = true;
    const Setpoint zeroed = liftWorldLocal(partial, Time(2.05), 3523, false);
    EXPECT_NEAR(zeroed.x, 0.0, 1e-12);
    EXPECT_FALSE(liftForBackend(partial, Time(2.05), 0.1, 3523, false, TrackingBackend::SMC,
                                Px4LocalLiftMode::Legacy)
                     .success);
    partial.type_mask = 0;
    EXPECT_FALSE(liftForBackend(partial, Time(2.05), 0.1, 3523, false, TrackingBackend::SMC,
                                Px4LocalLiftMode::Legacy)
                     .success);

    MpcTrajectoryState forced = partial;
    forced.type_mask = static_cast<uint16_t>(kDefaultPvaLocalTypeMask | kForceBit);
    forced.coordinate_frame = 1;
    forced.is_valid = true;
    EXPECT_FALSE(liftForBackend(forced, Time(2.05), 0.1, kDefaultPvaLocalTypeMask, false,
                                TrackingBackend::SMC, Px4LocalLiftMode::Legacy)
                     .success);
    forced.coordinate_frame = 8;
    forced.type_mask = kDefaultPvaLocalTypeMask;
    EXPECT_FALSE(liftForBackend(forced, Time(2.05), 0.1, kDefaultPvaLocalTypeMask, false,
                                TrackingBackend::SMC, Px4LocalLiftMode::Legacy)
                     .success);
}

TEST_F(SmcCustom1RuntimeTest, IgnoredAxesDoNotBecomeAZeroReference) {
    start(TrackingBackend::SMC);
    ControllerConfig config = controller_->getConfig();
    config.local_type_mask = 3523;
    controller_->setConfig(config);
    config_ = controller_->getConfig();
    enterCustom1();
    sensor_.local_x = 0.2;
    const double t0 = controller_->getCurrentTime();
    cache(Time(t0), Time(t0 - 0.05), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.2, -0.1, 0.0),
          Eigen::Vector3d(3.0, 3.0, 3.0));
    at(t0 + 0.02);
    EXPECT_FALSE(publishedCommand(*controller_).attitude);
}

TEST_F(SmcCustom1RuntimeTest, Stage4TenHertzPvaClosesThroughOnboardLiftAndSmc) {
    start(TrackingBackend::SMC);
    enterCustom1();
    sensor_.local_x = 0.2;
    const double t0 = controller_->getCurrentTime();
    const Eigen::Vector3d velocity(0.3, 0.0, 0.0);
    const Eigen::Vector3d acceleration(0.2, 0.0, 0.0);
    const auto segment = [&](int index, double when) {
        const Eigen::Vector3d position(0.4 * index, 0.0, 1.0);
        cache(Time(t0 + 0.1 * index), Time(when), position, velocity, acceleration,
              kDefaultPvaLocalTypeMask);
        at(t0 + 0.1 * index + 0.05);
        const double tau = 0.05;
        const Eigen::Vector3d reference_position =
            position + velocity * tau + 0.5 * acceleration * tau * tau;
        const Eigen::Vector3d reference_velocity = velocity + acceleration * tau;
        const auto lifted =
            liftForBackend(controller_->mpcTrajectoryBuffer().active(), Time(t0 + 0.1 * index + 0.05),
                           0.1, kDefaultPvaLocalTypeMask, false, TrackingBackend::SMC,
                           Px4LocalLiftMode::Legacy);
        ASSERT_TRUE(lifted.success);
        EXPECT_NEAR(lifted.setpoint.x, reference_position.x(), 1e-9);
        EXPECT_NEAR(lifted.setpoint.vx, reference_velocity.x(), 1e-9);
        EXPECT_NEAR(lifted.setpoint.ax, acceleration.x(), 1e-12);
        EXPECT_NEAR(lifted.setpoint.ay, 0.0, 1e-12);
        EXPECT_NEAR(lifted.setpoint.az, 0.0, 1e-12);
        const auto expected = expectedSmcAcceleration(config_, sensor_, reference_position,
                                                      reference_velocity, acceleration);
        expectAccelerationSetpoint(publishedCommand(*controller_), expected);
    };

    segment(0, t0 - 0.05);
    segment(1, t0 + 0.10);

    at(t0 + 0.21);
    const auto expired = publishedCommand(*controller_);
    EXPECT_FALSE(expired.attitude);
    ASSERT_TRUE(expired.position);
    EXPECT_NEAR(expired.position_cmd.x, sensor_.local_x, 1e-12);
    EXPECT_NEAR(expired.position_cmd.y, sensor_.y, 1e-12);
    EXPECT_NEAR(expired.position_cmd.z, sensor_.z, 1e-12);
    EXPECT_DOUBLE_EQ(expired.position_cmd.vx, 0.0);
    const double extrapolated = 0.4 + 0.3 * 0.11 + 0.5 * 0.2 * 0.11 * 0.11;
    EXPECT_GT(std::abs(expired.position_cmd.x - extrapolated), 0.05);

    segment(2, t0 + 0.21);
}

TEST_F(SmcCustom1RuntimeTest, ConsumerDropsBodyRateAndHoldsFrozenHover) {
    if (!ros::isInitialized()) {
        ros::init(ros::M_string(), "smc_release_consumer", ros::init_options::AnonymousName);
    }
    ASSERT_TRUE(ros::master::check()) << "ROS master is required to observe ControlOutputConsumer";
    ros::NodeHandle nh;
    mavros_msgs::PositionTarget received_setpoint;
    int setpoint_count = 0;
    int attitude_count = 0;
    ros::Subscriber setpoint_sub = nh.subscribe<mavros_msgs::PositionTarget>(
        "mavros/setpoint_raw/local", 10,
        [&](const mavros_msgs::PositionTarget::ConstPtr& msg) {
            received_setpoint = *msg;
            ++setpoint_count;
        });
    ros::Subscriber attitude_sub = nh.subscribe<mavros_msgs::AttitudeTarget>(
        "mavros/setpoint_raw/attitude", 10,
        [&](const mavros_msgs::AttitudeTarget::ConstPtr&) { ++attitude_count; });

    start(TrackingBackend::SMC);
    enterCustom1();
    const double t0 = controller_->getCurrentTime();
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> executor(nh);
    ControlOutputConsumer consumer(nh, executor, *controller_, 10);
    const auto connected = [&]() {
        ros::spinOnce();
        return setpoint_sub.getNumPublishers() > 0 && attitude_sub.getNumPublishers() > 0;
    };
    const ros::WallTime connect_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while (ros::WallTime::now() < connect_deadline && !connected()) {
        ros::WallDuration(0.01).sleep();
    }
    ASSERT_GT(setpoint_sub.getNumPublishers(), 0u);
    ASSERT_GT(attitude_sub.getNumPublishers(), 0u);

    sensor_.local_x = 0.2;
    cache(Time(t0), Time(t0 - 0.05), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero());
    at(t0 + 0.02);
    const auto tracking = publishedCommand(*controller_);
    const auto expected = expectedSmcAcceleration(config_, sensor_, Eigen::Vector3d(0.0, 0.0, 1.0),
                                                  Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    expectAccelerationSetpoint(tracking, expected);
    const auto publish_current_events = [&]() {
        for (const auto& event : controller_->getStateMachine().currentOutputEvents()) {
            if (event.id == output_event_type::PUBLISH_SETPOINT ||
                event.id == output_event_type::PUBLISH_ATTITUDE_RATE_TARGET) {
                EXPECT_TRUE(consumer.handle(event));
            }
        }
    };
    publish_current_events();
    const ros::WallTime accel_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while (ros::WallTime::now() < accel_deadline && setpoint_count < 1) {
        ros::spinOnce();
        ros::WallDuration(0.01).sleep();
    }
    ASSERT_GE(setpoint_count, 1);
    EXPECT_EQ(attitude_count, 0);
    EXPECT_EQ(received_setpoint.type_mask, kSmcAccelerationTypeMask);
    EXPECT_EQ(received_setpoint.coordinate_frame, 1);
    EXPECT_NEAR(received_setpoint.acceleration_or_force.x, expected.x(), 1e-9);
    EXPECT_NEAR(received_setpoint.acceleration_or_force.y, expected.y(), 1e-9);
    EXPECT_NEAR(received_setpoint.acceleration_or_force.z, expected.z(), 1e-9);

    at(t0 + 2.0);
    const auto released = publishedCommand(*controller_);
    ASSERT_TRUE(released.position);
    ASSERT_FALSE(released.attitude);
    const double frozen_x = released.position_cmd.x;
    const double frozen_y = released.position_cmd.y;
    const double frozen_z = released.position_cmd.z;

    const int before_hover = setpoint_count;
    publish_current_events();

    const ros::WallTime first_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while (ros::WallTime::now() < first_deadline && setpoint_count == before_hover) {
        ros::spinOnce();
        ros::WallDuration(0.01).sleep();
    }
    ASSERT_GT(setpoint_count, before_hover);
    EXPECT_EQ(attitude_count, 0);
    EXPECT_NEAR(received_setpoint.position.x, frozen_x, 1e-12);
    EXPECT_NEAR(received_setpoint.position.y, frozen_y, 1e-12);
    EXPECT_NEAR(received_setpoint.position.z, frozen_z, 1e-12);
    EXPECT_DOUBLE_EQ(received_setpoint.velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(received_setpoint.velocity.y, 0.0);
    EXPECT_DOUBLE_EQ(received_setpoint.velocity.z, 0.0);

    sensor_.local_x += 0.35;
    sensor_.local_y -= 0.2;
    sensor_.local_z += 0.15;
    const int setpoints_before_drift = setpoint_count;
    at(t0 + 2.05);
    publish_current_events();
    const ros::WallTime drift_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
    while (ros::WallTime::now() < drift_deadline && setpoint_count == setpoints_before_drift) {
        ros::spinOnce();
        ros::WallDuration(0.01).sleep();
    }
    ASSERT_GT(setpoint_count, setpoints_before_drift);
    EXPECT_EQ(attitude_count, 0);
    EXPECT_NEAR(received_setpoint.position.x, frozen_x, 1e-12);
    EXPECT_NEAR(received_setpoint.position.y, frozen_y, 1e-12);
    EXPECT_NEAR(received_setpoint.position.z, frozen_z, 1e-12);
    EXPECT_GT(std::abs(received_setpoint.position.x - sensor_.local_x), 0.2);
}

}  // namespace
}  // namespace px4_multirotor_controller
