#include <gtest/gtest.h>

#include <cmath>

#include "estimator_vrpn_px4_rotor_state/RigidStateEstimate.h"
#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"
#include "px4_multirotor_controller/tracking/dfbc_attitude_rate_strategy.h"
#include "px4_multirotor_controller/tracking/px4_local_raw_strategy.h"
#include "px4_multirotor_controller/uav/active_trajectory_cache.h"
#include "px4_multirotor_controller/uav/nmpc_result_buffer.h"
#include "xgc2_math/control.hpp"

namespace px4_multirotor_controller {
namespace {

multirotor_reference_trajectory::AnalyticReference makeAnalyticReference() {
    multirotor_reference_trajectory::AnalyticReference msg;
    msg.header.stamp = ros::Time(10.0);
    msg.request_id = 1U;
    msg.trajectory_id = 2U;
    msg.revision = 3U;
    msg.analytic_type = multirotor_reference_trajectory::AnalyticReference::ANALYTIC_HEIGHT_CIRCLE;
    msg.start_time = ros::Time(10.0);
    msg.duration = 60.0;
    msg.origin.position.z = 3.0;
    msg.origin.orientation.w = 1.0;
    msg.params = {3.0, 1.5, 3.0, 0.5, 0.5, 0.0, 0.0, 0.0};
    return msg;
}

multirotor_reference_trajectory::AnalyticReference makeAnalyticCurveReference(
    uint16_t analytic_type) {
    auto msg = makeAnalyticReference();
    msg.analytic_type = analytic_type;
    msg.duration = 6.0;
    msg.origin.position.x = 0.0;
    msg.origin.position.y = 0.0;
    msg.origin.position.z = 1.5;
    switch (analytic_type) {
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_LINE:
            msg.params = {1.0, 0.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            break;
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_LEMNISCATE:
            msg.params = {1.0, 0.7, 1.0};
            break;
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_HELIX_YZ:
            msg.params = {0.5, 0.6, 10.0};
            break;
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_HELIX_XY:
            msg.params = {0.5, 0.6, 10.0};
            break;
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_TORUS_KNOT:
            msg.params = {0.5, 0.25};
            break;
        default:
            break;
    }
    return msg;
}

multirotor_reference_trajectory::SampledReference makeSampledReference() {
    multirotor_reference_trajectory::SampledReference msg;
    msg.header.stamp = ros::Time(20.0);
    msg.trajectory_id = 4U;
    msg.revision = 5U;
    msg.start_time = ros::Time(20.0);
    msg.sample_dt = 0.1;
    for (int i = 0; i < 3; ++i) {
        multirotor_reference_trajectory::FlatReferencePoint point;
        point.t_from_start = 0.1 * static_cast<double>(i);
        point.position.x = point.t_from_start;
        point.position.z = 3.0;
        point.velocity.x = 1.0;
        point.acceleration.z = 0.0;
        point.yaw = 0.0;
        msg.points.push_back(point);
    }
    return msg;
}

xgc2_math::trajectory::FlatOutput3 makeHoverFlatReference(double yaw = 0.0) {
    xgc2_math::trajectory::FlatOutput3 flat;
    flat.position = Eigen::Vector3d(0.0, 0.0, 3.0);
    flat.velocity.setZero();
    flat.acceleration.setZero();
    flat.jerk.setZero();
    flat.snap.setZero();
    flat.yaw = yaw;
    return flat;
}

SensorData makeDfbcSensor() {
    SensorData sensor;
    sensor.z = 3.0;
    sensor.qw = 1.0;
    sensor.uav_state_estimate_stats.is_active = true;
    sensor.uav_state_estimator_state =
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::STATE_RUNNING;
    sensor.hover_thrust_estimate = 0.3;
    sensor.hover_thrust_estimate_stamp = 10.0;
    sensor.hover_thrust_estimate_available = true;
    return sensor;
}

UavReferencePoint makeDfbcReference() {
    UavReferencePoint reference;
    reference.position = Eigen::Vector3d(0.0, 0.0, 3.0);
    reference.velocity.setZero();
    reference.acceleration.setZero();
    reference.jerk.setZero();
    reference.snap.setZero();
    reference.yaw = 0.0;
    return reference;
}

TEST(ActiveTrajectoryCache, AnalyticReferenceSamplesAndBuildsHorizon) {
    ActiveTrajectoryCache cache;
    ASSERT_TRUE(cache.updateAnalytic(makeAnalyticReference(), ros::Time(10.0)));
    EXPECT_EQ(cache.trajectoryId(), 2U);
    EXPECT_EQ(cache.revision(), 3U);

    UavReferencePoint sample;
    ASSERT_TRUE(cache.sample(ros::Time(10.5), 1.0, sample));
    EXPECT_TRUE(sample.position.array().isFinite().all());
    EXPECT_TRUE(sample.snap.array().isFinite().all());
    EXPECT_NEAR(sample.yaw, 0.0, 1e-12);
    EXPECT_NEAR(sample.yaw_rate, 0.0, 1e-12);
    EXPECT_NEAR(sample.yaw_accel, 0.0, 1e-12);

    std::vector<Se3Reference> horizon;
    ASSERT_TRUE(cache.sampleHorizon(ros::Time(10.0), 0.1, 10, 1.0, 9.8066, horizon));
    EXPECT_EQ(horizon.size(), 12U);
    EXPECT_TRUE(control::packState(horizon.front().state).array().isFinite().all());
    EXPECT_TRUE(control::packControl(horizon.front().control).array().isFinite().all());
}

TEST(ActiveTrajectoryCache, AnalyticCurveReferencesSampleAndBuildHorizons) {
    const uint16_t analytic_types[] = {
        multirotor_reference_trajectory::AnalyticReference::ANALYTIC_LINE,
        multirotor_reference_trajectory::AnalyticReference::ANALYTIC_LEMNISCATE,
        multirotor_reference_trajectory::AnalyticReference::ANALYTIC_HELIX_YZ,
        multirotor_reference_trajectory::AnalyticReference::ANALYTIC_HELIX_XY,
        multirotor_reference_trajectory::AnalyticReference::ANALYTIC_TORUS_KNOT,
    };

    for (const auto analytic_type : analytic_types) {
        ActiveTrajectoryCache cache;
        ASSERT_TRUE(
            cache.updateAnalytic(makeAnalyticCurveReference(analytic_type), ros::Time(10.0)))
            << "analytic_type=" << analytic_type;

        UavReferencePoint sample;
        ASSERT_TRUE(cache.sample(ros::Time(10.5), 1.0, sample))
            << "analytic_type=" << analytic_type;
        EXPECT_TRUE(sample.position.array().isFinite().all()) << "analytic_type=" << analytic_type;
        EXPECT_TRUE(sample.snap.array().isFinite().all()) << "analytic_type=" << analytic_type;
        EXPECT_NEAR(sample.yaw, 0.0, 1e-12) << "analytic_type=" << analytic_type;
        EXPECT_NEAR(sample.yaw_rate, 0.0, 1e-12) << "analytic_type=" << analytic_type;
        EXPECT_NEAR(sample.yaw_accel, 0.0, 1e-12) << "analytic_type=" << analytic_type;

        std::vector<Se3Reference> horizon;
        ASSERT_TRUE(cache.sampleHorizon(ros::Time(10.0), 0.1, 10, 1.0, 9.8066, horizon))
            << "analytic_type=" << analytic_type;
        EXPECT_EQ(horizon.size(), 12U) << "analytic_type=" << analytic_type;
        EXPECT_TRUE(control::packState(horizon.front().state).array().isFinite().all())
            << "analytic_type=" << analytic_type;
        EXPECT_TRUE(control::packControl(horizon.front().control).array().isFinite().all())
            << "analytic_type=" << analytic_type;
    }
}

TEST(ActiveTrajectoryCache, ReportsFiniteReferenceEndBeforeHorizonSamplingFails) {
    ActiveTrajectoryCache cache;
    ASSERT_TRUE(cache.updateAnalytic(makeAnalyticReference(), ros::Time(10.0)));

    double remaining = 0.0;
    std::vector<Se3Reference> horizon;
    ASSERT_TRUE(cache.finiteTimeRemaining(ros::Time(68.8), 100.0, remaining));
    EXPECT_NEAR(remaining, 1.2, 1e-9);
    EXPECT_TRUE(cache.sampleHorizon(ros::Time(68.8), 0.1, 10, 100.0, 9.8066, horizon));

    ASSERT_TRUE(cache.finiteTimeRemaining(ros::Time(68.95), 100.0, remaining));
    EXPECT_NEAR(remaining, 1.05, 1e-9);
    EXPECT_FALSE(cache.sampleHorizon(ros::Time(68.95), 0.1, 10, 100.0, 9.8066, horizon));
}

TEST(ActiveTrajectoryCache, SampledReferenceRequiresFiniteHighOrderSamples) {
    ActiveTrajectoryCache cache;
    ASSERT_TRUE(cache.updateSampled(makeSampledReference(), ros::Time(20.0)));
    UavReferencePoint sample;
    ASSERT_TRUE(cache.sample(ros::Time(20.05), 1.0, sample));
    EXPECT_NEAR(sample.position.x(), 0.05, 1e-9);
}

TEST(NmpcResultBuffer, KeepsNewestSequence) {
    NmpcResultBuffer buffer;
    NmpcSolveResult newer;
    newer.sequence = 2;
    newer.success = true;
    newer.stamp = ros::Time(1.0);
    buffer.store(newer);

    NmpcSolveResult older;
    older.sequence = 1;
    older.success = false;
    older.stamp = ros::Time(2.0);
    buffer.store(older);

    NmpcSolveResult output;
    ASSERT_TRUE(buffer.consumeNewerThan(0, output));
    EXPECT_EQ(output.sequence, 2U);
    EXPECT_TRUE(output.success);
    EXPECT_TRUE(buffer.hasFreshSuccess(ros::Time(1.05), 0.1));
    EXPECT_FALSE(buffer.hasFreshSuccess(ros::Time(1.2), 0.1));
}

TEST(SensorChecks, StateEstimateGateIsOnlyControlStateSource) {
    SensorData sensor;
    sensor.uav_state_estimate_stats.is_active = true;
    sensor.uav_state_estimator_state =
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::STATE_RUNNING;
    sensor.uav_state_estimator_flags = 0u;
    EXPECT_TRUE(sensor_checks::isStateEstimateUsableForControl(sensor));

    sensor.uav_state_estimator_state =
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::STATE_COASTING;
    EXPECT_TRUE(sensor_checks::isStateEstimateUsableForControl(sensor));

    sensor.uav_state_estimator_flags =
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_FAULT;
    EXPECT_FALSE(sensor_checks::isStateEstimateUsableForControl(sensor));

    sensor.uav_state_estimator_flags =
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_FILTER_DEGRADED |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_VRPN_SUSPECTED |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_POSE_TIME_ALIGNMENT_REJECTED |
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_INNOVATION_REJECTED;
    EXPECT_TRUE(sensor_checks::isStateEstimateUsableForControl(sensor));

    sensor.uav_state_estimator_flags =
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_VRPN_FAULT;
    EXPECT_FALSE(sensor_checks::isStateEstimateUsableForControl(sensor));

    sensor.uav_state_estimator_flags =
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::FLAG_FILTER_IMU_ONLY;
    EXPECT_FALSE(sensor_checks::isStateEstimateUsableForControl(sensor));

    sensor.uav_state_estimator_flags = 0u;
    sensor.uav_state_estimator_state =
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::STATE_SELF_CHECK;
    EXPECT_FALSE(sensor_checks::isStateEstimateUsableForControl(sensor));
}

TEST(DfbcGeometricController, HoverReferenceOutputsGravityThrust) {
    xgc2_math::control::DfbcGeometricController controller;
    xgc2_math::control::DfbcGeometricInput input;
    input.current.position = Eigen::Vector3d(0.0, 0.0, 3.0);
    input.current.attitude = Eigen::Quaterniond::Identity();
    input.reference = makeHoverFlatReference();

    const auto output = controller.compute(input);
    ASSERT_TRUE(output.success);
    EXPECT_NEAR(output.specific_thrust, 9.8066, 1e-6);
    EXPECT_NEAR(output.thrust_direction_error.norm(), 0.0, 1e-12);
}

TEST(DfbcGeometricController, PureYawDifferenceDoesNotCreateTiltError) {
    xgc2_math::control::DfbcGeometricConfig config;
    config.enable_yaw_control = true;
    xgc2_math::control::DfbcGeometricController controller(config);
    xgc2_math::control::DfbcGeometricInput input;
    input.current.position = Eigen::Vector3d(0.0, 0.0, 3.0);
    input.current.attitude = Eigen::Quaterniond::Identity();
    input.reference = makeHoverFlatReference(M_PI_2);

    const auto output = controller.compute(input);
    ASSERT_TRUE(output.success);
    EXPECT_NEAR(output.thrust_direction_error.norm(), 0.0, 1e-12);
    EXPECT_GT(output.body_rate_command.z(), 0.0);
}

TEST(DfbcGeometricController, PositionErrorCommandsTiltTowardReference) {
    xgc2_math::control::DfbcGeometricController controller;
    xgc2_math::control::DfbcGeometricInput input;
    input.current.position = Eigen::Vector3d::Zero();
    input.current.attitude = Eigen::Quaterniond::Identity();
    input.reference = makeHoverFlatReference();
    input.reference.position.x() = 1.0;

    const auto output = controller.compute(input);
    ASSERT_TRUE(output.success);
    EXPECT_GT(output.thrust_direction_error.y(), 0.0);
    EXPECT_GT(output.body_rate_command.y(), 0.0);
}

TEST(DfbcGeometricController, AccelerationCorrectionAddsFilteredAccelerationDeficit) {
    xgc2_math::control::DfbcGeometricConfig config;
    config.acceleration_correction_enabled = true;
    config.acceleration_correction_gain = Eigen::Vector3d(0.5, 0.0, 0.0);
    config.acceleration_correction_limit = Eigen::Vector3d(2.0, 0.0, 0.0);
    config.acceleration_correction_filter_tau = 0.0;
    xgc2_math::control::DfbcGeometricController controller(config);
    xgc2_math::control::DfbcGeometricInput input;
    input.current.attitude = Eigen::Quaterniond::Identity();
    input.reference = makeHoverFlatReference();
    input.reference.acceleration.x() = 1.0;
    input.has_measured_acceleration = true;
    input.measured_acceleration = Eigen::Vector3d::Zero();
    input.dt = 0.01;

    const auto output = controller.compute(input);
    ASSERT_TRUE(output.success);
    ASSERT_TRUE(output.acceleration_correction_active);
    EXPECT_NEAR(output.nominal_acceleration.x(), 1.0, 1e-12);
    EXPECT_NEAR(output.acceleration_error.x(), 1.0, 1e-12);
    EXPECT_NEAR(output.acceleration_correction.x(), 0.5, 1e-12);
    EXPECT_NEAR(output.corrected_acceleration.x(), 1.5, 1e-12);
}

TEST(DfbcAttitudeRateStrategy, DisablesYawAndClampsBodyRates) {
    ControllerConfig config;
    config.tracking_backend = TrackingBackend::DFBC_ATTITUDE_RATE;
    config.enable_yaw_control = false;
    config.nmpc.hover_thrust_enabled = true;
    config.nmpc.max_roll_pitch_body_rate = 0.2;
    config.nmpc.max_yaw_body_rate = 0.1;
    config.dfbc.tilt_gain = 20.0;

    DfbcAttitudeRateStrategy strategy;
    strategy.configure(config);
    const ros::Time now(10.0);
    ASSERT_TRUE(strategy.enter(makeDfbcSensor(), now));

    TrackingStrategyInput input;
    input.sensor = makeDfbcSensor();
    input.now = now;
    input.reference = makeDfbcReference();
    input.reference.position.x() = 10.0;
    input.reference.yaw = M_PI_2;

    TrackingStrategyResult result;
    ASSERT_TRUE(strategy.update(input, result)) << result.message;
    EXPECT_NEAR(result.attitude_rate_target.body_rate_z, 0.0, 1e-12);
    EXPECT_LE(std::abs(result.attitude_rate_target.body_rate_x), 0.2 + 1e-12);
    EXPECT_LE(std::abs(result.attitude_rate_target.body_rate_y), 0.2 + 1e-12);
    EXPECT_GE(result.attitude_rate_target.thrust, config.nmpc.normalized_thrust_min);
    EXPECT_LE(result.attitude_rate_target.thrust, config.nmpc.normalized_thrust_max);
}

TEST(Px4LocalRawStrategy, OutputsPositionVelocityAccelerationAndIgnoresYaw) {
    ControllerConfig config;
    config.tracking_backend = TrackingBackend::PX4_LOCAL_RAW;
    config.nmpc.control_period = 0.01;

    Px4LocalRawStrategy strategy;
    strategy.configure(config);
    ASSERT_TRUE(strategy.enter(makeDfbcSensor(), ros::Time(10.0)));

    TrackingStrategyInput input;
    input.sensor = makeDfbcSensor();
    input.now = ros::Time(10.0);
    input.reference = makeDfbcReference();
    input.reference.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    input.reference.velocity = Eigen::Vector3d(0.4, 0.5, 0.6);
    input.reference.acceleration = Eigen::Vector3d(0.7, 0.8, 0.9);
    input.reference.yaw = 1.2;
    input.reference.yaw_rate = 0.3;

    TrackingStrategyResult result;
    ASSERT_TRUE(strategy.update(input, result)) << result.message;
    EXPECT_EQ(result.output_kind, TrackingStrategyResult::OutputKind::LocalSetpoint);
    EXPECT_NEAR(result.local_setpoint.x, 1.0, 1e-12);
    EXPECT_NEAR(result.local_setpoint.y, 2.0, 1e-12);
    EXPECT_NEAR(result.local_setpoint.z, 3.0, 1e-12);
    EXPECT_NEAR(result.local_setpoint.vx, 0.4, 1e-12);
    EXPECT_NEAR(result.local_setpoint.vy, 0.5, 1e-12);
    EXPECT_NEAR(result.local_setpoint.vz, 0.6, 1e-12);
    EXPECT_NEAR(result.local_setpoint.ax, 0.7, 1e-12);
    EXPECT_NEAR(result.local_setpoint.ay, 0.8, 1e-12);
    EXPECT_NEAR(result.local_setpoint.az, 0.9, 1e-12);
    EXPECT_EQ(result.local_setpoint.type_mask, static_cast<uint16_t>((1U << 10U) | (1U << 11U)));
    EXPECT_EQ(result.local_setpoint.coordinate_frame, 1);
}

TEST(UavNmpcSolver, SolvesHoverEquilibrium) {
    ros::Time::init();
    UavNmpcSolver solver;
    ASSERT_TRUE(solver.initialize());

    Se3Reference hover;
    hover.state.position.z() = 3.0;
    hover.state.attitude = Eigen::Quaterniond::Identity();
    hover.control.body_z_specific_force = 9.8066;
    hover.control.angular_acceleration.setZero();

    Se3StateVector x0 = control::packState(hover.state);
    std::vector<Se3Reference> references(static_cast<size_t>(UavNmpcSolver::horizonSteps() + 2),
                                         hover);
    EXPECT_TRUE(solver.solve(x0, hover.control.body_z_specific_force,
                             hover.control.body_z_specific_force, Eigen::Vector3d::Zero(),
                             references))
        << "status=" << solver.status();
    EXPECT_NEAR(solver.optimalControl()(0), 9.8066, 1e-3);
    EXPECT_NEAR(solver.predictedBodyRate().norm(), 0.0, 1e-3);
}

TEST(UavNmpcSolver, AppliesRuntimeSpecificThrustBounds) {
    ros::Time::init();
    UavNmpcSolver solver;
    ASSERT_TRUE(solver.configureInputBounds(8.0, 12.0, 3.0, 1.0, 15.0, 2.0));
    ASSERT_TRUE(solver.initialize());

    Se3Reference hover;
    hover.state.position.z() = 3.0;
    hover.state.attitude = Eigen::Quaterniond::Identity();
    hover.control.body_z_specific_force = 9.8066;
    hover.control.angular_acceleration.setZero();

    const Se3StateVector x0 = control::packState(hover.state);
    std::vector<Se3Reference> references(static_cast<size_t>(UavNmpcSolver::horizonSteps() + 2),
                                         hover);
    ASSERT_TRUE(solver.solve(x0, hover.control.body_z_specific_force,
                             hover.control.body_z_specific_force, Eigen::Vector3d::Zero(),
                             references))
        << "status=" << solver.status();
    EXPECT_GE(solver.optimalControl()(0), 8.0 - 1e-5);
    EXPECT_LE(solver.optimalControl()(0), 12.0 + 1e-5);
}

TEST(UavNmpcBridge, IntegratesAngularAccelerationOverControlPeriod) {
    const Eigen::Vector3d current_rate(0.10, -0.20, 0.05);
    const Eigen::Vector3d alpha(2.0, -1.0, 0.5);
    const Eigen::Vector3d command =
        bodyRateCommandFromAngularAcceleration(current_rate, alpha, 0.01, 3.5, 0.9);

    EXPECT_NEAR(command.x(), 0.12, 1e-12);
    EXPECT_NEAR(command.y(), -0.21, 1e-12);
    EXPECT_NEAR(command.z(), 0.055, 1e-12);
}

TEST(UavNmpcBridge, ClampsBodyRateCommand) {
    const Eigen::Vector3d command = bodyRateCommandFromAngularAcceleration(
        Eigen::Vector3d(3.4, -3.4, 0.85), Eigen::Vector3d(20.0, -20.0, 10.0), 0.01, 3.5, 0.9);

    EXPECT_NEAR(command.x(), 3.5, 1e-12);
    EXPECT_NEAR(command.y(), -3.5, 1e-12);
    EXPECT_NEAR(command.z(), 0.9, 1e-12);
}

TEST(UavNmpcBridge, RecoversBodyRateFromReferenceAttitudeDelta) {
    const double dt = 0.02;
    const double yaw_rate = 0.7;
    const Eigen::Quaterniond q0 = yawToQuaternion(0.3);
    const Eigen::Quaterniond q1 = yawToQuaternion(0.3 + yaw_rate * dt);

    const Eigen::Vector3d body_rate = bodyRateFromRotationDelta(q0, q1, dt);
    EXPECT_NEAR(body_rate.x(), 0.0, 1e-12);
    EXPECT_NEAR(body_rate.y(), 0.0, 1e-12);
    EXPECT_NEAR(body_rate.z(), yaw_rate, 1e-12);
}

TEST(UavNmpcSolver, AnalyticReferenceSmallErrorsDoNotBangBodyRate) {
    ros::Time::init();
    UavNmpcSolver solver;
    ASSERT_TRUE(solver.initialize());

    ActiveTrajectoryCache cache;
    ASSERT_TRUE(cache.updateAnalytic(makeAnalyticReference(), ros::Time(10.0)));

    constexpr double kStageDt = 0.1;
    constexpr double kSaturationGuard = 3.0;
    int near_saturation_count = 0;
    double max_body_rate = 0.0;
    for (int phase = 0; phase < 360; phase += 30) {
        const double t0 = static_cast<double>(phase) * M_PI / 180.0;
        std::vector<Se3Reference> references;
        ASSERT_TRUE(cache.sampleHorizon(ros::Time(10.0 + t0), kStageDt,
                                        UavNmpcSolver::horizonSteps(), 100.0, 9.8066, references));

        Se3StateVector x0 = control::packState(references.front().state);
        const double angle = static_cast<double>(phase) * M_PI / 180.0;
        x0(0) += 0.01 * std::cos(angle);
        x0(1) += 0.01 * std::sin(angle);
        x0(3) += 0.02 * -std::sin(angle);
        x0(4) += 0.02 * std::cos(angle);

        solver.resetWarmStart();
        EXPECT_TRUE(solver.solve(x0, references.front().control.body_z_specific_force,
                                 references.front().control.body_z_specific_force,
                                 Eigen::Vector3d::Zero(), references))
            << "phase=" << phase << " status=" << solver.status();
        const Se3ControlVector command = solver.optimalControl();
        const double body_rate = command.tail<3>().cwiseAbs().maxCoeff();
        max_body_rate = std::max(max_body_rate, body_rate);
        if (body_rate > kSaturationGuard) {
            ++near_saturation_count;
        }
    }

    EXPECT_LE(max_body_rate, kSaturationGuard);
    EXPECT_EQ(near_saturation_count, 0);
}

}  // namespace
}  // namespace px4_multirotor_controller
