#include <gtest/gtest.h>

#include <cmath>

#include "estimator_vrpn_px4_rotor_state/RigidStateEstimate.h"
#include "px4_multirotor_controller/common/sensor_checks.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"
#include "px4_multirotor_controller/uav/active_trajectory_cache.h"
#include "px4_multirotor_controller/uav/nmpc_result_buffer.h"

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

TEST(ActiveTrajectoryCache, AnalyticReferenceSamplesAndBuildsHorizon) {
    ActiveTrajectoryCache cache;
    ASSERT_TRUE(cache.updateAnalytic(makeAnalyticReference(), ros::Time(10.0)));
    EXPECT_EQ(cache.trajectoryId(), 2U);
    EXPECT_EQ(cache.revision(), 3U);

    UavReferencePoint sample;
    ASSERT_TRUE(cache.sample(ros::Time(10.5), 1.0, sample));
    EXPECT_TRUE(sample.position.array().isFinite().all());
    EXPECT_TRUE(sample.snap.array().isFinite().all());

    std::vector<Se3Reference> horizon;
    ASSERT_TRUE(cache.sampleHorizon(ros::Time(10.0), 0.1, 10, 1.0, 9.8066, horizon));
    EXPECT_EQ(horizon.size(), 12U);
    EXPECT_TRUE(control::packState(horizon.front().state).array().isFinite().all());
    EXPECT_TRUE(control::packControl(horizon.front().control).array().isFinite().all());
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

    sensor.uav_state_estimator_flags = 0u;
    sensor.uav_state_estimator_state =
        estimator_vrpn_px4_rotor_state::RigidStateEstimate::STATE_SELF_CHECK;
    EXPECT_FALSE(sensor_checks::isStateEstimateUsableForControl(sensor));
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
    EXPECT_TRUE(solver.solve(x0, references)) << "status=" << solver.status();
    EXPECT_NEAR(solver.optimalControl()(0), 9.8066, 1e-3);
    EXPECT_NEAR(solver.predictedBodyRate().norm(), 0.0, 1e-3);
}

TEST(UavNmpcSolver, AnalyticReferenceSmallErrorsDoNotBangAngularAcceleration) {
    ros::Time::init();
    UavNmpcSolver solver;
    ASSERT_TRUE(solver.initialize());

    ActiveTrajectoryCache cache;
    ASSERT_TRUE(cache.updateAnalytic(makeAnalyticReference(), ros::Time(10.0)));

    constexpr double kStageDt = 0.1;
    constexpr double kSaturationGuard = 8.5;
    int near_saturation_count = 0;
    double max_angular_accel = 0.0;
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
        EXPECT_TRUE(solver.solve(x0, references))
            << "phase=" << phase << " status=" << solver.status();
        const Se3ControlVector command = solver.optimalControl();
        const double angular_accel = command.tail<3>().cwiseAbs().maxCoeff();
        max_angular_accel = std::max(max_angular_accel, angular_accel);
        if (angular_accel > kSaturationGuard) {
            ++near_saturation_count;
        }
    }

    EXPECT_LE(max_angular_accel, kSaturationGuard);
    EXPECT_EQ(near_saturation_count, 0);
}

}  // namespace
}  // namespace px4_multirotor_controller
