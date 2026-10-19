#include <gtest/gtest.h>

#include <limits>

#include "px4_multirotor_controller/common/control_feedback.h"
#include "px4_multirotor_controller/driver/body_inertial_input.h"

using namespace px4_multirotor_controller;

namespace {
void localReady(SensorData& sensor) {
    sensor.feedback_source = FeedbackSource::MAVROS_LOCAL;
    sensor.local_pos_stats.is_active = sensor.local_velocity_stats.is_active = true;
    sensor.imu_stats.is_active = sensor.state_stats.is_active = true;
    sensor.local_qw = 1.0;
    BodyInertialInput imu;
    imu.specific_force.z() = 9.8066;
    imu.stamp_sec = 100.0;
    ASSERT_TRUE(ingestBodyInertial(sensor, imu, 9.8066));
}
}  // namespace

TEST(LocalFeedback, BodySpecificForceBecomesWorldAccelerationWithoutRseMutation) {
    SensorData sensor;
    sensor.x = 91.0;
    sensor.qw = .37;
    sensor.uav_state_estimator_flags = 0x1234;
    sensor.uav_state_estimate_stamp = 77.0;
    sensor.uav_state_filter_inertial_stamp = 78.0;
    BodyInertialInput imu;
    imu.orientation = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitY());
    imu.body_rate << .1, -.2, .3;
    const Eigen::Vector3d desired(1.0, 2.0, -3.0);
    imu.specific_force =
        imu.orientation.conjugate() * (desired + 9.8066 * Eigen::Vector3d::UnitZ());
    imu.stamp_sec = 100.0;
    ASSERT_TRUE(ingestBodyInertial(sensor, imu, 9.8066));
    EXPECT_LT((sensor.mavros_world_acceleration - desired).norm(), 1.e-12);
    EXPECT_EQ(sensor.mavros_body_rate, imu.body_rate);
    EXPECT_DOUBLE_EQ(sensor.mavros_imu_stamp, 100.0);
    EXPECT_DOUBLE_EQ(sensor.x, 91.0);
    EXPECT_DOUBLE_EQ(sensor.qw, .37);
    EXPECT_EQ(sensor.uav_state_estimator_flags, 0x1234U);
    EXPECT_DOUBLE_EQ(sensor.uav_state_estimate_stamp, 77.0);
    EXPECT_DOUBLE_EQ(sensor.uav_state_filter_inertial_stamp, 78.0);
    imu.orientation.coeffs().setZero();
    EXPECT_FALSE(ingestBodyInertial(sensor, imu, 9.8066));
    EXPECT_FALSE(sensor.mavros_inertial_valid);
}

TEST(LocalFeedback, HealthUsesSelectedSourceAndDoesNotInventRseHealth) {
    SensorData sensor;
    localReady(sensor);
    sensor.uav_state_estimator_flags = state_estimate::FLAG_FAULT;
    EXPECT_TRUE(sensor_checks::areSensorsReady(sensor, TrackingBackend::DFBC));
    EXPECT_TRUE(sensor_checks::areSensorsReady(sensor, TrackingBackend::NMPC));
    sensor.imu_stats.is_active = false;
    EXPECT_FALSE(sensor_checks::areSensorsReady(sensor, TrackingBackend::DFBC));
    sensor.imu_stats.is_active = true;
    sensor.mavros_inertial_valid = false;
    EXPECT_FALSE(sensor_checks::areSensorsReady(sensor, TrackingBackend::DFBC));
    // Existing local-setpoint / SMC contracts do not need numerical IMU or RSE.
    EXPECT_TRUE(sensor_checks::areSensorsReady(sensor, TrackingBackend::PX4_LOCAL));
    EXPECT_TRUE(sensor_checks::areSensorsReady(sensor, TrackingBackend::SMC));
    sensor.feedback_source = FeedbackSource::RIGID_ESTIMATE;
    sensor.uav_state_estimate_stats.is_active = true;
    sensor.uav_state_estimator_state = state_estimate::STATE_RUNNING;
    EXPECT_FALSE(sensor_checks::areSensorsReady(sensor, TrackingBackend::DFBC));
    sensor.uav_state_estimator_flags = 0;
    EXPECT_TRUE(sensor_checks::areSensorsReady(sensor, TrackingBackend::DFBC));
}

TEST(LocalFeedback, PositionVelocityAndQuaternionUseOneSelectedSource) {
    SensorData sensor;
    sensor.x = 1;
    sensor.vx = 2;
    sensor.qw = .3;
    sensor.qx = .4;
    sensor.local_x = 11;
    sensor.local_vx = 12;
    sensor.local_qw = .7;
    sensor.local_qx = .8;
    EXPECT_DOUBLE_EQ(sensor_checks::worldX(sensor, TrackingBackend::DFBC), 1);
    EXPECT_DOUBLE_EQ(sensor_checks::worldQw(sensor, TrackingBackend::DFBC), .3);
    sensor.feedback_source = FeedbackSource::MAVROS_LOCAL;
    EXPECT_DOUBLE_EQ(sensor_checks::worldX(sensor, TrackingBackend::DFBC), 11);
    EXPECT_DOUBLE_EQ(sensor_checks::worldVx(sensor, TrackingBackend::DFBC), 12);
    EXPECT_DOUBLE_EQ(sensor_checks::worldQw(sensor, TrackingBackend::DFBC), .7);
    EXPECT_DOUBLE_EQ(sensor_checks::worldQx(sensor, TrackingBackend::DFBC), .8);
    sensor.feedback_source = FeedbackSource::RIGID_ESTIMATE;
    EXPECT_DOUBLE_EQ(sensor_checks::worldQw(sensor, TrackingBackend::PX4_LOCAL), .7);
    EXPECT_DOUBLE_EQ(sensor_checks::worldQw(sensor, TrackingBackend::SMC), .7);
}

TEST(LocalFeedback, SharedDfbcNmpcMapperMatchesRigidStateExactly) {
    SensorData rigid, local;
    localReady(local);
    local.local_x = 1;
    local.local_y = 2;
    local.local_z = 3;
    local.local_vx = .1;
    local.local_vy = .2;
    local.local_vz = .3;
    const Eigen::Quaterniond q(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()));
    local.local_qx = q.x();
    local.local_qy = q.y();
    local.local_qz = q.z();
    local.local_qw = q.w();
    local.mavros_body_rate << .4, .5, .6;
    rigid.x = 1;
    rigid.y = 2;
    rigid.z = 3;
    rigid.vx = .1;
    rigid.vy = .2;
    rigid.vz = .3;
    rigid.qx = q.x();
    rigid.qy = q.y();
    rigid.qz = q.z();
    rigid.qw = q.w();
    rigid.wx = .4;
    rigid.wy = .5;
    rigid.wz = .6;
    rigid.uav_state_estimate_stats.is_active = true;
    rigid.uav_state_estimator_state = state_estimate::STATE_RUNNING;
    for (auto backend : {TrackingBackend::DFBC, TrackingBackend::NMPC}) {
        xgc2_math::control::Se3State a, b;
        ASSERT_TRUE(readControlFeedback(rigid, backend, a));
        ASSERT_TRUE(readControlFeedback(local, backend, b));
        EXPECT_EQ(a.position, b.position);
        EXPECT_EQ(a.velocity, b.velocity);
        EXPECT_EQ(a.attitude.coeffs(), b.attitude.coeffs());
        EXPECT_EQ(a.body_rate, b.body_rate);
    }
}
