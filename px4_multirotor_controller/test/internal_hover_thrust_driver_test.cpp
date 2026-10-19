#include <gtest/gtest.h>

#include <limits>

#include "px4_multirotor_controller/common/control_feedback.h"
#include "px4_multirotor_controller/driver/controller_config.h"
#include "px4_multirotor_controller/driver/controller_driver.h"
#include "px4_multirotor_controller/tracking/dfbc_attitude_rate_strategy.h"

using namespace px4_multirotor_controller;

TEST(InternalHoverThrust, DefaultExternalDoesNotCreateRuntime) {
    SensorData sensor;
    ControllerDriver driver(sensor);
    driver.start(Time(100.0));
    EXPECT_EQ(driver.hoverThrustRuntime(), nullptr);
    EXPECT_EQ(sensor.feedback_source, FeedbackSource::RIGID_ESTIMATE);
    driver.stop();
}

TEST(InternalHoverThrust, SameRuntimeSameSamplesSameEstimatesAndPublicationGates) {
    SensorData sensor;
    ControllerDriver driver(sensor);
    ControllerConfig config;
    config.feedback_source = FeedbackSource::MAVROS_LOCAL;
    config.hover_thrust.source = HoverThrustSource::INTERNAL;
    config.hover_thrust.estimator.filter_enabled = false;
    config.hover_thrust.estimator.publish_rate_hz = 50;
    driver.controller().setConfig(config);
    driver.start(Time(100.0));
    ASSERT_NE(driver.hoverThrustRuntime(), nullptr);
    hover_thrust_estimator::HoverThrustEstimatorRuntime reference;
    reference.setConfig(config.hover_thrust.estimator);
    BodyInertialInput imu;
    imu.specific_force.z() = config.hover_thrust.estimator.gravity;
    unsigned publications = 0;
    for (int i = 0; i < 70; ++i) {
        const double now = Time(100.0 + i * .02).toSec();
        imu.stamp_sec = now;
        const bool ignored = i >= 50 && i < 54;
        const double thrust = i == 58 ? std::numeric_limits<double>::quiet_NaN() : .4;
        driver.ingestBodyImu(imu, now);
        driver.ingestThrust(thrust, ignored, now, now);
        driver.ingestAltitude(1.0, now, now);
        ASSERT_TRUE(reference.ingestImu(imu.specific_force.z(), now, now).ok());
        ASSERT_TRUE(reference.ingestThrust(thrust, ignored, now, now).ok());
        ASSERT_TRUE(reference.ingestAltitude(1.0, now, now).ok());
        driver.update(Time(now));
        reference.update(now);
        const auto* actual = driver.hoverThrustRuntime();
        ASSERT_EQ(actual->publishedEstimates().size(), reference.publishedEstimates().size());
        EXPECT_DOUBLE_EQ(actual->snapshotOutput().hover_thrust,
                         reference.snapshotOutput().hover_thrust);
        EXPECT_DOUBLE_EQ(actual->snapshotOutput().raw_hover_thrust,
                         reference.snapshotOutput().raw_hover_thrust);
        EXPECT_EQ(actual->snapshotOutput().flags, reference.snapshotOutput().flags);
        EXPECT_EQ(actual->snapshotOutput().state, reference.snapshotOutput().state);
        EXPECT_DOUBLE_EQ(actual->input().imu_acc_z.value, imu.specific_force.z());
        for (size_t j = 0; j < reference.publishedEstimates().size(); ++j) {
            const auto& expected = reference.publishedEstimates()[j];
            EXPECT_DOUBLE_EQ(actual->publishedEstimates()[j].stamp_sec, expected.stamp_sec);
            EXPECT_DOUBLE_EQ(actual->publishedEstimates()[j].output.hover_thrust,
                             expected.output.hover_thrust);
            ++publications;
        }
    }
    EXPECT_GT(publications, 10U);
    EXPECT_NEAR(sensor.mavros_world_acceleration.norm(), 0, 1.e-12);
    // Delayed data and a backwards source stamp retain the Runtime's own policy.
    const double old_stamp = sensor.hover_thrust_estimate_stamp;
    driver.update(Time(102.0));
    reference.update(102.0);
    EXPECT_EQ(driver.hoverThrustRuntime()->snapshotOutput().flags,
              reference.snapshotOutput().flags);
    EXPECT_LE(sensor.hover_thrust_estimate_stamp, 102.0);
    EXPECT_GE(sensor.hover_thrust_estimate_stamp, old_stamp);
    imu.stamp_sec = 100.0;
    driver.ingestBodyImu(imu, 102.1);
    reference.ingestImu(imu.specific_force.z(), 100.0, 102.1);
    driver.update(Time(102.1));
    reference.update(102.1);
    EXPECT_EQ(driver.hoverThrustRuntime()->snapshotOutput().flags,
              reference.snapshotOutput().flags);
    driver.stop();
    EXPECT_EQ(driver.hoverThrustRuntime(), nullptr);
}

TEST(InternalHoverThrust, ExplicitConfigurationSourcesAndDefaults) {
    auto read = [](const std::string& key, const std::string& val) {
        return readControllerConfig(ControllerParameters(
            [&](const std::string& requested, ControllerParameterValue& value) {
                if (requested == "world_boundary_json") {
                    value = std::string("null");
                    return true;
                }
                if (requested == key) {
                    value = val;
                    return true;
                }
                return false;
            },
            {}, false));
    };
    const auto defaults = read("", "");
    EXPECT_EQ(defaults.feedback_source, FeedbackSource::RIGID_ESTIMATE);
    EXPECT_EQ(defaults.hover_thrust.source, HoverThrustSource::EXTERNAL);
    EXPECT_EQ(defaults.dfbc.reference_source, DfbcReferenceSource::TRAJECTORY_CACHE);
    EXPECT_EQ(read("feedback_source", "mavros_local").feedback_source,
              FeedbackSource::MAVROS_LOCAL);
    EXPECT_EQ(read("hover_thrust/source", "internal").hover_thrust.source,
              HoverThrustSource::INTERNAL);
    EXPECT_EQ(read("dfbc/reference_source", "planner_setpoint").dfbc.reference_source,
              DfbcReferenceSource::PLANNER_SETPOINT);
    EXPECT_THROW(read("feedback_source", "truth-ish"), std::invalid_argument);
    EXPECT_THROW(read("hover_thrust/source", "both"), std::invalid_argument);
    EXPECT_THROW(read("dfbc/reference_source", "guess"), std::invalid_argument);
}

TEST(LocalFeedback, DfbcProducesSameControlForEquivalentDeclaredStates) {
    SensorData rigid, local;
    local.feedback_source = FeedbackSource::MAVROS_LOCAL;
    local.local_pos_stats.is_active = local.local_velocity_stats.is_active =
        local.imu_stats.is_active = true;
    local.local_qw = 1;
    BodyInertialInput imu;
    imu.specific_force.z() = 9.8066;
    imu.stamp_sec = 100;
    ASSERT_TRUE(ingestBodyInertial(local, imu, 9.8066));
    local.local_x = .2;
    local.local_y = -.3;
    local.local_z = 1.1;
    local.local_vx = .01;
    local.local_vy = -.02;
    local.local_vz = .03;
    local.mavros_body_rate << .1, .2, -.1;
    rigid.x = local.local_x;
    rigid.y = local.local_y;
    rigid.z = local.local_z;
    rigid.vx = local.local_vx;
    rigid.vy = local.local_vy;
    rigid.vz = local.local_vz;
    rigid.qw = 1;
    rigid.wx = .1;
    rigid.wy = .2;
    rigid.wz = -.1;
    rigid.uav_state_estimate_stats.is_active = true;
    rigid.uav_state_estimator_state = state_estimate::STATE_RUNNING;
    rigid.uav_state_estimate_stamp = local.mavros_imu_stamp;
    for (auto* sensor : {&rigid, &local}) {
        sensor->hover_thrust_estimate_available = true;
        sensor->hover_thrust_estimate = .4;
        sensor->hover_thrust_estimate_stamp = 100.0;
    }
    ControllerConfig config;
    config.nmpc.hover_thrust_enabled = true;
    config.dfbc.acceleration_correction_enabled = true;
    config.nmpc.enable_timing_log = false;
    DfbcAttitudeRateStrategy a, b;
    a.configure(config);
    b.configure(config);
    ASSERT_TRUE(a.enter(rigid, Time(100.01)));
    ASSERT_TRUE(b.enter(local, Time(100.01)));
    UavReferencePoint reference;
    reference.position << .3, -.2, 1.0;
    TrackingStrategyResult x, y;
    TrackingStrategyInput input_a, input_b;
    input_a.sensor = rigid;
    input_b.sensor = local;
    input_a.reference = input_b.reference = reference;
    input_a.now = input_b.now = Time(100.01);
    ASSERT_TRUE(a.update(input_a, x));
    ASSERT_TRUE(b.update(input_b, y));
    EXPECT_DOUBLE_EQ(x.attitude_rate_target.thrust, y.attitude_rate_target.thrust);
    EXPECT_DOUBLE_EQ(x.attitude_rate_target.body_rate_x, y.attitude_rate_target.body_rate_x);
    EXPECT_DOUBLE_EQ(x.attitude_rate_target.body_rate_y, y.attitude_rate_target.body_rate_y);
    EXPECT_DOUBLE_EQ(x.attitude_rate_target.body_rate_z, y.attitude_rate_target.body_rate_z);
}
