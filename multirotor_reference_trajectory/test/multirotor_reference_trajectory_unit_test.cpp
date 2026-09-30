#include <gtest/gtest.h>

#include <cmath>
#include <xgc2_math/trajectory.hpp>

namespace {
namespace trajectory = xgc2_math::trajectory;

TEST(ReferenceTrajectoryCore, HeightCircleProvidesHighOrderDerivatives) {
    trajectory::CircleCurveParameters3 params;
    params.radius = 3.0;
    params.line_speed = 3.0;
    params.height = 3.0;
    params.z_amplitude = 1.0;
    params.z_frequency = 0.5;

    trajectory::CircleCurveEvaluator3 evaluator(params);
    trajectory::FlatOutput3 output;
    ASSERT_TRUE(evaluator.evaluate(0.0, output));
    EXPECT_NEAR(output.position.x(), 3.0, 1e-12);
    EXPECT_NEAR(output.velocity.y(), 3.0, 1e-12);
    EXPECT_TRUE(output.snap.allFinite());
    EXPECT_TRUE(std::isfinite(output.yaw_rate));
    EXPECT_TRUE(std::isfinite(output.yaw_accel));
}

TEST(ReferenceTrajectoryCore, AnalyticCurvesMatchReferenceFormulas) {
    trajectory::FlatOutput3 output;

    trajectory::LineCurveParameters3 line_params;
    line_params.duration = 4.0;
    line_params.start = Eigen::Vector3d::Zero();
    line_params.target = Eigen::Vector3d(1.0, 0.5, 1.5);
    trajectory::LineCurveEvaluator3 line(line_params);
    ASSERT_TRUE(line.evaluate(line_params.duration, output));
    EXPECT_TRUE(output.position.isApprox(line_params.target, 1e-12));
    EXPECT_TRUE(output.velocity.isApprox(Eigen::Vector3d::Zero(), 1e-12));

    trajectory::LemniscateCurveParameters3 lemniscate_params;
    lemniscate_params.radius = 2.0;
    lemniscate_params.omega = 0.5;
    lemniscate_params.height = 1.5;
    trajectory::LemniscateCurveEvaluator3 lemniscate(lemniscate_params);
    ASSERT_TRUE(lemniscate.evaluate(0.0, output));
    EXPECT_TRUE(output.position.isApprox(Eigen::Vector3d(0.0, 0.0, 1.5), 1e-12));
    EXPECT_TRUE(output.velocity.isApprox(Eigen::Vector3d(1.0, 1.0, 0.0), 1e-12));

    trajectory::HelixYzCurveParameters3 helix_yz_params;
    helix_yz_params.radius = 2.0;
    helix_yz_params.omega = 0.5;
    helix_yz_params.linear_scale = 10.0;
    trajectory::HelixYzCurveEvaluator3 helix_yz(helix_yz_params);
    ASSERT_TRUE(helix_yz.evaluate(0.0, output));
    EXPECT_TRUE(output.position.isApprox(Eigen::Vector3d(0.0, 2.0, 0.0), 1e-12));
    EXPECT_TRUE(output.velocity.isApprox(Eigen::Vector3d(0.1, 0.0, 1.0), 1e-12));

    trajectory::HelixXyCurveParameters3 helix_xy_params;
    helix_xy_params.radius = 2.0;
    helix_xy_params.omega = 0.5;
    helix_xy_params.linear_scale = 10.0;
    trajectory::HelixXyCurveEvaluator3 helix_xy(helix_xy_params);
    ASSERT_TRUE(helix_xy.evaluate(0.0, output));
    EXPECT_TRUE(output.position.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0), 1e-12));
    EXPECT_TRUE(output.velocity.isApprox(Eigen::Vector3d(0.0, 1.0, 0.1), 1e-12));

    trajectory::TorusKnotCurveParameters3 torus_params;
    torus_params.omega = 0.6;
    torus_params.scale = 0.4;
    trajectory::TorusKnotCurveEvaluator3 torus(torus_params);
    ASSERT_TRUE(torus.evaluate(0.0, output));
    EXPECT_TRUE(output.position.isApprox(Eigen::Vector3d(0.0, -0.4, 1.6), 1e-12));
    EXPECT_TRUE(output.velocity.isApprox(Eigen::Vector3d(1.2, 0.0, 0.72), 1e-12));
}

TEST(ReferenceTrajectoryCore, FlatnessMapperRejectsLowThrustSingularity) {
    trajectory::FlatOutput3 flat;
    flat.acceleration = -9.8066 * Eigen::Vector3d::UnitZ();

    const auto mapped = trajectory::FlatnessMapper3(9.8066, 0.1).map(flat);
    EXPECT_NE(mapped.flags & trajectory::kFlagLowThrust, 0U);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
