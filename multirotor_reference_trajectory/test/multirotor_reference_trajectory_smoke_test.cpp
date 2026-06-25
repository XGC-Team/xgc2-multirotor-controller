#include <gtest/gtest.h>

#include <xgc2_math/trajectory.hpp>

TEST(ReferenceTrajectorySmoke, AnalyticTypesSampleFiniteValues) {
    namespace trajectory = xgc2_math::trajectory;
    trajectory::FlatOutput3 output;

    EXPECT_TRUE(trajectory::HoldCurveEvaluator3().evaluate(0.5, output));
    EXPECT_TRUE(trajectory::TrajectoryValidator3::finite(output));

    EXPECT_TRUE(trajectory::CircleCurveEvaluator3().evaluate(0.5, output));
    EXPECT_TRUE(trajectory::TrajectoryValidator3::finite(output));

    trajectory::CircleCurveParameters3 height_circle;
    height_circle.z_amplitude = 1.0;
    EXPECT_TRUE(trajectory::CircleCurveEvaluator3(height_circle).evaluate(0.5, output));
    EXPECT_TRUE(trajectory::TrajectoryValidator3::finite(output));

    EXPECT_TRUE(trajectory::CircleEntryCurveEvaluator3().evaluate(0.5, output));
    EXPECT_TRUE(trajectory::TrajectoryValidator3::finite(output));

    EXPECT_TRUE(trajectory::FigureEightCurveEvaluator3().evaluate(0.5, output));
    EXPECT_TRUE(trajectory::TrajectoryValidator3::finite(output));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
