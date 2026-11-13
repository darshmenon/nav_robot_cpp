#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "nav_robot_cpp/logic.hpp"

using namespace nav_robot_cpp;

TEST(Logic, CountFiniteIgnoresInfAndNan)
{
  const float inf = std::numeric_limits<float>::infinity();
  EXPECT_EQ(countFinite({1.0f, inf, 2.0f, std::nanf(""), 3.0f}), 3);
  EXPECT_EQ(countFinite({inf, inf, inf}), 0);   // a blind lidar
  EXPECT_EQ(countFinite({}), 0);
}

TEST(Logic, NormalizeAngleWraps)
{
  EXPECT_NEAR(normalizeAngle(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(normalizeAngle(-3.5 * M_PI), 0.5 * M_PI, 1e-9);
  EXPECT_NEAR(normalizeAngle(0.3), 0.3, 1e-12);
}

TEST(Logic, InterpolateIncludesEndsAndSpacing)
{
  const auto pts = interpolate(0, 0, 1, 0, 0.1);
  ASSERT_EQ(pts.size(), 11u);
  EXPECT_NEAR(pts.front().first, 0.0, 1e-12);
  EXPECT_NEAR(pts.back().first, 1.0, 1e-12);
  EXPECT_EQ(interpolate(2, 2, 2, 2, 0.1).size(), 2u);   // zero-length segment still gives two points
}

TEST(Logic, PursuitCurvatureSignAndMagnitude)
{
  EXPECT_NEAR(pursuitCurvature(1.0, 0.0), 0.0, 1e-12);          // straight ahead
  EXPECT_GT(pursuitCurvature(1.0, 0.5), 0.0);                   // target on the left turns left
  EXPECT_LT(pursuitCurvature(1.0, -0.5), 0.0);
  EXPECT_NEAR(pursuitCurvature(0.0, 1.0), 2.0, 1e-12);          // 90 degrees at L=1 gives 2/L
  EXPECT_DOUBLE_EQ(pursuitCurvature(0.0, 0.0), 0.0);
}
