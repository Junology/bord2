#include <gtest/gtest.h>

#include "math/misc.hpp"

TEST(MathMisc, TriangleIntersection2D)
{
    std::array<Eigen::Vector2d,3> reftrig {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1,0, 0.0),
        Eigen::Vector2d(0.0, 1.0)
    };

    EXPECT_TRUE(
        triangleIntersection2D(
            reftrig,
            { Eigen::Vector2d(0.5, 0.5),
              Eigen::Vector2d(-0.5, 0.5),
              Eigen::Vector2d(0.5, -0.5) });
}
