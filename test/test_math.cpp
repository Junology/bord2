#include <gtest/gtest.h>

#include "math/misc.hpp"

TEST(MathMisc, TriangleIntersection2D)
{
    std::array<Eigen::Vector2d,3> reftrig {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(0.0, 1.0)
    };

    EXPECT_TRUE(
        trianglesIntersection2D(
            reftrig,
            { Eigen::Vector2d(0.5, 0.5),
              Eigen::Vector2d(-0.5, 0.5),
              Eigen::Vector2d(0.5, -0.5) }));

    EXPECT_TRUE(
        trianglesIntersection2D(
            reftrig,
            { Eigen::Vector2d(0.25, 0.5),
              Eigen::Vector2d(-0.5, 1.0),
              Eigen::Vector2d(-0.5, 0.0) }));

    EXPECT_TRUE(
        trianglesIntersection2D(
            reftrig,
            { Eigen::Vector2d(1.0, 0.5),
              Eigen::Vector2d(-0.5, 1.0),
              Eigen::Vector2d(-0.5, 0.0) }));

    EXPECT_TRUE(
        trianglesIntersection2D(
            reftrig,
            { Eigen::Vector2d(-0.5, -0.5),
              Eigen::Vector2d(2.0, -0.5),
              Eigen::Vector2d(-0.5, 2.0) }));

    EXPECT_TRUE(
        trianglesIntersection2D(
            reftrig,
            { Eigen::Vector2d(0.2, 0.2),
              Eigen::Vector2d(0.6, 0.2),
              Eigen::Vector2d(0.2, 0.6) }));

    EXPECT_FALSE(
        trianglesIntersection2D(
            reftrig,
            { Eigen::Vector2d(-0.5, 2.0),
              Eigen::Vector2d(0.75, 0.5),
              Eigen::Vector2d(2.0, 1.0) }));
}
