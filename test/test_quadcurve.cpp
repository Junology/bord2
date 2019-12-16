#include <gtest/gtest.h>

#include "math/QuadraticCurve.hpp"

TEST(QuadraticCurve, Evaluate)
{
    // Unit circle with center (1.0,-1.0).
    QuadraticCurve circle{1.0, 1.0, 1.0, 0.0, -2.0, 2.0};

    EXPECT_LT(std::abs(circle.evaluate(1.0, 0.0)), 1e-14);
    EXPECT_LT(std::abs(circle.evaluate(0.0, -1.0)), 1e-14);
    EXPECT_LT(std::abs(circle.evaluate(0.0, 0.0) - 1.0), 1e-14);
    EXPECT_LT(std::abs(circle.evaluate(1.0, -1.0) + 1.0), 1e-14);
}

TEST(QuadraticCurve, Tangent)
{
    // Unit circle with center (1.0,-1.0).
    QuadraticCurve circle{1.0, 1.0, 1.0, 0.0, -2.0, 2.0};

    AffHypPlane<2> tangent1 = circle.tangent(1.0, 0.0);

    EXPECT_LT(std::abs(tangent1.height(1.0, 0.0)), AffHypPlane<2>::threshold);
    EXPECT_LT(std::abs(tangent1.height(2.0, 0.0)), AffHypPlane<2>::threshold);

    AffHypPlane<2> tangent2 = circle.tangent(1.0, -2.0);

    EXPECT_LT(std::abs(tangent2.height(1.0, -2.0)), AffHypPlane<2>::threshold);
    EXPECT_LT(std::abs(tangent2.height(2.0, -2.0)), AffHypPlane<2>::threshold);
    EXPECT_FALSE(tangent1.intersect(tangent2).first)
        << "Intersect at" << tangent1.intersect(tangent2).second;
}

TEST(QuadraticCurve, Curvature)
{
    using VT = Eigen::Vector2d;
    // Unit circle with center (1.0,-1.0).
    QuadraticCurve circle{1.0, 1.0, 1.0, 0.0, -2.0, 2.0};
    for(double t = 0.0; t < 10.0; t += 1.0) {
        VT p(std::cos(0.2*t*M_PI)+1.0, std::sin(0.2*t*M_PI)-1.0);
        VT k = circle.curvature(p);
        EXPECT_LT((k+p-Eigen::Vector2d(1.0,-1.0)).norm(), 1e-14)
            << "k=" << std::endl << k << std::endl;
    }

    // Unit circle with center (1.0,-1.0); coefficients are negated.
    QuadraticCurve circle2{-1.0, -1.0, -1.0, 0.0, 2.0, -2.0};
    for(double t = 0.0; t < 10.0; t += 1.0) {
        VT p(std::cos(0.2*t*M_PI)+1.0, std::sin(0.2*t*M_PI)-1.0);
        VT k = circle2.curvature(p);
        EXPECT_LT((k+p-Eigen::Vector2d(1.0,-1.0)).norm(), 1e-14)
            << "k=" << std::endl << k << std::endl;
    }

    // Hyperboila
    QuadraticCurve hyperbola{1.0, -1.0, -1.0, 0.0, 0.0, 0.0};
    for(double t = 0.0; t < 10.0; t += 1.0) {
        double x = std::cosh(0.2*t*M_PI);
        double y = std::sinh(0.2*t*M_PI);
        VT k = hyperbola.curvature(x,y);

        // The formula for the curvature of hyperbola can be found in
        // http://mathworld.wolfram.com/Hyperbola.html
        EXPECT_LT(std::abs(k.norm()- 1.0/bord2::cipow(VT(x,y).norm(),3)), 1e-14);
    }
}

TEST(QuadraticCurve, DivisionAxis)
{
    // Vertical axis
    for(double t = 0.0; t < 10.0; t += 1.0) {
        // The division axis of (x-t)^2 - (1+t)(y-1)^2 - 1 = 0.
        auto axis = QuadraticCurve(1.0, -1-t, t*t-t-2, 0, -2*t, 2.0+2*t).divAxis();
        ASSERT_TRUE((bool)axis);
        EXPECT_LT(std::abs(axis->height(t,1)), 1e-14);
        EXPECT_LT(std::abs(axis->height(t,2)), 1e-14);
    }

    // Horizontal axis
    for(double t = 0.0; t < 10.0; t += 1.0) {
        // The division axis of (x-t)^2 - (1+t)(y-1)^2 + 1 = 0.
        auto axis = QuadraticCurve(1.0, -1-t, t*t-t, 0, -2*t, 2.0+2*t).divAxis();
        ASSERT_TRUE((bool)axis);
        EXPECT_LT(std::abs(axis->height(t,1)), 1e-14);
        EXPECT_LT(std::abs(axis->height(t+1,1)), 1e-14);
    }

    // Diagonal axis
    for(double t = 0.0; t < 10.0; t += 1.0) {
        // The division axis of (x-t)(y-1) - 1 = 0.
        auto axis = QuadraticCurve(0.0, 0.0, t-1, 1.0, -1.0, -t).divAxis();
        ASSERT_TRUE((bool)axis);
        EXPECT_LT(std::abs(axis->height(t,1)), 1e-14);
        EXPECT_LT(std::abs(axis->height(t+1,0)), 1e-14);
    }
}

TEST(QuadraticCurve, Intersect)
{
    // Unit circle with center (1.0,-1.0).
    QuadraticCurve circle{1.0, 1.0, 1.0, 0.0, -2.0, 2.0};

    auto params1 = circle.intersectParams(1.0, -1.0, 1.0+std::sqrt(2.0), -1.0+std::sqrt(2.0));

    ASSERT_EQ(params1.size(), 1);
    EXPECT_LT(std::abs(params1[0] - 0.5), 1e-14);

    auto params2 = circle.intersectParams(
        Eigen::Vector2d(0.0, -0.2),
        Eigen::Vector2d(2.0, -0.2) );

    ASSERT_EQ(params2.size(), 2);
    EXPECT_LT(std::abs(params2[0] - 0.2), 1e-14);
    EXPECT_LT(std::abs(params2[1] - 0.8), 1e-14);

    EXPECT_TRUE(
        circle.intersectParams(
            Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0)).empty());

    EXPECT_TRUE(circle.intersectParams(0.8, -1.1, 1.1, -0.9).empty());
}

/*
TEST(QuadraticCurve, Triangle)
{
    using VT = Eigen::Vector2d;

    // Unit circle with center (1.0,-1.0).
    QuadraticCurve circle{1.0, 1.0, 1.0, 0.0, -2.0, 2.0};

    auto beziers = circle.onTriangle(
        {VT(0.5,0.5), VT(0.5,-1.5), VT(2.5,-1.5)} );

    EXPECT_EQ(beziers.size(), 2);
    EXPECT_LT( (beziers[0].get<0>() - VT(1.0, 0.0)).norm(), QuadraticCurve::threshold)
        << beziers[0].get<0>() << std::endl << beziers[0].get<1>() << std::endl;
}
*/

