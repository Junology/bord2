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

    // Hyperbola
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

    for(double t = 0.0; t < 1.0; t += 0.125) {
        double theta = 2*M_PI*t;
        double r = 1.5 - 0.5*t;

        auto params2 = circle.intersectParams(
            Eigen::Vector2d(1.0-r*cos(theta), -1.0-r*sin(theta)),
            Eigen::Vector2d(1.0+r*cos(theta), -1.0+r*sin(theta)) );

        ASSERT_EQ(params2.size(), 2);
        EXPECT_LT(std::abs(params2[0] - (0.5-0.5*t)/(3.0-t)), 1e-14);
        EXPECT_LT(std::abs(params2[1] - (2.5-0.5*t)/(3.0-t)), 1e-14);
    }

    auto params3 = circle.intersectParams(2.0, -1.1, -0.2, -1.0);

    ASSERT_EQ(params3.size(), 2)
        << params3[0] << std::endl;

    EXPECT_TRUE(
        circle.intersectParams(
            Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0)).empty());

    EXPECT_TRUE(circle.intersectParams(0.8, -1.1, 1.1, -0.9).empty());
}

TEST(QuadraticCurve, Triangle)
{
    using VT = Eigen::Vector2d;

    // Unit circle with center (1.0,-1.0).
    QuadraticCurve circle{1.0, 1.0, 1.0, 0.0, -2.0, 2.0};

    for(double t = 0.0; t < 1.0; t += 0.125) {
        Eigen::Vector2d pt1(1.0 - 1.2*sin(M_PI*t),-1.0 + 1.2*cos(M_PI*t));
        Eigen::Vector2d pt2(1.0 - 0.6*t, -1.0 - 0.8*t);
        Eigen::Vector2d pt3(2.0, -1.1);

        auto beziers = circle.onTriangle({pt1, pt2, pt3});

        auto is12 = circle.intersectParams(pt1, pt2);
        auto is23 = circle.intersectParams(pt2, pt3);
        auto is31 = circle.intersectParams(pt3, pt1);

        EXPECT_TRUE(beziers.second);
        ASSERT_EQ(is12.size(), 1);
        ASSERT_EQ(is23.size(), 1);
        ASSERT_EQ(is31.size(), 2)
            << pt3 << std::endl << pt1 << std::endl;
        ASSERT_EQ(beziers.first.size(), 2);

        EXPECT_LT(
            (beziers.first[0].get<0>() - pt3 - is31[1]*(pt1-pt3)).norm(),
            1e-14);
        EXPECT_LT(
            (beziers.first[0].get<3>() - pt1 - is12[0]*(pt2-pt1)).norm(),
            1e-14);
        EXPECT_LT(
            (beziers.first[1].get<0>() - pt3 - is31[0]*(pt1-pt3)).norm(),
            1e-14);
        EXPECT_LT(
            (beziers.first[1].get<3>() - pt2 - is23[0]*(pt3-pt2)).norm(),
            1e-14);
    }

    // Hyperbola
    // (x-1)^2 - (y+1)^2 - 1 = 0
    QuadraticCurve hyperbola{1.0, -1.0, -1.0, 0.0, -2.0, -2.0};
    for(double t = 0.0; t < 1.0; t += 0.125) {
        VT pt1{3.0, -1.0};
        VT pt2{-1.0, -5.0};
        VT pt3{-1.0, -1.0 + 2.0*sqrt(3.0)*t};
        auto beziers2 = hyperbola.onTriangle({pt1, pt2, pt3});

        auto is12 = hyperbola.intersectParams(pt1, pt2);
        auto is23 = hyperbola.intersectParams(pt2, pt3);
        auto is31 = hyperbola.intersectParams(pt3, pt1);

        EXPECT_TRUE( beziers2.second != (t==0.5) );
        ASSERT_EQ(is12.size(), 1);
        ASSERT_EQ(is23.size(), (t>=0.5) ? 2 : 1);
        ASSERT_EQ(is31.size(), (t<=0.5) ? 2 : 1);
        ASSERT_EQ( beziers2.first.size(), 2 );

        if (t == 0.5) {
            EXPECT_LT(
                (beziers2.first[1].get<0>() - pt3 - is31[1]*(pt1-pt3)).norm(),
                1e-14);
            EXPECT_LT(
                (beziers2.first[1].get<3>() - pt1 - is12[0]*(pt2-pt1)).norm(),
                1e-14);
            EXPECT_LT(
                (beziers2.first[0].get<0>() - pt3).norm(), 1e-14);
            EXPECT_LT(
                (beziers2.first[0].get<3>() - pt2 - is23[0]*(pt3-pt2)).norm(),
                1e-14);
        }
        else if (t > 0.5) {
            EXPECT_LT(
                (beziers2.first[1].get<0>() - pt3 - is31[0]*(pt1-pt3)).norm(),
                1e-14);
            EXPECT_LT(
                (beziers2.first[1].get<3>() - pt1 - is12[0]*(pt2-pt1)).norm(),
                1e-14);
            EXPECT_LT(
                (beziers2.first[0].get<0>() - pt2 - is23[1]*(pt3-pt2)).norm(),
                1e-14);
            EXPECT_LT(
                (beziers2.first[0].get<3>() - pt2 - is23[0]*(pt3-pt2)).norm(),
                1e-14);
        }
        else {
            EXPECT_LT(
                (beziers2.first[1].get<0>() - pt3 - is31[1]*(pt1-pt3)).norm(),
                1e-14);
            EXPECT_LT(
                (beziers2.first[1].get<3>() - pt1 - is12[0]*(pt2-pt1)).norm(),
                1e-14);
            EXPECT_LT(
                (beziers2.first[0].get<0>() - pt3 - is31[0]*(pt1-pt3)).norm(),
                1e-14);
            EXPECT_LT(
                (beziers2.first[0].get<3>() - pt2 - is23[0]*(pt3-pt2)).norm(),
                1e-14);
        }
    };
}
