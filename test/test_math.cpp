#include <gtest/gtest.h>

#include "math/funcs.hpp"
#include "math/AffHypPlane.hpp"
#include "math/Bezier.hpp"
#include "math/QuadraticCurve.hpp"

TEST(TestSolvers, Linear)
{
    using namespace bord2;

    ASSERT_EQ(solveLinearRange(1.0, -1.0, {0.0, 2.0}).size(), 1);
    EXPECT_EQ(solveLinearRange(1.0, -1.0, {0.0, 2.0})[0], 1.0);

    EXPECT_TRUE(solveLinearRange(1.0, -1.0, {-1.0, 0.0}).empty());
    EXPECT_TRUE(solveLinearRange(1.0, -1.0, {2.0, 3.0}).empty());
    EXPECT_TRUE(solveLinearRange(1.0, -1.0, {2.0, -1.0}).empty());

    ASSERT_EQ(solveLinearRange(4.0, -2.0, {0.0, 1.0}).size(), 1);
    EXPECT_EQ(solveLinearRange(4.0, -2.0, {0.0, 2.0})[0], 0.5);

    EXPECT_TRUE(solveLinearRange(0.0, 1.0, {0.0, 2.0}).empty());
    ASSERT_EQ(solveLinearRange(0.0, 0.0, {0.0, 2.0}).size(), 2);
    EXPECT_EQ(solveLinearRange(0.0, 0.0, {0.0, 2.0})[0], 0.0);
    EXPECT_EQ(solveLinearRange(0.0, 0.0, {0.0, 2.0})[1], 2.0);
    EXPECT_TRUE(solveLinearRange(0.0, 0.0, {2.0, -1.0}).empty());
}

TEST(TestSolvers, Quadratic)
{
    using namespace bord2;

    ASSERT_EQ(solveQuadRange(1.0, -1.0, 0.0, {-1.0, 2.0}).first.size(), 2);
    EXPECT_LT(
        solveQuadRange(1.0, -1.0, 0.0, {-1.0, 2.0}).first[0] - 1.0, 10e-14);
    EXPECT_LT(
        solveQuadRange(1.0, -1.0, 0.0, {-1.0, 2.0}).first[1] - 0.0, 10e-14);

    ASSERT_EQ(
        solveQuadRange(1.0, -1.0, 0.0, {-1.0, 1.0}).first.size(), 1);
    EXPECT_LT(
        solveQuadRange(1.0, -1.0, 0.0, {-1.0, 1.0}).first[1] - 0.0, 10e-14);

    ASSERT_EQ(
        solveQuadRange(1.0, -1.0, 0.0, {0.5, 2.0}).first.size(), 1);
    EXPECT_LT(
        solveQuadRange(1.0, -1.0, 0.0, {0.5, 2.0}).first[0] - 1.0, 10e-14);

    ASSERT_EQ(
        solveQuadRange(1.0, -2.0, 1.0, {0.0, 2.0}).first.size(), 1);
    EXPECT_TRUE(
        solveQuadRange(1.0, -2.0, 1.0, {0.0, 2.0}).second);
    EXPECT_LT(
        solveQuadRange(1.0, -2.0, 1.0, {0.0, 2.0}).first[0] - 1.0, 10e-14);

    EXPECT_TRUE(solveQuadRange(1.0, -2.0, 2.0, {0.0, 2.0}).first.empty());
}

TEST(TestAffHypPlane, Height1dim)
{
    using VT = typename AffHypPlane<1>::VecT;
    AffHypPlane<1> plane({1.0}, 2.0);

    EXPECT_EQ(plane.height(VT{1.0}), 3.0);
    EXPECT_EQ(plane.height(1.0), 3.0);

    EXPECT_EQ(plane.height(-3.0), -1.0);
    EXPECT_EQ(plane.height(-2.0), 0.0);

    // EXPECT_EQ(plane.height(1.0, 0.0), 0.0)
    // ^ This must cause a compile error thanks to SFINAE;
}

TEST(TestAffHypPlane, Intersect1dim)
{
    using VT = typename AffHypPlane<1>::VecT;

    AffHypPlane<1> plane1({1.0}, 2.0);
    AffHypPlane<1> plane2({2.0}, 3.0);
    AffHypPlane<1> plane3(VT(2.0), VT(-2.0));

    EXPECT_TRUE(plane1.intersect(plane1).first);
    EXPECT_LT((plane1.intersect(plane1).second - VT(-2.0)).norm(), AffHypPlane<1>::threshold);
    EXPECT_FALSE(plane1.intersect(plane2).first);
    EXPECT_TRUE(plane1.intersect(plane3).first);
    EXPECT_TRUE(plane1.intersect(plane3).second == VT(-2.0));
}

TEST(TestAffHypPlane, Height2dim)
{
    using VT = typename AffHypPlane<2>::VecT;
    AffHypPlane<2> plane({1.0, 1.0}, -2.0);

    EXPECT_EQ(plane.height(VT{0.0, 1.0}), -1.0);
    EXPECT_EQ(plane.height(2.0, 1.0), 1.0);

    AffHypPlane<2> plane2(VT{2.0, 0.0}, VT{1.0, 3.0});

    EXPECT_EQ(plane2.height(VT{1.0, 2.0}), 0.0);
}

TEST(TestAffHypPlane, Intersect2dim)
{
    using VT = typename AffHypPlane<2>::VecT;
    // x-y+1=0
    AffHypPlane<2> plane1({1.0,-1.0}, 1.0);
    // 2x+y+2=0
    AffHypPlane<2> plane2({2.0, 1.0}, 2.0);
    // 2(x-0)-2(y+4)=0 <=> x-y-5=0
    AffHypPlane<2> plane3(VT(2.0,-2.0), VT(0.0, -5.0));

    EXPECT_TRUE(plane1.intersect(plane1).first);
    EXPECT_TRUE(plane1.intersect(plane2).first);
    EXPECT_LT((plane1.intersect(plane2).second - VT(-1.0, 0.0)).norm(), AffHypPlane<2>::threshold);
    EXPECT_FALSE(plane1.intersect(plane3).first);
    EXPECT_FALSE(plane3.intersect(plane1).first);
    EXPECT_TRUE(plane3.intersect(plane2).first);
    EXPECT_LT((plane3.intersect(plane2).second - VT(1.0, -4.0)).norm(), AffHypPlane<2>::threshold);
}

TEST(Bezier, Linear2D)
{
    using VT = Eigen::Vector2d;

    auto bez = Bezier<VT,1>(VT(0.0, 0.0), VT(1.0, 2.0));
    EXPECT_TRUE(bez.get<0>() == VT(0.0, 0.0));
    EXPECT_TRUE(bez.get<1>() == VT(1.0, 2.0));
    EXPECT_LT( (bez.eval(0.2) - VT(0.2, 0.4)).norm(), 10e-14 );
    EXPECT_LT( (bez.eval(0.5) - VT(0.5, 1.0)).norm(), 10e-14 );
    EXPECT_LT( (bez.eval(0.7) - VT(0.7, 1.4)).norm(), 10e-14 );

    auto bezdiv = bez.divide();
}

TEST(Bezier, LinearDiv)
{
    using VT = Eigen::Vector2d;

    auto bez = Bezier<VT,1>(VT(0.0, 0.0), VT(1.0, 2.0));
    auto bezdiv = bez.divide();

    EXPECT_LT( (bezdiv.first.get<0>()-bez.eval(0.0)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.first.get<1>()-bez.eval(0.5)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.second.get<0>()-bez.eval(0.5)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.second.get<1>()-bez.eval(1.0)).norm(), 10e-14 )
        << bezdiv.second.get<1>() << std::endl;
}

TEST(Bezier, Quad2D)
{
    using VT = Eigen::Vector2d;

    auto bez = Bezier<VT,2>(VT(2.0, 0.0), VT(0.0, 0.0), VT(0.0, 2.0));

    EXPECT_TRUE(bez.get<0>() == VT(2.0, 0.0));
    EXPECT_TRUE(bez.get<1>() == VT(0.0, 0.0));
    EXPECT_TRUE(bez.get<2>() == VT(0.0, 2.0));
    EXPECT_LT( (bez.eval(0.2) - VT(1.28, 0.08)).norm(), 10e-14 );
    EXPECT_LT( (bez.eval(0.5) - VT(0.5, 0.5)).norm(), 10e-14 );
    EXPECT_LT( (bez.eval(0.8) - VT(0.08, 1.28)).norm(), 10e-14 );
}

TEST(Bezier, QuadDiv)
{
    using VT = Eigen::Vector2d;

    auto bez = Bezier<VT,2>(VT(2.0, 0.0), VT(0.0, 0.0), VT(0.0, 2.0));
    auto bezdiv = bez.divide();

    EXPECT_LT( (bezdiv.first.get<0>() - bez.eval(0.0)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.first.get<1>() - VT(1.0,0.0)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.first.get<2>() - bez.eval(0.5)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.second.get<0>() - bez.eval(0.5)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.second.get<1>() - VT(0.0,1.0)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.second.get<2>() - bez.eval(1.0)).norm(), 10e-14 );

    for(double t = 0.0; t < 1.0; t += 0.1) {
        EXPECT_LT( (bezdiv.first.eval(t) - bez.eval(t/2.0)).norm(), 10e-14 )
            << "different at " << t << std::endl;
        EXPECT_LT( (bezdiv.second.eval(t) - bez.eval(0.5+t/2.0)).norm(), 10e-14 )
            << "different at " << t << std::endl;
    }
}

TEST(QuadraticCurve, Evaluate)
{
    // Unit circle with center (1.0,-1.0).
    QuadraticCurve circle{1.0, 1.0, 1.0, 0.0, -2.0, 2.0};

    EXPECT_LT(circle.evaluate(1.0, 0.0), AffHypPlane<2>::threshold);
    EXPECT_LT(circle.evaluate(0.0, -1.0), AffHypPlane<2>::threshold);
    EXPECT_LT(circle.evaluate(0.0, 0.0) - 2.0, AffHypPlane<2>::threshold);
    EXPECT_LT(circle.evaluate(1.0, -1.0) + 1.0, AffHypPlane<2>::threshold);
}

TEST(QuadraticCurve, Tangent)
{
    // Unit circle with center (1.0,-1.0).
    QuadraticCurve circle{1.0, 1.0, 1.0, 0.0, -2.0, 2.0};

    AffHypPlane<2> tangent1 = circle.tangent(1.0, 0.0);

    EXPECT_LT(tangent1.height(1.0, 0.0), AffHypPlane<2>::threshold);
    EXPECT_LT(tangent1.height(2.0, 0.0), AffHypPlane<2>::threshold);

    AffHypPlane<2> tangent2 = circle.tangent(1.0, -2.0);

    EXPECT_LT(tangent2.height(1.0, -2.0), AffHypPlane<2>::threshold);
    EXPECT_LT(tangent2.height(2.0, -2.0), AffHypPlane<2>::threshold);
    EXPECT_FALSE(tangent1.intersect(tangent2).first)
        << "Intersect at" << tangent1.intersect(tangent2).second;
}

TEST(QuadraticCurve, Intersect)
{
    // Unit circle with center (1.0,-1.0).
    QuadraticCurve circle{1.0, 1.0, 1.0, 0.0, -2.0, 2.0};

    auto params1 = circle.intersectParams(1.0, -1.0, 1.0+std::sqrt(2.0), -1.0+std::sqrt(2.0));

    ASSERT_EQ(params1.size(), 1);
    EXPECT_LT(params1[0] - 0.5, 10e-14);

    auto params2 = circle.intersectParams(
        Eigen::Vector2d(0.0, -0.2),
        Eigen::Vector2d(2.0, -0.2) );

    ASSERT_EQ(params2.size(), 2);
    EXPECT_LT(params2[0] - 0.8, 10e-14);
    EXPECT_LT(params2[1] - 0.2, 10e-14);

    EXPECT_TRUE(
        circle.intersectParams(
            Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0)).empty());

    EXPECT_TRUE(circle.intersectParams(0.8, -1.1, 1.1, -0.9).empty());
}
