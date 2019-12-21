#include <gtest/gtest.h>

#include <Eigen/Dense>
#include "math/Bezier.hpp"

TEST(Bezier, Linear2D)
{
    using VT = Eigen::Vector2d;

    auto bez = Bezier<VT,1>(VT(0.0, 0.0), VT(1.0, 2.0));
    EXPECT_TRUE(bez.get<0>() == VT(0.0, 0.0));
    EXPECT_TRUE(bez.get<1>() == VT(1.0, 2.0));
    EXPECT_LT( (bez.eval(0.2) - VT(0.2, 0.4)).norm(), 10e-14 );
    EXPECT_LT( (bez.eval(0.5) - VT(0.5, 1.0)).norm(), 10e-14 );
    EXPECT_LT( (bez.eval(0.7) - VT(0.7, 1.4)).norm(), 10e-14 );
}

TEST(Bezier, LinearDiv)
{
    using VT = Eigen::Vector2d;

    auto bez = Bezier<VT,1>(VT(0.0, 0.0), VT(1.0, 2.0));
    auto bezdiv = bez.divide();

    EXPECT_LT( (bezdiv.first.get<0>()-bez.eval(0.0)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.first.get<1>()-bez.eval(0.5)).norm(), 10e-14 );
    EXPECT_LT( (bezdiv.second.get<0>()-bez.eval(0.5)).norm(), 10e-14 )
        << bezdiv.second.get<0>() << std::endl;
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
