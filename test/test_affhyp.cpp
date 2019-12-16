#include <gtest/gtest.h>

#include "math/AffHypPlane.hpp"

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
