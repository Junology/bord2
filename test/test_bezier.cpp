#include <gtest/gtest.h>

#include <Eigen/Dense>
#include "math/Bezier.hpp"
#include "math/Bezier2D.hpp"

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
            << "different at " << t << std::endl
            << "bez(t/2) =" << bez.eval(t/2.0).adjoint() << std::endl
            << "bezdiv(t)=" << bezdiv.first.eval(t).adjoint() << std::endl;
        EXPECT_LT( (bezdiv.second.eval(t) - bez.eval(0.5+t/2.0)).norm(), 10e-14 )
            << "different at " << t << std::endl
            << "bez(t/2+0.5)=" << bez.eval(0.5+t/2.0).adjoint() << std::endl
            << "bezdiv2(t)  =" << bezdiv.second.eval(t).adjoint() << std::endl;
    }
}

// Test class
struct TestVec2 {
    double arr[2];
    double norm() const noexcept {
        return std::sqrt(arr[0]*arr[0] + arr[1]*arr[1]);
    }
    constexpr double& operator[](size_t i) noexcept{ return arr[i]; }
    constexpr double const& operator[](size_t i) const noexcept { return arr[i]; }
    constexpr TestVec2& operator+=(TestVec2 const& rhs) {
        arr[0] += rhs.arr[0];
        arr[1] += rhs.arr[1];
        return *this;
    }
    constexpr TestVec2& operator-=(TestVec2 const& rhs) {
        arr[0] -= rhs.arr[0];
        arr[1] -= rhs.arr[1];
        return *this;
    }
};

constexpr TestVec2 operator+(TestVec2 lhs, TestVec2 const& rhs)
{
    lhs += rhs;
    return lhs;
}

constexpr TestVec2 operator-(TestVec2 lhs, TestVec2 const& rhs)
{
    lhs -= rhs;
    return lhs;
}

constexpr TestVec2 operator*(double a, TestVec2 const& rhs)
{
    return {rhs[0]*a, rhs[1]*a};
}

bool operator==(TestVec2 const& lhs, TestVec2 const& rhs)
{
    return (lhs-rhs).norm() <= (lhs.norm() + rhs.norm())*10e-10;
}

TEST(Bezier, CubicWithMyVector)
{
    constexpr auto bez = Bezier<TestVec2,3>(
        TestVec2{0.0, 0.0},
        TestVec2{0.0, 1.0},
        TestVec2{1.0, 1.0},
        TestVec2{1.0, 0.0} );

    constexpr auto v = bez.eval(0.5);
    constexpr auto vshouldbe = TestVec2{1.0/8.0 + 3.0/8.0, 3.0/8.0 + 3.0/8.0};
    EXPECT_EQ(v, vshouldbe);

    constexpr auto bezdiv = bez.divide();
    for(double t = 0.0; t < 1.0; t += 0.1) {
        EXPECT_EQ(bezdiv.first.eval(t), bez.eval(t/2.0))
            << "different at " << t << std::endl;
        EXPECT_EQ(bezdiv.second.eval(t), bez.eval(0.5+t/2.0))
            << "different at " << t << std::endl;
    }
}

TEST(Bezier, ArbitraryDivision)
{
    constexpr auto bez = Bezier<TestVec2,3>(
        TestVec2{0.0, 0.0},
        TestVec2{0.0, 1.0},
        TestVec2{1.0, 1.0},
        TestVec2{1.0, 0.0} );

    for(double t0 = 0.1; t0 < 1.0; t0 += 0.1) {
        auto bezdiv = bez.divide(t0);
        for(double t = 0.0; t < 1.0; t += 0.1) {
            EXPECT_EQ(bezdiv.first.eval(t), bez.eval(t*t0))
                << "different at " << t << std::endl;
            EXPECT_EQ(bezdiv.second.eval(t), bez.eval(t0+t*(1.0-t0)))
                << "different at " << t << std::endl;
        }
    }
}

TEST(Bezier2D, HullFaceNone)
{
    // Nothing will happen for Bezier curve of degree 0 (i.e. just a point).
    auto hull1 = Bezier2D<0>(Eigen::Vector2d(0.0, 0.0)).getConvexHull();
    EXPECT_TRUE(hull1.empty());

    // Nothing will happen since there the convex hull is 0-dimensional.
    auto hull2 = Bezier2D<2>(
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(0.0, 0.0)
        ).getConvexHull();
    EXPECT_TRUE(hull2.empty());
}

TEST(Bezier2D, HullFaceLinear)
{
    // Test for lines
    constexpr size_t smp = 10;
    for(size_t i = 0; i < smp; ++i) {
        double t = (2*i*M_PI)/smp;
        Eigen::Matrix2d rmat;
        rmat << cos(t), -sin(t), sin(t), cos(t);
        Eigen::Vector2d p0 = rmat*Eigen::Vector2d(-0.5, 0.0);
        Eigen::Vector2d p1 = rmat*Eigen::Vector2d(1.0, 0.0);
        auto bndpath = Bezier2D<1>(p0, p1).getConvexHull();

        ASSERT_GE(bndpath.size(), 2)
            << "i=" << i << ", t=" << t << std::endl;
        auto itr = std::find(bndpath.begin(), bndpath.end(), p0);
        EXPECT_NE(itr, bndpath.end())
            << "p0 not found: " << p0.adjoint() << std::endl;
        itr = std::find(bndpath.begin(), bndpath.end(), p1);
        EXPECT_NE(itr, bndpath.end())
            << "p1 not found: " << p1.adjoint() << std::endl;
        if(bndpath.size() > 2) {
            ADD_FAILURE()
                << "i=" << i << ", t=" << t << std::endl
                << "p0= " << p0.adjoint() << std::endl
                << "p1= " << p1.adjoint() << std::endl
                << bndpath[2].adjoint() << std::endl;
        }
    }
}

TEST(Bezier2D, HullFaceHigher)
{
    // Test for a higher degree.
    auto bez2d = Bezier2D<8>(
        Eigen::Vector2d(0.0, 0.0), // 0
        Eigen::Vector2d(1.0, 0.5), // 1
        Eigen::Vector2d(2.0, 0.0), // 2
        Eigen::Vector2d(1.5, 1.0), // 3
        Eigen::Vector2d(2.0, 2.0), // 4
        Eigen::Vector2d(1.0, -1.0),// 5
        Eigen::Vector2d(0.0, 2.0), // 6
        Eigen::Vector2d(0.5, 1.0), // 7
        Eigen::Vector2d(2.0, 0.0)  // 8
        );
    auto bndpath = bez2d.getConvexHull();

    ASSERT_EQ(bndpath.size(), 5);
    EXPECT_EQ(bndpath[0], bez2d.get<5>())
        << bndpath[0].adjoint() << std::endl;
    EXPECT_EQ(bndpath[1], bez2d.get<2>())
        << bndpath[1].adjoint() << std::endl;
    EXPECT_EQ(bndpath[2], bez2d.get<4>())
        << bndpath[2].adjoint() << std::endl;
    EXPECT_EQ(bndpath[3], bez2d.get<6>())
        << bndpath[3].adjoint() << std::endl;
    EXPECT_EQ(bndpath[4], bez2d.get<0>())
        << bndpath[4].adjoint() << std::endl;
}

TEST(Bezier2D, IntersectionLinLin)
{
    constexpr size_t max_smp = 20;
    for(size_t i = 0; i < max_smp; ++i) {
        double t = 2*i*M_PI/max_smp;
        Eigen::Matrix2d mat;
        mat << cos(t), -sin(t), sin(t), cos(t);

        auto bez1 = Bezier2D<1>(
            mat*Eigen::Vector2d(-1.0, 0.0),
            mat*Eigen::Vector2d(1.0, 0.0) );
        auto bez2 = Bezier2D<1>(
            mat*Eigen::Vector2d(0.8, 0.6),
            mat*Eigen::Vector2d(0.8, -0.6) );

        auto params = intersect(bez1, bez2);
        ASSERT_GE(params.size(), 1) << "t=" << t;
        EXPECT_LT(std::abs(params[0].first - 0.9), 10e-10);
        EXPECT_LT(std::abs(params[0].second - 0.5), 10e-10);
        if (params.size() > 1) {
            ADD_FAILURE()
                << "i=" << i << ", t=" << t << std::endl
                << "(" << params[0].first << ", " << params[0].second << ")"
                << std::endl
                << "(" << params[1].first << ", " << params[1].second << ")"
                << std::endl;
        }
    }
}

TEST(Bezier2D, IntersectionQQ)
{
    constexpr size_t max_smp = 20;
    for(size_t i = 0; i < max_smp; ++i) {
        double t = 2*i*M_PI/max_smp;
        Eigen::Matrix2d mat;
        mat << cos(t), -sin(t), sin(t), cos(t);

        auto bez1 = Bezier2D<2>(
            mat*Eigen::Vector2d(-0.5, -1.0),
            mat*Eigen::Vector2d(1.0, 0.0),
            mat*Eigen::Vector2d(-0.5, 1.0) );
        auto bez2 = Bezier2D<2>(
            mat*Eigen::Vector2d(0.5, -1.0),
            mat*Eigen::Vector2d(-1.0, 0.0),
            mat*Eigen::Vector2d(0.5, 1.0) );

        auto params = intersect(bez1, bez2);
        ASSERT_GE(params.size(), 2) << "t=" << t;
        EXPECT_LT(
            std::abs((mat.adjoint()*bez1.eval(params[0].first))(0)),
            10e-10);
        EXPECT_LT(
            std::abs((mat.adjoint()*bez2.eval(params[1].first))(0)),
            10e-10);
        if (params.size() > 2) {
            ADD_FAILURE()
                << "i=" << i << ", t=" << t << std::endl
                << "(" << params[0].first << ", " << params[0].second << ")"
                << std::endl
                << "(" << params[1].first << ", " << params[1].second << ")"
                << std::endl;
        }
    }
}
