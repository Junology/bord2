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

TEST(Bezier2D, EachFaceNone)
{
    // Nothing will happen for Bezier curve of degree 0 (i.e. just a point).
    Bezier2D<0>(Eigen::Vector2d(0.0, 0.0)).forEachFace(
        [](Eigen::Vector2d const& pt, Eigen::Vector2d const& nv) {
            EXPECT_TRUE(false);
        } );

    // Nothing will happen since there the convex hull is 0-dimensional.
    Bezier2D<2>(
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(0.0, 0.0)
        ).forEachFace(
            [](Eigen::Vector2d const& pt, Eigen::Vector2d const& nv) {
                EXPECT_TRUE(false);
            } );
}

TEST(Bezier2D, EachFaceLinear)
{
    // Test for lines
    std::vector<std::array<Eigen::Vector2d,2>> faces;
    bool flag = Bezier2D<1>(
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 1.0)
        ).forEachFace(
            [&faces](Eigen::Vector2d const& pt, Eigen::Vector2d const& nv) {
                faces.push_back({pt,nv});
            } );

    ASSERT_TRUE(flag);
    ASSERT_EQ(faces.size(), 2);
    EXPECT_EQ(faces[0][0], Eigen::Vector2d(0.0, 0.0));
    EXPECT_LT((faces[0][1]-Eigen::Vector2d(sqrt(2)/2,-sqrt(2)/2)).norm(), 10e-10);
    EXPECT_EQ(faces[1][0], Eigen::Vector2d(1.0, 1.0));
    EXPECT_LT((faces[1][1]-Eigen::Vector2d(-sqrt(2)/2,sqrt(2)/2)).norm(), 10e-10);
}

TEST(Bezier2D, EachFaceHigher)
{
    // Test for a higher degree.
    auto bez2d = Bezier2D<8>(
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.5),
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(1.5, 1.0),
        Eigen::Vector2d(2.0, 2.0),
        Eigen::Vector2d(1.0, -1.0),
        Eigen::Vector2d(0.0, 2.0),
        Eigen::Vector2d(0.5, 1.0),
        Eigen::Vector2d(2.0, 0.0)
        );
    std::vector<std::array<Eigen::Vector2d,2>> faces{};
    bool flag = bez2d.forEachFace(
        [&faces](Eigen::Vector2d const& pt, Eigen::Vector2d const& nv) {
            faces.push_back({pt,nv});
        });
    ASSERT_TRUE(flag);
    ASSERT_EQ(faces.size(), 5);
    EXPECT_EQ(faces[0][0], Eigen::Vector2d(0.0, 0.0));
    EXPECT_LT((faces[0][1]-Eigen::Vector2d(-sqrt(2)/2,-sqrt(2)/2)).norm(), 10e-10);
    EXPECT_EQ(faces[1][0], Eigen::Vector2d(1.0, -1.0));
    EXPECT_LT((faces[1][1]-Eigen::Vector2d(sqrt(2)/2,-sqrt(2)/2)).norm(), 10e-10);
    EXPECT_EQ(faces[2][0], Eigen::Vector2d(2.0, 0.0));
    EXPECT_LT((faces[2][1]-Eigen::Vector2d(1,0)).norm(), 10e-10);
    EXPECT_EQ(faces[3][0], Eigen::Vector2d(2.0, 2.0));
    EXPECT_LT((faces[3][1]-Eigen::Vector2d(0,1)).norm(), 10e-10);
    EXPECT_EQ(faces[4][0], Eigen::Vector2d(0.0, 2.0));
    EXPECT_LT((faces[4][1]-Eigen::Vector2d(-1,0)).norm(), 10e-10);
}

TEST(Bezier2D, IntersectionLinLin)
{
    for(double t = 0.0; t < 2*M_PI; t += M_PI/10) {
        Eigen::Matrix2d mat;
        mat << cos(t), -sin(t), sin(t), cos(t);

        auto bez1 = Bezier2D<1>(
            mat*Eigen::Vector2d(-1.0, 0.0),
            mat*Eigen::Vector2d(1.0, 0.0) );
        auto bez2 = Bezier2D<1>(
            mat*Eigen::Vector2d(0.8, 0.6),
            mat*Eigen::Vector2d(0.8, -0.6) );
        EXPECT_TRUE(bez1.hasHullIntersection(bez2));
        auto params = intersect(bez1, bez2);
        ASSERT_EQ(params.size(), 1) << "t=" << t;
        EXPECT_LT(std::abs(params[0].first - 0.9), 10e-10);
        EXPECT_LT(std::abs(params[0].second - 0.5), 10e-10);
    }
}
