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

TEST(Bezier, LinearClip)
{
    Bezier<Eigen::Vector2d,1> bez(
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 2.0) );

    constexpr size_t smp = 10;
    constexpr double dur = 1.0/smp;
    for(size_t i = 0; i < smp; ++i) {
        for(size_t j = i+1; j < smp; ++j) {
            auto bezclip = bez.clip(i*dur, j*dur);
            EXPECT_LT( (bezclip.get<0>()-bez.eval(i*dur)).norm(), 10e-10);
            EXPECT_LT( (bezclip.get<1>()-bez.eval(j*dur)).norm(), 10e-10);        }
    }
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

TEST(Bezier, QuadClip)
{
    Bezier<Eigen::Vector2d,2> bez(
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(0.0, 2.0));

    constexpr size_t smp = 10;
    constexpr double dur = 1.0/smp;
    for(size_t i = 0; i < smp; ++i) {
        for(size_t j = i+1; j < smp; ++j) {
            auto bezclip = bez.clip(i*dur, j*dur);
            EXPECT_LT( (bezclip.get<0>()-bez.eval(i*dur)).norm(), 10e-10);
            EXPECT_LT( (bezclip.eval(0.5)-bez.eval(i*dur/2 + j*dur/2)).norm(), 10e-10);
            EXPECT_LT( (bezclip.get<2>()-bez.eval(j*dur)).norm(), 10e-10);        }
    }
}

TEST(Bezier, Range)
{
    Bezier<Eigen::Vector2d,3> bez = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(1.0, 0.0),
        Eigen::Vector2d(0.0, 1.0),
        Eigen::Vector2d(1.0, 1.0)
    };

    std::vector<Eigen::Vector2d> vec;

    for(auto& pt : bez) {
        vec.push_back(pt);
    }

    ASSERT_EQ(vec.size(), 4);
    EXPECT_EQ(vec[0], Eigen::Vector2d(0.0, 0.0));
    EXPECT_EQ(vec[1], Eigen::Vector2d(1.0, 0.0));
    EXPECT_EQ(vec[2], Eigen::Vector2d(0.0, 1.0));
    EXPECT_EQ(vec[3], Eigen::Vector2d(1.0, 1.0));

    Bezier<Eigen::Vector2d, 3> const cbez = bez;

    vec.clear();

    for(auto& pt : cbez) {
        vec.push_back(pt);
    }

    ASSERT_EQ(vec.size(), 4);
    EXPECT_EQ(vec[0], Eigen::Vector2d(0.0, 0.0));
    EXPECT_EQ(vec[1], Eigen::Vector2d(1.0, 0.0));
    EXPECT_EQ(vec[2], Eigen::Vector2d(0.0, 1.0));
    EXPECT_EQ(vec[3], Eigen::Vector2d(1.0, 1.0));
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

bool operator!=(TestVec2 const& lhs, TestVec2 const& rhs)
{
    return !(lhs==rhs);
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
    constexpr size_t smp = 10;
    constexpr double dur = 1.0/smp;
    for(size_t i = 0; i < smp; ++i) {
        double t = i*dur;
        EXPECT_EQ(bezdiv.first.eval(t), bez.eval(t/2.0))
            << "different at " << t << std::endl;
        EXPECT_EQ(bezdiv.second.eval(t), bez.eval(0.5+t/2.0))
            << "different at " << t << std::endl;
    }

    constexpr auto bezclip = bez.clip(0.3, 0.7);
    for(size_t i = 0; i < smp; ++i) {
        double t = i*dur;
        EXPECT_EQ(bezclip.eval(t), bez.eval(0.3+t*0.4));
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

TEST(Bezier, Convert)
{
    struct Rotate90 {
        constexpr TestVec2 operator()(TestVec2 src) const noexcept {
            return TestVec2{-src.arr[1], src.arr[0]};
        }
    };

    constexpr auto bez = Bezier<TestVec2,3>(
        TestVec2{0.0, 0.0},
        TestVec2{0.0, 1.0},
        TestVec2{1.0, 1.0},
        TestVec2{1.0, 0.0} );
    constexpr auto bez90 = bez.convert(Rotate90());
    constexpr size_t smp = 10;
    constexpr double dur = 1.0/smp;
    for(size_t i = 0; i < smp; ++i) {
        double t = i*dur;
        auto pt = bez.eval(t);
        auto pt90 = bez90.eval(t);
        auto pt90shouldbe = TestVec2{-pt.arr[1], pt.arr[0]};
        EXPECT_EQ(pt90, pt90shouldbe);
        if(i > 0)
            EXPECT_NE(pt90, pt);
    }
};

TEST(Bezier, Variant)
{
    constexpr Bezier<TestVec2,2> bez2(
        TestVec2{0.0, 0.0},
        TestVec2{1.0, 0.0},
        TestVec2{1.0, 1.0});
    constexpr BezierVariant<TestVec2,0,1,2,3> bezvar(std::integral_constant<size_t,2>(), bez2);

    std::vector<TestVec2> vs;
    for(auto pt : bezvar) {
        vs.push_back(pt);
    }
    ASSERT_EQ(vs.size(), 3);
    EXPECT_EQ(vs[0], (TestVec2{0.0, 0.0}));
    EXPECT_EQ(vs[1], (TestVec2{1.0, 0.0}));
    EXPECT_EQ(vs[2], (TestVec2{1.0, 1.0}));

    constexpr size_t smp = 10;
    constexpr double dur = 1.0/smp;

    // Evaluation test
    for(size_t i = 0; i <= smp; ++i) {
        double t = i*dur;
        EXPECT_EQ(bez2.eval(t),
                  bezvar.eval(t));
    }

    // Division test
    constexpr auto bez2div = bez2.divide();
    constexpr auto bezvardiv = bezvar.divide();
    for(size_t i = 0; i <= smp; ++i) {
        double t = i*dur;
        EXPECT_EQ(bez2div.first.eval(t),
                  bezvardiv.first.eval(t));
        EXPECT_EQ(bez2div.second.eval(t),
                  bezvardiv.second.eval(t));
    }

    // Clip test
    for(size_t i = 0; i < smp; ++i) {
        for(size_t j = i+1; j <= smp; ++j) {
            auto bez2clip = bez2.clip(i*dur, j*dur);
            auto bezvarclip = bezvar.clip(i*dur, j*dur);
            for(size_t k = 0; k <= smp; ++k) {
                double t = k*dur;
                EXPECT_EQ(bez2clip.eval(t), bezvarclip.eval(t));
            }
        }
    }

    // Convert test
    struct Rotate90 {
        constexpr TestVec2 operator()(TestVec2 src) const noexcept {
            return TestVec2{-src.arr[1], src.arr[0]};
        }
    };
    constexpr auto bez2_90 = bez2.convert(Rotate90());
    constexpr auto bezvar_90 = bezvar.convert(Rotate90());
    for(size_t i = 0; i <= smp; ++i) {
        double t = i*dur;
        EXPECT_EQ(bez2_90.eval(t), bezvar_90.eval(t));
    }

    // Assignment
    constexpr Bezier<TestVec2,3> bez3(
        TestVec2{0.0, 0.0},
        TestVec2{1.0, 0.0},
        TestVec2{0.0, 1.0},
        TestVec2{1.0, 1.0});
    auto bezvar3 = bezvar;
    bezvar3.assign(std::integral_constant<size_t,3>(), bez3);
    vs.clear();
    for(auto pt : bezvar3) {
        vs.push_back(pt);
    }
    ASSERT_EQ(vs.size(), 4);
    EXPECT_EQ(vs[0], (TestVec2{0.0, 0.0}));
    EXPECT_EQ(vs[1], (TestVec2{1.0, 0.0}));
    EXPECT_EQ(vs[2], (TestVec2{0.0, 1.0}));
    EXPECT_EQ(vs[3], (TestVec2{1.0, 1.0}));
}

TEST(Bezier2D, IntersectionLinLin)
{
    constexpr size_t max_smp = 20;
    for(size_t i = 0; i < max_smp; ++i) {
        double t = 2*i*M_PI/max_smp;
        Eigen::Matrix2d mat;
        mat << cos(t), -sin(t), sin(t), cos(t);

        auto bez1 = Bezier2D<Bezier<Eigen::Vector2d,1>>(
            mat*Eigen::Vector2d(-1.0, 0.0),
            mat*Eigen::Vector2d(1.0, 0.0) );
        auto bez2 = Bezier2D<Bezier<Eigen::Vector2d,1>>(
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

        auto bez1 = Bezier2D<Bezier<Eigen::Vector2d,2>>(
            mat*Eigen::Vector2d(-0.5, -1.0),
            mat*Eigen::Vector2d(1.0, 0.0),
            mat*Eigen::Vector2d(-0.5, 1.0) );
        auto bez2 = Bezier2D<Bezier<Eigen::Vector2d,2>>(
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

TEST(Bezier2D, IntersectionVarQL)
{
    constexpr size_t max_smp = 20;
    for(size_t i = 0; i < max_smp; ++i) {
        double t = 2*i*M_PI/max_smp;
        Eigen::Matrix2d mat;
        mat << cos(t), -sin(t), sin(t), cos(t);

        auto bez1 = Bezier2D<BezierVariant<Eigen::Vector2d,0,1,2>>(
            std::integral_constant<size_t,2>(),
            mat*Eigen::Vector2d(-0.5, -1.0),
            mat*Eigen::Vector2d(1.0, 0.0),
            mat*Eigen::Vector2d(-0.5, 1.0) );
        auto bez2 = Bezier2D<Bezier<Eigen::Vector2d,2>>(
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
