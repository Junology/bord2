#include <gtest/gtest.h>

#include "math/solvers.hpp"

TEST(TestSolvers, Binary)
{
    using namespace bord2;
    constexpr double pi = 3.14159265358979323846264338327950288;
    using ftype = double (*)(double);

    // Success
    // Pi from below.
    double pi2 = binSolve<double,ftype>(3.0, 22.0/7.0, std::sin);
    EXPECT_LT(std::abs(pi2 - pi), 1e-14);
    EXPECT_LT(std::abs(std::sin(pi2)), 1e-14);
    // Pi from above.
    double x0 = binSolve<double,ftype>(-0.5,0.5, std::sin);
    EXPECT_LT(std::abs(x0), 1e-14);
    // Trivial case
    double x1 = binSolve(0.0, 10.0, [](double){return 0.0;});
    EXPECT_EQ(x1, 0.0);
    x1 = binSolve(0.0, 10.0, [](double x){return x;});
    EXPECT_EQ(x1, 0.0);
    x1 = binSolve(0.0, 10.0, [](double x){return 10.0-x;});
    EXPECT_EQ(x1, 10.0);

    // Failure
    double x2 = binSolve(-2.0, 2.0, [](double x){return x*x-1;});
    EXPECT_TRUE(std::isnan(x2)) << x2;
}

TEST(TestSolvers, Newton)
{
    using namespace bord2;
    constexpr double pi = 3.14159265358979323846264338327950288;
    using ftype = double (*)(double);

    // Success
    // Pi from below.
    double pi2 = newton<double,ftype,ftype>(3.0, std::sin, std::cos);
    EXPECT_LT(std::abs(pi2 - pi), 1e-14);
    EXPECT_LT(std::abs(std::sin(pi2)), 1e-14);
    // Pi from above.
    double pi3 = newton<double,ftype,ftype>(22.0/7.0, std::sin, std::cos);
    EXPECT_LT(std::abs(pi3 - pi), 1e-14);
    EXPECT_LT(std::abs(std::sin(pi3)), 1e-14);
    // The peak of a parabola.
    double x
        = newton(0.0, [](double x){return x*x;}, [](double x){return 2*x;});
    EXPECT_EQ(x, 0.0);

    // Failure
    double x1
        = newton(0.0, [](double){return 1.0;}, [](double){return 0.0;});
    EXPECT_TRUE(std::isinf(x1)) << x1;
    double x2
        = newton(0.1, [](double x){return x*x+1;}, [](double x){return 2*x;});
    EXPECT_TRUE(std::isnan(x2)) << x2;
}

TEST(TestSolvers, Linear)
{
    using namespace bord2;

    // Solvable cases
    for(size_t i = 0; i < 10; ++i) {
        auto res = solveLinear<double>(2.0*i-9.0, (2.0*i-9.0)*3.0);
        EXPECT_LT(std::abs(res+3.0), 1e-14);
    }

    // Critical cases
    EXPECT_EQ(solveLinear<double>(0.0, 0.0), 0.0);
    EXPECT_TRUE(std::isinf(solveLinear<double>(0.0, 1.0)));
    EXPECT_TRUE(std::isnan(solveLinear<double>(0.0, std::numeric_limits<double>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(solveLinear<double>(std::numeric_limits<double>::quiet_NaN(), 0.0)));
}

TEST(TestSolvers, Quad)
{
    using namespace bord2;

    // Generic cases
    for(double t = 0.0; t < 10.0; t+=1.0) {
        auto a = 2.0*t - 9.0;
        auto res = solveQuad<double>(a, -a*(2*t-10.0), a*t*(t-10.0));
        EXPECT_LT(std::abs(res.first - (t-10.0)), 1e-14)
            << t << ":" << res.first << std::endl;
        EXPECT_LT(std::abs(res.second - t), 1e-14)
            << t << ":" << res.second << std::endl;
    }

    // Double root
    for(double t = 0.0; t < 10.0; t+=1.0) {
        auto a = 2.0*t - 9.0;
        auto res = solveQuad<double>(a, -2.0*a*(t-1.0), a*(t-1.0)*(t-1.0));
        EXPECT_LT(std::abs(res.first - (t-1.0)), 1e-14);
        EXPECT_EQ(res.first, res.second);
    }

    // Negative discriminant
    for(double t = 0.0; t < 10.0; t+=1.0) {
        auto a = 2.0*t - 9.0;
        auto c = std::signbit(a) ? -1.0 : 1.0;
        auto res = solveQuad<double>(a, 2.0*a*t, a*t*t + c);
        EXPECT_TRUE(std::isnan(res.first))
            << t << ":" << res.first << std::endl;
        EXPECT_TRUE(std::isnan(res.second))
            << t << ":" << res.second << std::endl;
    }

    // Degenerate to linear
    auto res1 = solveQuad<double>(0.0, 2.0, 4.0);
    EXPECT_LT(std::abs(res1.first + 2.0), 1e-14);
    EXPECT_TRUE(std::isinf(res1.second));
}
