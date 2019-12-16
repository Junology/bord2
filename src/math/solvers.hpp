/*!
 * \file solvers.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 16, 2019: created
 */

#pragma once

#include <type_traits>
#include <limits>
#include <utility>
#include <cmath>

//#include <iostream> // Debug

namespace bord2 {

/*! Binary search of a solution for f(x)==0.
 * \param{x0, x1} Boundary of the range where we find the solution.
 * \param f The function whose zero we want to find.
 * \return The solution, which may be
 * - NaN if f(x0) and f(x1) are both non-zero and have the same sign.
 */
template <class T, class F>
inline constexpr T binSolve(T x0, T x1, F f)
{
    auto y0 = f(x0);
    auto y1 = f(x1);

    if (y0 == 0)
        return x0;

    if (y1 == 0)
        return x1;

    if (std::signbit(y0) == std::signbit(y1))
        return std::numeric_limits<T>::quiet_NaN();

    while(x0 != x1) {
        T mid = (x0+x1)*0.5;

        if(mid == x0 || mid == x1)
            break;

        auto ymid = f(mid);

        if(ymid == 0)
            return mid;

        if(std::signbit(ymid) == std::signbit(y0)) {
            x0 = mid;
            y0 = ymid;
        }
        else {
            x1 = mid;
            y1 = ymid;
        }
    }

    return x0;
}

/*! Newton method to find a solution of f(x)==0.
 * \param x The initial value where the method begins.
 * \param f The function whose zero we want to find.
 * \param df The derivative of f.
 * \return The solution, which may be
 * - inf if the method reaches to a point where df==0 before a solution found;
 * - NaN if the approximation becomes worse than that in the previous step.
 */
template <class T, class F, class G>
inline constexpr T newton(T x, F f, G df)
{
    auto y = f(x);

    while(y != 0) {
        // If the derivative gets zero, teminate with inf.
        if(df(x) == 0)
            return std::numeric_limits<T>::infinity();

        T x1 = x - y/df(x);

        // Already the best approximation.
        if(x==x1)
            break;

        auto y1 = f(x1);

        // The value is no more improved.
        if (y == y1)
            break;

        // When the sign of the value is changed, switch to the binary search.
        if (y == -y1)
            return binSolve(x,x1,std::forward<F>(f));

        // If the result gets worse, terminate with NaN;
        if(std::abs(y1) > std::abs(y))
            return std::numeric_limits<T>::quiet_NaN();

        x = x1;
        y = y1;
    }

    return x;
}

/*! Find solutions of a linear equation ax+b==0.
 * \return The solution x if exists. In the other cases, it returns
 * - HUGE_VAL if a==0 and b!=0;
 * - 0 if a==b==0;
 * - NaN if !(std::isfinite(a) && std::isfinite(b)).
 */
template <
    class T,
    class = std::enable_if_t<
        std::numeric_limits<T>::has_infinity
        && std::numeric_limits<T>::has_quiet_NaN
        >
    >
constexpr T solveLinear(T a, T b)
{
    // floating point exception check
    if (!std::isfinite(a) || !std::isfinite(b))
        return std::numeric_limits<T>::quiet_NaN();

    // trivial cases.
    if (a == 0) {
        if (b == 0)
            return 0;
        else
            return std::numeric_limits<T>::infinity();
    }

    // Find a solution by Newton's method.
    return newton(-b/a, [&a,&b](T x){return a*x+b;}, [&a](T){return a;});
}

/*! Find solutions of a quadratic equation ax^2+bx+c==0.
 * \return the pair (s1,s2) of the solutions, which is
 * - (NaN,NaN) if either of a, b, or c is not finite;
 * - (solveLinear(b,c),inf) if a == 0;
 * - exactly equal, i.e. s1==s2 without error, if the equation has a multiplicity;
 * - (s1,s2) with s1 < s2 if the equation has two different solutions;
 * - (NaN,NaN) if no solution; i.e. the equation has negative discriminant.
 */
template <
    class T,
    class = std::enable_if_t<
        std::numeric_limits<T>::has_infinity
        && std::numeric_limits<T>::has_quiet_NaN
        >
    >
constexpr std::pair<T,T> solveQuad(T a, T b, T c)
{
    constexpr std::pair<T,T> failpair{
        std::numeric_limits<T>::quiet_NaN(),
            std::numeric_limits<T>::quiet_NaN() };

    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c)) {
        return std::move(failpair);
    }

    // Discriminant
    T discr = b*b - 4*a*c;

    // If the discriminant is negative, return the empty set.
    if (std::signbit(discr))
        return std::move(failpair);

    // If the equation is actually linear, use the appropriate one.
    if (a==static_cast<T>(0))
        return {solveLinear(b,c), std::numeric_limits<T>::infinity()};

    // Verify the leading coefficient to be positive.
    if (std::signbit(a)) {
        a = -a;
        b = -b;
        c = -c;
    }

    T smaller = newton(
        (-b-sqrt(discr))/(2*a),
        [&a,&b,&c](T x){return a*x*x + b*x + c;},
        [&a,&b](T x){return 2*a*x + b;}
        );
    T larger = (discr == static_cast<T>(0)) ? smaller
        : newton(
            (-b+sqrt(discr))/(2*a),
            [&a,&b,&c](T x){return a*x*x + b*x + c;},
            [&a,&b](T x){return 2*a*x + b;}
            );

    return {smaller, larger};
}

} // end namespace bord2
