/*!
 * \file funcs.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 9, 2019: created
 */

#include "funcs.hpp"

#include <iostream> // For debug
#include <cmath>

namespace bord2 {;

//! Find solutions of a linear equation ax+b==0 in a given range.
//! \return The solution x in the range if exists; If a==b==0.0, then the ends of the range are returned.
std::vector<double> solveLinearRange(double a, double b, std::pair<double,double> const &range)
{
    // range check
    if (range.first > range.second)
        return std::vector<double>();

    // floating point exception check
    if (!std::isfinite(a) || !std::isfinite(b))
        return std::vector<double>();

    // trivial cases.
    if (a == 0.0) {
        if (b == 0.0)
            return std::vector<double>({range.first, range.second});
        else
            return std::vector<double>();
    }

    // normalize so that the coefficient a is positive; so the function becomes strictly increasing.
    if (std::signbit(a)) {
        a = -a;
        b = -b;
    }

    // sign change doesn't happen in the range.
    if (a*range.first+b > 0.0 || a*range.second+b < 0.0)
        return std::vector<double>();

    double x = -b/a;

    // Find a solution by Newton's method.
    while(a*x+b != 0.0) {
        double x1 = x - (a*x+b)/a;

        if (x == x1)
            break;

        x = x1;
    }

    return {x};
}

//! Find solutions of a quadratic equation ax^2+bx+c==0 in a given range.
//! If either of coefficients is nan or infinity, then the empty vector is returned.
//! \param range The range where solution is searched; first: lower bound, second: upper bound.
std::pair<std::vector<double>,bool> solveQuadRange(double a, double b, double c, std::pair<double,double> const &range)
{
    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c))
        return {std::vector<double>(), false};

    // Discriminant
    double discr = b*b - 4*a*c;

    // If the discriminant is negative, return the empty set.
    if (discr < 0.0)
        return {std::vector<double>(), false};

    // If the equation is actually linear, use the appropriate one.
    if (a==0) {
        return {solveLinearRange(b, c, range), false};
    }

    std::vector<double> result;

    // Find the first, or bigger, solution.
    double x = (-b + std::sqrt(discr))/(2*a);
    double y = a*x*x + b*x + c;

    while(y != 0.0) {
        if (2*a*x+b == 0.0)
            break;

        double x1 = x - y/(2*a*x+b);

        if (x == x1)
            break;

        x = x1;
        y = a*x*x + b*x + c;
    }

    if (range.first < x && x < range.second) {
        /* Debug
        std::cout << __FILE__ << ":" <<  __LINE__ << std::endl;
        std::cout << range.first << " < " << x << " < " << range.second << std::endl;
        // */
        result.emplace_back(x);
    }

    // If the discriminant is zero, stop here.
    if (discr == 0.0)
        return {std::move(result), true};

    // Otherwise, find the second, or smaller, solution.
    x = (-b - std::sqrt(discr))/(2*a);
    y = a*x*x + b*x + c;

    while(y != 0.0) {
        if (2*a*x+b == 0.0)
            break;

        double x1 = x - y/(2*a*x+b);

        if (x == x1)
            break;

        x = x1;
        y = a*x*x + b*x + c;
    }

    if (range.first < x && x < range.second) {
        /* Debug
        std::cout << __FILE__ << ":" <<  __LINE__ << std::endl;
        std::cout << a << "x^2 + " << b << "x + " << c << std::endl;
        std::cout << "discriminant = " << discr << std::endl;
        std::cout << range.first << " < " << x << " < " << range.second << std::endl;
        // */
        result.emplace_back(x);
    }

    return {std::move(result), false};
}

} // end namespace bord2
