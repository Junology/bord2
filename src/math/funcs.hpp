/*!
 * \file funcs.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 9, 2019: created
 */

#pragma once

#include <utility>
#include <vector>

namespace bord2 {

/*******************************
 * \section Equation solvers
 *******************************/

/*! Find solutions of a linear equation ax+b==0 in a given range (as an open interval).
 * \return The solution x in the range if exists; If a==b==0.0, then the ends of the range are returned.
 */
std::vector<double> solveLinearRange(double a, double b, std::pair<double,double> const &range);

/*! Find solutions of a quadratic equation ax^2+bx+c==0 in a given range (as an open interval).
 * If either of coefficients is nan or infinity, then the empty vector is returned.
 * \param range The range where solution is searched; first: lower bound, second: upper bound.
 * \return the list of solution and the flag that is true precisely when the discriminant is zero.
 */
std::pair<std::vector<double>,bool> solveQuadRange(double a, double b, double c, std::pair<double,double> const &range);

} // end namespace bord2
