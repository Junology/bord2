/*!
 * \file QuadraticCurve.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 9, 2019: created
 */

#pragma once

#include <Eigen/Dense>

#include "funcs.hpp"
#include "AffHypPlane.hpp"

/*!
 * The class represents a quadratic curve on the Euclidean plane R^2.
 */
class QuadraticCurve
{
private:
    Eigen::Matrix3d m_M;

public:
    //! Constructor, which accepts the coefficients of the associated quadratic form; namely,
    //! ax^2+by^2+cz^2+dxy+exz+fyz
    QuadraticCurve(double a, double b, double c, double d, double e, double f)
    {
        m_M <<
            a, d/2, e/2,
            d/2, b, f/2,
            e/2, f/2, c;
    }

    ~QuadraticCurve() = default;

    //! Evaluate the defining function; i.e. q(x,y,1) for the associated quadratic form q.
    double evaluate(double x0, double y0) const
    {
        Eigen::Vector3d v{x0, y0, 1.0};

        return v.adjoint()*m_M*v;
    }

    //! Generate an affine line tangent to the curve q(x,y,1)=q(x0,y0,1).
    AffHypPlane<2> tangent(double x0, double y0) const
    {
        Eigen::Vector2d normal = (m_M.block<2,3>(0,0) * Eigen::Vector3d(x0, y0, 1.0));
        return AffHypPlane<2>(normal, Eigen::Vector2d(x0, y0));
    }

    //! Compute the parameter t (0<t<1) at which the segment (1-t)p0 + t p1 intersects with the curve.
    std::vector<double> intersectParams(Eigen::Vector2d const &p0, Eigen::Vector2d const &p1)
    {
        // Vector of the direction of p1 from p0
        Eigen::Vector2d v = p1-p0;
        Eigen::Vector3d p0E(p0(0), p0(1), 1);

        // Coefficients of the equation we have to solve
        double ca = (v.adjoint() * m_M.block<2,2>(0,0) * v)(0);
        double cb = 2.0 * (v.adjoint() * (m_M.block<2,3>(0,0) * p0E))(0);
        double cc = (p0E.adjoint() * m_M * p0E)(0);

        return bord2::solveQuadRange(ca, cb, cc, {0.0, 1.0}).first;
    }

    //! Compute the parameter t (0<t<1) at which the segment (1-t)(x0,y0)+t(x1,y1) intersects with the curve.
    std::vector<double> intersectParams(double x0, double y0, double x1, double y1)
    {
        return intersectParams(Eigen::Vector2d(x0,y0), Eigen::Vector2d(x1,y1));
    }
};
