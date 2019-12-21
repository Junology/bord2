/*!
 * \file QuadraticCurve.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 9, 2019: created
 */

#pragma once

#include <memory>
#include <array>
#include <vector>
#include <Eigen/Dense>

#include "AffHypPlane.hpp"
#include "Bezier.hpp"

/*!
 * The class represents a quadratic curve on the Euclidean plane R^2.
 */
class QuadraticCurve
{
private:
    Eigen::Matrix3d m_M;

public:
    static constexpr double threshold = 10e-14;

    //! Constructor, which accepts the coefficients of the associated quadratic form; namely,
    //! ax^2+by^2+cz^2+dxy+exz+fyz
    QuadraticCurve(double a, double b, double c, double d, double e, double f);

    QuadraticCurve(QuadraticCurve const &) = default;
    QuadraticCurve(QuadraticCurve &&) = default;

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
        Eigen::Vector2d df = m_M.block<2,3>(0,0) * Eigen::Vector3d(x0, y0, 1.0);
        return AffHypPlane<2>(df, Eigen::Vector2d(x0, y0));
    }

    AffHypPlane<2> tangent(Eigen::Vector2d const &p) const
    {
        return tangent(p(0), p(1));
    }

    //! Compute the curvature vector at a given point (x0,y0).
    Eigen::Vector2d curvature(double x0, double y0) const;

    Eigen::Vector2d curvature(Eigen::Vector2d const &p) const
    {
        return curvature(p(0),p(1));
    }

    /*! If the curve is hyperbolic, compute an affine line such that
     * - it divides the plain into two components each of which contains one of the two connected component of the hyperbola;
     * - the curve is invariant under the reflection associated to the line.
     * If the curve is not hyperbolic, i.e. it is elliptic, parabolic, or degenerate, then nullptr is returned.
     */
    std::unique_ptr<AffHypPlane<2> > divAxis() const;

    //! Compute the parameter t (0<=t<=1) at which the segment (1-t)p0 + t p1 intersects with the curve.
    std::vector<double> intersectParams(Eigen::Vector2d const &p0, Eigen::Vector2d const &p1);

    //! Compute the parameter t (0<=t<=1) at which the segment (1-t)(x0,y0)+t(x1,y1) intersects with the curve.
    std::vector<double> intersectParams(double x0, double y0, double x1, double y1)
    {
        return intersectParams(Eigen::Vector2d(x0,y0), Eigen::Vector2d(x1,y1));
    }

    /*! Approximate the intersection with a given triangle by cubic Bezier curves.
     * \param p The array of vertices that span a triangle.
     * \return The pair of
     * - the set of Bezier curves each of which approximates a connected component of the intersection;
     * - the flag indicating if the returned list is reliable.
     */
    auto onTriangle(std::array<Eigen::Vector2d,3> const& p)
        -> std::pair<std::vector<Bezier<Eigen::Vector2d,3> >,bool>;
};
