/*!
 * \file QuadraticCurve.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 13, 2019: created
 */

#include "QuadraticCurve.hpp"

//#include <iostream> // For Debug

#include "solvers.hpp"

constexpr double QuadraticCurve::threshold;

// Constructor
QuadraticCurve::QuadraticCurve(double a, double b, double c, double d, double e, double f)
{
    m_M <<
        a, d/2, e/2,
        d/2, b, f/2,
        e/2, f/2, c;
}

// Compute the curvature vector at a given point (x0,y0).
Eigen::Vector2d QuadraticCurve::curvature(double x0, double y0) const
{
    Eigen::Vector2d df = 2.0*m_M.block<2,3>(0,0) * Eigen::Vector3d(x0, y0, 1.0);
    double norm2 = df(0)*df(0) + df(1)*df(1);
    double coeff
        = 2*df(1)*df(1)*m_M(0,0)
        - 4*df(0)*df(1)*m_M(0,1)
        + 2*df(0)*df(0)*m_M(1,1);

    return -(coeff/(norm2*norm2))*df;
}

/* If the curve is hyperbolic, compute an affine line such that
 * - it divides the plain into two components each of which contains one of the two connected component of the hyperbola;
 * - the curve is invariant under the reflection associated to the line.
 * If the curve is not hyperbolic, i.e. it is elliptic, parabolic, or degenerate, then nullptr is returned.
 */
std::unique_ptr<AffHypPlane<2> > QuadraticCurve::divAxis() const
{
    // Eigenvalues are the roots of the characteristic polynomial.
    double detM = m_M.determinant();
    double detm = m_M(0,0)*m_M(1,1) - m_M(1,0)*m_M(0,1);

    // If the associated quadratic form is degenerate or if the curve is elliptic or parabolic, then return nullptr.
    if (detM == 0.0 || detm >= 0.0)
        return nullptr;

    // Since detm is the determinant of a symmetric matrix, the discriminant of the quadratic equation is always non-negative.
    // Hence, both values in eigens are finite.
    // Moreover, since detm < 0.0, we have
    //  > eigens.first < 0.0 < eigens.second
    auto eigens = bord2::solveQuad(1.0, -m_M(0,0)-m_M(1,1), detm);
    // We choose the eigenvalue which has the different sign from detM/detm.
    double lambda = std::signbit(detM) ? eigens.first : eigens.second;

    // Compute an eigenvector associated to lambda.
    Eigen::Vector2d evec
        = (m_M(0,0)!=lambda)
        ? Eigen::Vector2d(-m_M(0,1), m_M(0,0)-lambda)
        : Eigen::Vector2d(m_M(1,1)-lambda, -m_M(1,0));

    return std::make_unique<AffHypPlane<2> >(
        std::array<double,2>{evec(0)*lambda, evec(1)*lambda},
        evec(0)*m_M(0,2) + evec(1)*m_M(1,2)
        );
}

// Compute the parameter t (0<=t<=1) at which the segment (1-t)p0 + t p1 intersects with the curve.
std::vector<double> QuadraticCurve::intersectParams(Eigen::Vector2d const &p0, Eigen::Vector2d const &p1)
{
    // Vector of the direction of p1 from p0
    Eigen::Vector2d v = p1-p0;
    Eigen::Vector3d p0E(p0(0), p0(1), 1);

    // Coefficients of the equation we have to solve
    double ca = (v.adjoint() * m_M.block<2,2>(0,0) * v)(0);
    double cb = 2.0 * (v.adjoint() * (m_M.block<2,3>(0,0) * p0E))(0);
    double cc = (p0E.adjoint() * m_M * p0E)(0);

    auto sols = bord2::solveQuad<double>(ca, cb, cc);

    std::vector<double> result;

    if (std::isfinite(sols.first))
    {
        if (!std::signbit(sols.first) && sols.first <= 1.0) {
            result.push_back(sols.first);
        }

        if (std::isfinite(sols.second)
            && !std::signbit(sols.second)
            && sols.second <= 1.0
            && sols.first != sols.second )
        {
            result.push_back(sols.second);
        }
    }
    return std::move(result);
}

//! Approximate the intersection with a given triangle by quadratic Bezier curves.
//! \param p The array of vertices that span a triangle.
//! \return The set of Bezier curves each of which approximates a connected component of the intersection.
auto QuadraticCurve::onTriangle(std::array<Eigen::Vector2d,3> const& p)
    -> std::vector<Bezier<Eigen::Vector2d,3> >
{
    // The variable where the intersections with the boundary of the triangle together with a vector directing the "inside of the triangle" will be stored.
    std::vector<std::array<Eigen::Vector2d,2> > ends;

    // Check if the vertices are solutions.
    for(size_t i = 0; i < p.size(); ++i) {
        if (QuadraticCurve::evaluate(p[i](0),p[i](1)) < QuadraticCurve::threshold) {
            // We are interested only on the vertex such that the tangent line divides the other two vertices into different components.
            auto tline = QuadraticCurve::tangent(p[i]);
            if (std::signbit(tline.height(p[(i+1)%3])) != std::signbit(tline.height(p[(i+2)%3]))) {
                Eigen::Vector2d vec = 0.5*(p[(i+1)%3]+p[(i+2)%3])-p[i];
                ends.emplace_back(std::array<Eigen::Vector2d,2>{p[i], vec});
            }
        }
    }

    // Compute the intersections with edges of the triangle.
    for(size_t i = 0; i < p.size(); ++i)
    {
        auto ts = QuadraticCurve::intersectParams(p[i%3], p[(i+1)%3]);

        if (ts.empty())
            continue;

        // A vector normal to the segment between p[i] and p[(i+1)%3].
        Eigen::Vector2d normal
            = ((p[i]-p[(i+1)%3]).norm() < QuadraticCurve::threshold)
            ? (p[(i+2)%3] - p[i])
            : Eigen::Vector2d(
                p[i](1) - p[(i+1)%3](1),
                p[(i+1)%3](0) - p[i](0)
                );

        normal.normalize();

        // For each intersection, add the pair of the associated point and the normal vector, directed inside the triangle, to the buffer.
        for(double t : ts) {
            Eigen::Vector2d pt = (1-t)*p[i%3] + t*p[(i+1)%3];
            Eigen::Vector2d vec
                = normal * normal.adjoint() * (p[(i+2)%3] - pt);
            ends.emplace_back(std::array<Eigen::Vector2d,2>{pt,vec});
        }
    }

    // Eliminate duplicates.
    for(size_t i = 0; i < ends.size(); ++i) {
        for(auto itr = std::next(ends.begin(),i); itr != ends.end(); ) {
            if ((ends[i][0]-(*itr)[0]).norm() < QuadraticCurve::threshold) {
                itr = ends.erase(itr);
            }
            else
                ++itr;
        }
    }

    // The variable where the result will be stored.
    std::vector<Bezier<Eigen::Vector2d,3> > result;

    // Working stack; we expand the recursion using stack.
    std::vector<decltype(ends)> vstack{ends};

    while(!vstack.empty()) {
        // Pop an element from the stack.
        auto& pts = vstack.back();
        vstack.pop_back();

        // If there are only at most 1 point, then ignore the list.
        if (pts.size() <= 1)
            continue;

        // If the list has exactly two point, make a Bezier curve.
        if (pts.size() == 2) {
            auto tline1 = QuadraticCurve::tangent(pts[0][0]);
            auto tline2 = QuadraticCurve::tangent(pts[1][0]);
            auto intersect = tline1.intersect(tline2);

            // If two tangent lines intersect, determine the tangent vectors in appropriate directions using the intersection and take them as control points.
            if(intersect.first) {
                Eigen::Vector2d v1 = (2.0/3.0)*(intersect.second - pts[0][0]);
                Eigen::Vector2d v2 = (2.0/3.0)*(intersect.second - pts[1][0]);

                if(std::signbit(static_cast<double>(pts[0][1].adjoint() * v1)))
                    v1 = -v1;

                if(std::signbit(static_cast<double>(pts[1][1].adjoint() * v2)))
                    v2 = -v2;

                result.emplace_back(
                    pts[0][0],
                    pts[0][0] + v1,
                    pts[1][0] + v2,
                    pts[1][0]
                    );
            }
            // Otherwise, use the half of the attached normal vectors.
            else {
                result.emplace_back(
                    pts[0][0],
                    pts[0][0] + 0.5*pts[0][1],
                    pts[1][0] + 0.5*pts[1][0],
                    pts[1][0]
                    );
            }
        }
        // If the list consists of more than 2 points, divide it into two and compute the paring for each one.
        else {
            // Take the last point as a reference point.
            std::array<Eigen::Vector2d,2> pt_top = pts.back();
            auto tline = QuadraticCurve::tangent(pt_top[0]);
            pts.pop_back();

            // The reference point is considered in the both continued steps.
            decltype(ends) pts_pos{pt_top}, pts_neg{pt_top};

            for(auto pt : pts) {
                if (std::signbit(tline.height(pt[0])))
                    pts_neg.push_back(pt);
                else
                    pts_pos.push_back(pt);
            }

            vstack.push_back(pts_neg);
            vstack.push_back(pts_pos);
        }
    }

    return std::move(result);
}
