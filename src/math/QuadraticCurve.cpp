/*!
 * \file QuadraticCurve.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 13, 2019: created
 */

#include "QuadraticCurve.hpp"

// #include <iostream> // For Debug

#include "misc.hpp"
#include "solvers.hpp"

/*********************************************************
 *** Implementation of members of QuadraticCurve class ***
 *********************************************************/
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
    return result;
}

/* Approximate the intersection with a given triangle by quadratic Bezier curves.
 * \param p The array of vertices that span a triangle.
 * \return The set of Bezier curves each of which approximates a connected component of the intersection.
 */
auto QuadraticCurve::onTriangle(std::array<Eigen::Vector2d,3> const& p)
    -> std::pair<std::vector<Bezier<Eigen::Vector2d,3> >,bool>
{
    // If the triangle is extremely small, nothing to return;
    if (std::abs(cross2D(p[1]-p[0], p[2]-p[0])) < QuadraticCurve::threshold)
        return {std::vector<Bezier<Eigen::Vector2d,3> >(), false};

    // The local struct to represent end points of the curve in the triangle.
    struct EndType {
        // The point.
        Eigen::Vector2d pt;
        // a vector toward the interior of the triangle.
        Eigen::Vector2d toInn;
        // The flag indicating wheather it is a positive end or not.
        bool is_positive;
    };

    // The variable where the set of end points will be stored.
    // The end points in different global connected components are distinguished.
    std::array<std::vector<EndType>,2> ends;

    // Compute the intersections with edges of the triangle.
    for(size_t i = 0; i < p.size(); ++i)
    {
        auto ts = QuadraticCurve::intersectParams(p[i], p[(i+1)%3]);

        if (ts.empty())
            continue;

        // For each intersection, add the point provided it is either positive or negative.
        for(double t : ts) {
            Eigen::Vector2d v = p[(i+1)%3] - p[i];
            Eigen::Vector2d pt = p[i] + t*v;
            Eigen::Vector2d vperp(-v(1),v(0));
            vperp.normalize();
            Eigen::Vector2d toInn
                = vperp * (p[(i+2)%3]-pt).adjoint() * vperp;
            Eigen::Vector2d kappa = QuadraticCurve::curvature(pt);
            double inprod = kappa.adjoint() * v;

            // If the curve has non-zero curvature and if the edge is tangent at pt, we skip.
            if(std::abs(inprod) < kappa.norm()*QuadraticCurve::threshold)
                continue;

            // Otherwise, push it to the list of endpoints.
            // Later, we will separate end points in different components.
            ends[0].push_back({pt, toInn, !std::signbit(inprod)});
        }
    }

    auto sepline = QuadraticCurve::divAxis();
    // If the curve is hyperbolic, separate end points in different components.
    if (sepline) {
        for(auto itr = ends[0].begin(); itr != ends[0].end(); /* nothing */) {
            // For one side, move the element to the other buffer.
            if (std::signbit(sepline->height(itr->pt))) {
                ends[1].push_back(*itr);
                itr = ends[0].erase(itr);
            }
            // Else just keep the element.
            else {
                ++itr;
            }
        }
    }

    // The variable where the result will be stored.
    std::vector<Bezier<Eigen::Vector2d,3> > result;
    bool reliability = true;

    // For each cluster of end points, compute their connections.
    for (auto& e : ends) {
        while (!e.empty()) {
            // If the number of remaining end points becomes odd, the result is no longer reliable.
            if (e.size() < 2) {
                reliability = false;
                break;
            }

            // The index of the last end point in the buffer.
            size_t i = e.size() - 1;
            // The index of the possible companion.
            size_t j = e[i].is_positive ? (e.size()-2) : 0;

            // Something is wrong on the companion
            if (e[i].is_positive == e[j].is_positive) {
                // Just ignore the current point
                e.pop_back();
                // The result is no longer reliable.
                reliability = false;
                continue;
            }

            // The tangent lines
            auto tline_i = QuadraticCurve::tangent(e[i].pt);
            auto tline_j = QuadraticCurve::tangent(e[j].pt);

            // Compute the intersection
            auto ret = tline_i.intersect(tline_j);

            // If two tangent lines intersect.
            if (ret.first) {
                Eigen::Vector2d vi = ret.second - e[i].pt;
                Eigen::Vector2d vj = ret.second - e[j].pt;

                if (static_cast<double>(e[i].toInn.adjoint()*vi) < 0)
                    vi = -vi;
                if (static_cast<double>(e[j].toInn.adjoint()*vj) < 0)
                    vj = -vj;

                result.emplace_back(
                    e[i].pt,
                    e[i].pt + (2.0/3.0)*vi,
                    e[j].pt + (2.0/3.0)*vj,
                    e[j].pt
                    );
            }
            // If two tangent lines are parallel.
            else {
                result.emplace_back(
                    e[i].pt,
                    e[i].pt + 0.5*e[i].toInn,
                    e[j].pt + 0.5*e[j].toInn,
                    e[j].pt
                    );
            }

            // Remove the processed end points.
            e.pop_back();
            e.erase(std::next(e.begin(),j));
        }
    }

    return {std::move(result), std::move(reliability)};
}
