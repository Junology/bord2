/*!
 * \file PlainGeometry.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 20, 2020: created
 */

#include "PlaneGeometry.hpp"

#include <set>

namespace bord2{

Orientation orientation(Eigen::Vector2d const& p0, Eigen::Vector2d const& p1, Eigen::Vector2d const& p2) noexcept {
    Eigen::Vector2d v = p2-p0;
    double innprod = (p1-p0).dot(Eigen::Vector2d(v(1),-v(0)));
    if (innprod > 0.0)
        return CntrClockwise;
    else if (innprod < 0.0)
        return Clockwise;
    else
        return OnLine;
}

//! Compute the convex hull.
//! \param A set of vertices.
//! \return A list of control points spanning the hull. They are stored in the counter-clockwise order.
std::vector<Eigen::Vector2d> getConvexHull(std::vector<Eigen::Vector2d> const& vs) noexcept
{
    // Return immediately if there are at most one control point.
    if (vs.size() <= 1)
        return {};

    // Find a control point with the minimal y-coordinate.
    auto miny_itr = std::min_element(
        std::begin(vs),
        std::end(vs),
        [](Eigen::Vector2d const& lhs, Eigen::Vector2d const& rhs) {
            return lhs(1)<rhs(1) || (lhs(1)==rhs(1) && lhs(0)<rhs(0));
        } );

    // angle-radius lexicographical ordering on 2d vectors.
    struct PolarLexLess {
        Eigen::Vector2d const& m_orig;
        PolarLexLess(Eigen::Vector2d const& orig) : m_orig(orig) {}
        bool operator()(Eigen::Vector2d const& lhs, Eigen::Vector2d const& rhs) const noexcept {
            switch(orientation(m_orig, lhs, rhs)) {
            case OnLine:
                return (lhs-m_orig).norm() < (rhs-m_orig).norm();

            case Clockwise:
                return false;

            case CntrClockwise:
                return true;
            }
        }
    };
    // Store the point in the angle-radius lexicographical order.
    std::set<Eigen::Vector2d,PolarLexLess> buf{PolarLexLess(*miny_itr)};
    for(auto itr = std::begin(vs); itr != std::end(vs); ++itr) {
        if (itr == miny_itr)
            continue;

        if (!(*itr-*miny_itr).isZero())
            buf.insert(*itr);
    }

    // If no points in buffer, return immediately.
    if (buf.empty())
        return {};

    // Push the initial point and the next to the result stack.
    std::vector<Eigen::Vector2d> result = {
        *miny_itr, *buf.begin()
    };
    buf.erase(buf.begin());

    // Traverse all the control points in the buffer.
    for(auto pt : buf) {
        // Pop the points from the stack until a counter-clock triangle is found.
        while(orientation(result[result.size()-2], result.back(), pt) != CntrClockwise) {
            result.pop_back();
        }
        result.push_back(pt);
    }

    return result;
}

//! Check if the convex hull of control points overlaps with that of another Bezier curve.
//! Here, so-called *The Separating Axis Theorem* is used.
bool hasHullIntersection(std::vector<Eigen::Vector2d> const& lhs,
                         std::vector<Eigen::Vector2d> const& rhs) noexcept
{
    bool is_separated;

    Eigen::Matrix2d rmat;
    rmat << 0.0, -1.0, 1.0, 0.0;

    auto hull = getConvexHull(lhs);
    for(size_t i = 1; i < hull.size(); ++i) {
        is_separated = true;
        for(auto& pt : rhs) {
            is_separated = is_separated && (hull[i]-hull[i-1]).dot(rmat*(pt-hull[i-1])) > 0;
        }

        if (is_separated)
            return false;
    }
    is_separated = true;
    for(auto& pt : rhs) {
        is_separated = is_separated && (hull.front()-hull.back()).dot(rmat*(pt-hull.back())) > 0;
    }
    if (is_separated)
        return false;

    hull = getConvexHull(rhs);
    for(size_t i = 1; i < hull.size(); ++i) {
        is_separated = true;
        for(auto& pt : lhs) {
            is_separated = is_separated && (hull[i]-hull[i-1]).dot(rmat*(pt-hull[i-1])) > 0;
        }

        if (is_separated)
            return false;
    }
    is_separated = true;
    for(auto& pt : lhs) {
        is_separated = is_separated && (hull.front()-hull.back()).dot(rmat*(pt-hull.back())) > 0;
    }
    if (is_separated)
        return false;

    return true;
}

}
