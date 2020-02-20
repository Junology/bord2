/*!
 * \file Bezier2D.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 16, 2020: created
 */

#pragma once

#include "Bezier.hpp"

#include <set>
#include <map>
#include <numeric>
#include <thread>

#include "AffHypPlane.hpp"
#include "misc.hpp"

#include <iostream> // Debug

namespace bord2 {
struct Rect {
    // Coordinate is chosen so that left-to-right and bottom-to-top.
    double left, right, top, bottom;
    bool contains(Eigen::Vector2d const& pt) const noexcept {
        return left <= pt(0) && pt(0) <= right
            && bottom <= pt(1) && pt(1) <= top;
    }

    bool isOverlapping(Rect const& other) const noexcept {
        return left <= other.right
            && right >= other.left
            && top >= other.bottom
            && bottom <= other.top;
    }
};

//! Closed interval
struct Interval{
    double beg, end;
    constexpr bool contains(double t) const noexcept {
        return beg <= t && t <= end;
    }
    constexpr Interval first(double eps = 0.0) const noexcept {
        return {beg, beg/2 + end/2 + eps};
    }
    constexpr Interval latter(double eps = 0.0) const noexcept {
        return {beg/2 + end/2 - eps, end};
    }
    constexpr bool isDividable() const noexcept {
        auto mid = beg/2 + end/2;
        return beg < mid && mid < end;
    }

    // Composition in the sense of that in the little 1-cubes operad.
    constexpr Interval comp(Interval const& inner) const noexcept {
        return Interval{beg + inner.beg*(end-beg), beg + inner.end*(end-beg)};
    }

    template <class Container>
    bool isAcross(Container const& container) const noexcept {
        static_assert(
            std::is_convertible<std::remove_cv_t<std::remove_reference_t<decltype(*(container.begin()))>>,double>::value,
            "The argument type must " );
        bool flag_above = true;
        bool flag_below = true;
        for(double val : container) {
            flag_below = flag_below && val < beg;
            flag_above = flag_above && val > end;
        }

        return !flag_below && !flag_above;
    }

    // Lexicographical ordering
    constexpr bool operator<(Interval const& rhs) const noexcept {
        return beg<rhs.beg || (beg==rhs.beg && end<rhs.end);
    }
};

struct FatLine {
    AffHypPlane<2> line;
    Interval hrange;

    bool contains(Eigen::Vector2d const& pt) const noexcept {
        return hrange.contains(line.height(pt));
    }
};

enum Orientation {
    OnLine,
    Clockwise,
    CntrClockwise
};

inline Orientation orientation(Eigen::Vector2d const& p0, Eigen::Vector2d const& p1, Eigen::Vector2d const& p2) noexcept {
    Eigen::Vector2d v = p2-p0;
    double innprod = (p1-p0).dot(Eigen::Vector2d(v(1),-v(0)));
    if (innprod > 0.0)
        return CntrClockwise;
    else if (innprod < 0.0)
        return Clockwise;
    else
        return OnLine;
}
}

template <size_t deg>
struct Bezier2D : public Bezier<Eigen::Vector2d,deg>
{
    template <size_t>
    friend struct Bezier2D;

    using BaseType = Bezier<Eigen::Vector2d,deg>;
    using typename BaseType::vertex_type;

    // Delegate constructors
    using Bezier<Eigen::Vector2d,deg>::Bezier;

    //! Copy-Construction from base class
    Bezier2D(BaseType const& src) noexcept
        : Bezier<Eigen::Vector2d,deg>(src)
    {}

    //! Move-Construction from base class
    Bezier2D(BaseType &&src) noexcept
        : Bezier<Eigen::Vector2d,deg>(std::move(src))
    {}

    bord2::Rect getRect() const noexcept {
        bord2::Rect result{
            BaseType::m_pts[0](0),
            BaseType::m_pts[0](0),
            BaseType::m_pts[0](1),
            BaseType::m_pts[0](1)
        };

        for(size_t i = 1; i < BaseType::num_pts; ++i) {
            if (BaseType::m_pts[i](0) < result.left)
                result.left = BaseType::m_pts[i](0);
            else if (BaseType::m_pts[i](0) > result.right)
                result.right = BaseType::m_pts[i](0);
            if (BaseType::m_pts[i](1) > result.top)
                result.top = BaseType::m_pts[i](1);
            else if (BaseType::m_pts[i](1) < result.bottom)
                result.bottom = BaseType::m_pts[i](1);
        }
        return result;
    }

    bord2::FatLine getFatLine() const noexcept {
        vertex_type tangent = BaseType::target() - BaseType::source();
        vertex_type normal
            = tangent.isZero()
            ? Eigen::Vector2d(0.0, 1.0)
            : Eigen::Vector2d(-tangent(1), tangent(0));
        normal.normalize();
        bord2::FatLine result = {
            AffHypPlane<2>(normal, BaseType::source()),
            bord2::Interval{0.0, 0.0}
        };

        for(auto itr = BaseType::begin()+1; itr+1 < BaseType::end(); ++itr) {
            double height = result.line.height(*itr);
            if (height < result.hrange.beg)
                result.hrange.beg = height;
            if (height > result.hrange.end)
                result.hrange.end = height;
        }
        return result;
    }

    //! Check if there is a control point lying in a given shape.
    //! \param shape The shape which has a member function of signature
    //!   >> bool T::contains(Eigen::Vector2d const&);
    template <class T>
    bool ctrlLiesIn(T const& shape) const noexcept(T::contains(std::declval<T*>(), std::declval<Eigen::Vector2d const&>())) {
        for(auto& pt : *this) {
            if (shape.contains(pt))
                return true;
        }
        return false;
    }

    //! Clip the Bezier curve by a fatline.
    //! See the paper Nishita, Takita, Nakamae, "Hidden Curve Elimination of Trimmed Surfaces Using Bezier Clipping."
    //! \param fatline The fatline clipping the Bezier curve.
    //! \param num The number (or depth) of clipping.
    auto clipByFL(bord2::FatLine const& fatline, size_t num, double eps = 0.0) const noexcept
        -> std::vector<bord2::Interval>
    {
        // The list of Bezier functions (i.e. Bezier curves in the one-dimensional Euclidean plane) obtained by clipping the projection of the original Bezier curve.
        std::vector<Bezier<double,deg>> bez = {
            BaseType::convert(
                [&fatline](vertex_type const& pt) -> double {
                    return fatline.line.height(pt);
                } )
        };
        // The list of intervals obtained from the unit interval by the same clippings as bez.
        std::vector<bord2::Interval> result = {
            bord2::Interval{0.0, 1.0}
        };
        // Temporary variables.
        decltype(bez) bez_aux;
        decltype(result) result_aux;

        // Begin iteration: width-first binary search.
        for(size_t n = 0; n < num; ++n) {
            if (result.empty())
                return {};

            // Clear the temporary buffers.
            bez_aux.clear();
            result_aux.clear();

            // Traverse all clipped Bezier function.
            for(size_t i = 0; i < bez.size(); ++i) {
                // Divide the curve and the interval, and puth onto the stack
                auto bezdiv = bez[i].divide(0.5, eps);

                // Check if the convex hulls of the control points of the divided Bezier curves intersects with the fatline.
                // If so, push them onto the stack
                if (fatline.hrange.isAcross(bezdiv.first)) {
                    bez_aux.push_back(bezdiv.first);
                    result_aux.push_back(result[i].first(eps));
                }
                if (fatline.hrange.isAcross(bezdiv.second)) {
                    bez_aux.push_back(bezdiv.second);
                    result_aux.push_back(result[i].latter(eps));
                }
            }

            // Update the data
            bez.swap(bez_aux);
            result.swap(result_aux);
        }

        return result;
    }

    template <size_t N>
    bool hasRectOverlap(Bezier2D<N> const& other) const noexcept {
        return getRect().isOverlapping(other.getRect());
    }

    //! Compute the convex hull.
    //! \return A list of control points spanning the hull. They are stored in the counter-clockwise order.
    std::vector<vertex_type> getConvexHull() const noexcept
    {
        // Return immediately if there are at most one control point.
        if (deg <= 0)
            return {};

        // Find a control point with the minimal y-coordinate.
        auto miny_itr = std::min_element(
            std::begin(BaseType::m_pts),
            std::end(BaseType::m_pts),
            [](Eigen::Vector2d const& lhs, Eigen::Vector2d const& rhs) {
                return lhs(1)<rhs(1) || (lhs(1)==rhs(1) && lhs(0)<rhs(0));
            } );

        // angle-radius lexicographical ordering on 2d vectors.
        struct PolarLexLess {
            Eigen::Vector2d const& m_orig;
            PolarLexLess(Eigen::Vector2d const& orig) : m_orig(orig) {}
            bool operator()(Eigen::Vector2d const& lhs, Eigen::Vector2d const& rhs) const noexcept {
                switch(bord2::orientation(m_orig, lhs, rhs)) {
                case bord2::OnLine:
                    return (lhs-m_orig).norm() < (rhs-m_orig).norm();

                case bord2::Clockwise:
                    return false;

                case bord2::CntrClockwise:
                    return true;
                }
            }
        };
        // Store the point in the angle-radius lexicographical order.
        std::set<Eigen::Vector2d,PolarLexLess> buf{PolarLexLess(*miny_itr)};
        for(auto itr = std::begin(BaseType::m_pts); itr != std::end(BaseType::m_pts); ++itr) {
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
            while(bord2::orientation(result[result.size()-2], result.back(), pt) != bord2::CntrClockwise) {
                result.pop_back();
            }
            result.push_back(pt);
        }

        return result;
    }

    //! Check if the convex hull of control points overlaps with that of another Bezier curve.
    //! Here, so-called *The Separating Axis Theorem* is used.
    template <size_t N>
    bool hasHullIntersection(Bezier2D<N> const& other) const noexcept
    {
        bool is_separated = false;

        Eigen::Matrix2d rmat;
        rmat << 0.0, -1.0, 1.0, 0.0;

        auto hull = getConvexHull();
        for(size_t i = 1; i < hull.size(); ++i) {
            is_separated = other.allSatisfy(
                [&hull, &rmat, &i](vertex_type const& o_pt) -> bool{
                    return (hull[i]-hull[i-1]).dot(rmat*(o_pt-hull[i-1])) > 0;
                } );
            if (is_separated)
                return false;
        }
        is_separated = other.allSatisfy(
            [&hull, &rmat](vertex_type const& o_pt) -> bool{
                return (hull.front()-hull.back()).dot(rmat*(o_pt-hull.back())) > 0;
                } );
        if (is_separated)
            return false;

        hull = other.getConvexHull();
        for(size_t i = 1; i < hull.size(); ++i) {
            is_separated = BaseType::allSatisfy(
                [&hull, &rmat, &i](vertex_type const& o_pt) -> bool{
                    return (hull[i]-hull[i-1]).dot(rmat*(o_pt-hull[i-1])) > 0;
                } );
            if (is_separated)
                return false;
        }
        is_separated = BaseType::allSatisfy(
            [&hull, &rmat](vertex_type const& o_pt) -> bool{
                return (hull.front()-hull.back()).dot(rmat*(o_pt-hull.back())) > 0;
            } );
        if (is_separated)
            return false;

        return true;
    }
};

//! Compute the intersections with another 2d bezier curve.
//! We use iterated mutual Bezier clipping algorithm.
//! \return The list of parameters where two bezier curves intersect. As for each element of the vector, the first component is the parameter for *this* class while the second is the one for *other*.
template <size_t M, size_t N>
auto intersect(Bezier2D<M> const& lhs, Bezier2D<N> const& rhs) noexcept
    -> std::vector<std::pair<double,double>>
{
    using LHSBezier = Bezier2D<M>;
    using RHSBezier = Bezier2D<N>;
    using typename bord2::Interval;

    std::map<Interval, std::vector<Interval>> interval_map{
        std::make_pair(Interval{0.0, 1.0},
                       std::vector<Interval>{Interval{0.0, 1.0}})
    };
    decltype(interval_map) interval_aux;

    // cf 2^30 = 1073741824 ~ 10^9
    constexpr size_t max_steps = 9;
    constexpr size_t clip_depth = 4;
    constexpr double smp_dur = bord2::cipow(0.5, max_steps-1);
    constexpr double eps = bord2::cipow(0.5, max_steps*clip_depth);

    for(size_t i = 0; i < max_steps; ++i) {
        if (interval_map.empty())
            return {};

        interval_aux.clear();
        for(auto& ipair : interval_map) {
            LHSBezier bezl = lhs.clip(ipair.first.beg, ipair.first.end);
            for(auto& intrvl : ipair.second) {
                RHSBezier bezr = rhs.clip(intrvl.beg, intrvl.end);
                auto left_clips = bezl.clipByFL(bezr.getFatLine(),
                                                clip_depth,
                                                eps);

                if(left_clips.empty())
                   continue;

                for(auto& clipper : left_clips) {
                    auto itr = interval_aux.emplace(
                        ipair.first.comp(clipper), std::vector<Interval>{});
                    LHSBezier bezl_clipped
                        = bezl.clip(clipper.beg, clipper.end);
                    auto intvlR
                        = bezr.clipByFL(bezl_clipped.getFatLine(),
                                        clip_depth,
                                        eps);
                    std::transform(
                        intvlR.begin(), intvlR.end(),
                        std::back_inserter(itr.first->second),
                        [&intrvl](Interval const& ts) -> Interval {
                            return intrvl.comp(ts);
                        } );
                }
            }
        }
        interval_map.swap(interval_aux);

        /* Debug
        std::cout << __FILE__":" << __LINE__ << std::endl;
        for(auto& ipair : interval_map) {
            std::cout << "[" << ipair.first.beg
                      << "," << ipair.first.end
                      << "] intersects with:" << std::endl;
            for(auto& clipper : ipair.second) {
                std::cout << "- [" << clipper.beg
                          << "," << clipper.end
                          << "]" << std::endl;
            }
        }
        // */
    }

    // Write the result.
    std::vector<std::pair<double,double>> result{};
    for(auto& ipair : interval_map) {
        auto& intervalL = ipair.first;
        for(auto& intervalR : ipair.second) {
            double t0 = intervalL.beg/2 + intervalL.end/2;
            double t1 = intervalR.beg/2 + intervalR.end/2;

            auto itr = std::find_if(
                result.begin(),
                result.end(),
                [&t0, &t1](std::pair<double,double> const& params) -> bool{
                    return std::abs(params.first -t0) <= smp_dur + 2*eps
                        && std::abs(params.second - t1) <= smp_dur + 2*eps;
                } );
            if(itr == result.end()) {
                result.emplace_back(t0, t1);
            }
        }
    }
    return result;
}
