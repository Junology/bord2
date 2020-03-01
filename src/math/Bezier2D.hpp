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

//* Debug
#include <iostream>
#include <iomanip>
// */

namespace bord2 {
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
    constexpr bool isCollapsed() const noexcept {
        auto mid = beg/2 + end/2;
        return beg == mid || mid == end;
    }

    constexpr bool isDisjoint(Interval const& other) const noexcept {
        return beg > other.end || end < other.beg;
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

struct Rectangle {
    double left, right, top, bottom;
    constexpr bool isDisjoint(Rectangle const& other) const noexcept {
        return left > other.right
            || right < other.left
            || top < other.bottom
            || bottom > other.top;
    }
    constexpr bool isCollapsed() const noexcept {
        double xmid = left/2 + right/2;
        double ymid = top/2 + bottom/2;
        return (xmid == left || xmid == right)
            && (ymid == top || ymid == bottom);
    }
};

struct FatLine {
    AffHypPlane<2> line;
    Interval hrange;

    bool contains(Eigen::Vector2d const& pt) const noexcept {
        return hrange.contains(line.height(pt));
    }
};
}

/*!
 * Compute the interval spanned by the control points of a given Bernstein functions (i.e. Bezier curves in the one-dimensional real line).
 */
template <class BezierT>
constexpr auto getBand(
    BezierT const& bernfun,
    bord2::Interval const& domain = bord2::Interval{0.0, 1.0}
    ) noexcept
    -> bord2::Interval
{
    using vertex_type = std::remove_cv_t<std::remove_reference_t<typename BezierTraits<BezierT>::vertex_type>>;
    static_assert(std::is_convertible<decltype(std::declval<vertex_type>() < std::declval<vertex_type>()), bool>::value,
                  "The vertex type must be comparable with each other." );

    BezierT bern_clipped = bernfun.clip(domain.beg, domain.end);

    bord2::Interval result{ bern_clipped.source(), bern_clipped.source() };

    std::for_each(
        bern_clipped.begin()+1, bern_clipped.end(),
        [&result](vertex_type const& x) {
            if (x < result.beg)
                result.beg = x;
            if (result.end < x)
                result.end = x;
        } );

    return result;
}

/*!
 * Compute a fatline spanned by the control points of a Bezier curve.
 * \param bezsrc A Bezier curve spanning the fatline.
 * \param domain A (closed) subinterval of the unit interval [0,1] where the Bezier curve restricts to.
 */
template <class BezierT>
auto getFatLine(BezierT const& bezsrc,
                bord2::Interval const& domain = bord2::Interval{0.0, 1.0}
    ) noexcept
    -> bord2::FatLine
{
    using vertex_type = typename BezierTraits<BezierT>::vertex_type;

    vertex_type src = bezsrc.eval(domain.beg);
    vertex_type tangent = bezsrc.eval(domain.end) - src;
    vertex_type normal
        = tangent.isZero()
        ? Eigen::Vector2d(0.0, 1.0)
        : Eigen::Vector2d(-tangent(1), tangent(0));

    return {
        AffHypPlane<2>(normal, src),
        getBand(bezsrc.convert(
                    [&normal, &src](vertex_type const& v) -> double {
                        return normal.dot(v-src);
                    } ),
                domain )
    };
}

/*!
 * Clip the domain of the Bezier curve by a fatline.
 * See the paper Nishita, Takita, Nakamae, "Hidden Curve Elimination of Trimmed Surfaces Using Bezier Clipping."
 * \param fatline The fatline clipping the Bezier curve.
 * \param num The number (or depth) of clipping.
 * \param domain A subinterval of [0,1] where the parameter runs.
 * \param eps The margin of each division.
 * \return A vector of subintervals of the domain where the curve possibly intersects with the fatline.
 */
template <class BezierT>
auto clipByFL(BezierT const& bezsrc,
              bord2::FatLine const& fatline,
              size_t num,
              bord2::Interval domain = {0.0, 1.0},
              double eps = 0.0
    ) noexcept
    -> std::vector<bord2::Interval>
{
    // The type of control points.
    using vertex_type = typename BezierTraits<BezierT>::vertex_type;
    // The list of Bernstein functions (i.e. Bezier curves in the one-dimensional real line) obtained by clipping the projection of the original Bezier curve.
    using BernFun
        = typename BezierTraits<BezierT>::template converted_type<double>;
    std::vector<BernFun> bez = {
        bezsrc.convert(
            [&fatline](vertex_type const& pt) -> double {
                return fatline.line.height(pt);
            } ).clip(domain.beg, domain.end)
    };
    // The list of intervals obtained from the unit interval by the same clippings as bez.
    std::vector<bord2::Interval> result = {domain};

    // The procedure divides the interval into at most 2^num subintervals.
    result.reserve(bord2::cipow(2,num));

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

/*!
 * Compute the intersections with another 2d bezier curve.
 * We use iterated mutual Bezier clipping algorithm.
 * Since it may take much time, one can terminate the function using the flag argument (see below).
 * \tparam max_steps The number of steps of Bezier clippings. 12 is recommended.
 * \tparam clip_depth The depth of binary search in each Bezier clipping. 3 is recommended.
 * \param flag A flag if the computation is supposed to be continued or not, which should be a functor object with signature bool(). The default value is std::true_type{} which returns always true, so the compiler hopefully remove all its effects.
 * \return If the computation completed, the returned value is the list of parameters where two bezier curves intersect. As for each element of the vector, the first component is the parameter for *lhs* while the second is the one for *rhs*. On the other hand, if the computation is terminated, the returned value is always empty.
 * \post It is guaranteed that the returned vector is ordered so that the first components are non-decreasing (while no guarantee on the second).
 * \warning Intersections close to the ends will be thrown away.
 */
template <size_t max_steps, size_t clip_depth, class LHSBezier, class RHSBezier, class F = std::true_type>
auto intersect(LHSBezier const& lhs, RHSBezier const& rhs, F&& flag = {}) noexcept
    -> std::vector<std::pair<double,double>>
{
    // cf 2^30 = 1073741824 ~ 10^9
    constexpr double smp_dur = bord2::cipow(0.5, (max_steps-1)*clip_depth);
    constexpr double eps = bord2::cipow(0.5, max_steps*clip_depth + clip_depth);

    using typename bord2::Interval;

    // If two Bezier curves are the same, we think of them disjoint.
    // Hence the function returns the empty list immediately in this case.
    if(std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end())) {
        return {};
    }

    std::map<Interval, std::vector<Interval>> interval_map{
        std::make_pair(Interval{0.0, 1.0},
                       std::vector<Interval>(1,Interval{0.0, 1.0}))
    };
    decltype(interval_map) interval_aux;

    auto lhs_xbern = lhs.convert([](Eigen::Vector2d const& v) { return v(0); });
    auto lhs_ybern = lhs.convert([](Eigen::Vector2d const& v) { return v(1); });
    auto rhs_xbern = rhs.convert([](Eigen::Vector2d const& v) { return v(0); });
    auto rhs_ybern = rhs.convert([](Eigen::Vector2d const& v) { return v(1); });

    bool to_be_continued = flag();

    for(size_t i = 0; i < max_steps; ++i) {
        if (interval_map.empty() || !to_be_continued)
            return {};

        interval_aux.clear();
        for(auto& ipair : interval_map) {
            // Compute the enclosing rectangle of the LHS curve.
            std::array<bord2::Interval,2> rectL = {
                getBand(lhs_xbern, ipair.first),
                getBand(lhs_ybern, ipair.first)
            };

            // Intervals too close to the ends are ignored.
            if(ipair.first.end < smp_dur || ipair.first.beg > 1.0 - smp_dur
               || (rectL[0].isCollapsed() && rectL[1].isCollapsed()
                   && (ipair.first.beg < smp_dur
                       || ipair.first.end > 1.0 - smp_dur)))
                continue;

            for(auto& intrvl : ipair.second) {
                if(!to_be_continued)
                    return {};

                // Compute the enclosing rectangle of the RHS curve.
                std::array<bord2::Interval,2> rectR = {
                    getBand(rhs_xbern, intrvl),
                    getBand(rhs_ybern, intrvl)
                };

                // Intervals too close to the ends are ignored.
                if(intrvl.end < smp_dur || intrvl.beg > 1.0 - smp_dur
                   || (rectR[0].isCollapsed() && rectR[1].isCollapsed()
                       && (intrvl.beg < smp_dur
                           || intrvl.end > 1.0 - smp_dur)))
                    continue;

                // Skip if enclosing rectangles do not intersect.
                if(rectL[0].isDisjoint(rectR[0])
                   || rectL[1].isDisjoint(rectR[1]))
                    continue;

                // Clip the domain of the LHS Bezier curve.
                // If the enclosing rectangle is collapsed, just keep the old one.
                auto lhs_clips
                    = rectL[0].isCollapsed() && rectL[1].isCollapsed()
                    ? std::vector<bord2::Interval>{ipair.first}
                    : clipByFL(
                        lhs,
                        getFatLine(rhs, intrvl),
                        clip_depth, ipair.first, eps);

                if(lhs_clips.empty())
                   continue;

                for(auto& clipper : lhs_clips) {
                    // Check the continuation flag;
                    to_be_continued = flag();
                    // if it is false, return immediately.
                    if(!to_be_continued) {
                        return {};
                    }

                    // Append the clipped segments of LHS to the buffer.
                    auto itr = interval_aux.emplace(
                        clipper, std::vector<Interval>{});
                    // Clip the domain of the RHS Bezier curve.
                    // If the enclosing rectangle is collapsed, just keep the old one.
                    auto intvlR
                        = rectR[0].isCollapsed() && rectR[1].isCollapsed()
                        ? std::vector<bord2::Interval>{intrvl}
                        : clipByFL(
                            rhs,
                            getFatLine(lhs, clipper),
                            clip_depth, intrvl, eps);

                    std::copy(
                        std::make_move_iterator(intvlR.begin()),
                        std::make_move_iterator(intvlR.end()),
                        std::back_inserter(itr.first->second) );
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

            // Ignore intersections that are too close to the ends
            if(t0 < smp_dur || t0 > 1.0 - smp_dur
               || t1 < smp_dur || t1 > 1.0 - smp_dur)
                continue;

            auto itr = std::find_if(
                result.begin(), result.end(),
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
