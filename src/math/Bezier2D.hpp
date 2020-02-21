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
}

template <class T>
struct Bezier2D : public T
{
    using BaseType = T;
    using typename BaseType::vertex_type;

    // Delegate constructors
    using T::T;

    //! Copy-Construction from base class
    Bezier2D(T const& src) noexcept : T(src) {}

    //! Move-Construction from base class
    Bezier2D(T &&src) noexcept : T(std::move(src)) {}

    bord2::FatLine getFatLine(bord2::Interval const& domain = bord2::Interval{0.0, 1.0}) const noexcept {
        vertex_type src = BaseType::eval(domain.beg);
        vertex_type tangent = BaseType::eval(domain.end) - src;
        vertex_type normal
            = tangent.isZero()
            ? Eigen::Vector2d(0.0, 1.0)
            : Eigen::Vector2d(-tangent(1), tangent(0));
        normal.normalize();
        bord2::FatLine result = {
            AffHypPlane<2>(normal, src),
            bord2::Interval{0.0, 0.0}
        };

        auto hbezfun = BaseType::convert(
            [&normal, &src](vertex_type const& v) -> double {
                return normal.dot(v-src);
            } ).clip(domain.beg, domain.end);

        for(double height : hbezfun) {
            if (height < result.hrange.beg)
                result.hrange.beg = height;
            if (height > result.hrange.end)
                result.hrange.end = height;
        }
        return result;
    }

    //! Clip the domain of the Bezier curve by a fatline.
    //! See the paper Nishita, Takita, Nakamae, "Hidden Curve Elimination of Trimmed Surfaces Using Bezier Clipping."
    //! \param fatline The fatline clipping the Bezier curve.
    //! \param num The number (or depth) of clipping.
    //! \param domain A subinterval of [0,1] where the parameter runs.
    //! \param eps The margin of each division.
    //! \return A vector of subintervals of the domain where the curve possibly intersects with the fatline.
    auto clipByFL(bord2::FatLine const& fatline, size_t num, bord2::Interval domain = {0.0, 1.0}, double eps = 0.0) const noexcept
        -> std::vector<bord2::Interval>
    {
        // The list of Bezier functions (i.e. Bezier curves in the one-dimensional Euclidean plane) obtained by clipping the projection of the original Bezier curve.
        using BezierFunction = decltype(std::declval<T>().convert(std::declval<double(vertex_type)>()));
        std::vector<BezierFunction> bez = {
            BaseType::convert(
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
};

//! Compute the intersections with another 2d bezier curve.
//! We use iterated mutual Bezier clipping algorithm.
//! \return The list of parameters where two bezier curves intersect. As for each element of the vector, the first component is the parameter for *this* class while the second is the one for *other*.
template <class LHSBezier, class RHSBezier>
auto intersect(LHSBezier const& lhs, RHSBezier const& rhs) noexcept
    -> std::vector<std::pair<double,double>>
{
    using typename bord2::Interval;

    std::map<Interval, std::vector<Interval>> interval_map{
        std::make_pair(Interval{0.0, 1.0},
                       std::vector<Interval>{Interval{0.0, 1.0}})
    };
    decltype(interval_map) interval_aux;

    // cf 2^30 = 1073741824 ~ 10^9
    constexpr size_t max_steps = 12;
    constexpr size_t clip_depth = 3;
    constexpr double smp_dur = bord2::cipow(0.5, max_steps-1);
    constexpr double eps = bord2::cipow(0.5, max_steps*clip_depth);

    for(size_t i = 0; i < max_steps; ++i) {
        if (interval_map.empty())
            return {};

        interval_aux.clear();
        for(auto& ipair : interval_map) {
            for(auto& intrvl : ipair.second) {
                auto left_clips = lhs.clipByFL(
                    rhs.getFatLine(intrvl),
                    clip_depth, ipair.first, eps);

                if(left_clips.empty())
                   continue;

                for(auto& clipper : left_clips) {
                    auto itr = interval_aux.emplace(
                        clipper, std::vector<Interval>{});
                    auto intvlR = rhs.clipByFL(
                            lhs.getFatLine(clipper),
                            clip_depth, intrvl, eps);
                    itr.first->second.insert(
                        std::end(itr.first->second),
                        intvlR.begin(), intvlR.end() );
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
