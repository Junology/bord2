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
#include <numeric>

#include "misc.hpp"

template <size_t deg>
struct Bezier2D : public Bezier<Eigen::Vector2d,deg>
{
    template <size_t>
    friend struct Bezier2D;

    using BaseType = Bezier<Eigen::Vector2d,deg>;
    using typename BaseType::vertex_type;

    using Bezier<Eigen::Vector2d,deg>::Bezier;

    //! Apply an operation on each face of the convex hull of control points.
    //! \param f The function-like object looks like as follows:
    //!   >> T f(vertex_type const& point, vertex_type const& normal);
    //! where two arguments point and normal determines a face in the following way:
    //!   - point is a point that belongs to the face;
    //!   - normal is the normal vector of the face pointing outside of the convex hull.
    //! \return Whether the procedure finished successfully.
    template <class F>
    bool forEachFace(F const& f) const noexcept
    {
        // Remove duplicates and sort by lexicographical ordering.
        // In particular, the first element is a left-most vertex while the last right-most.
        struct Vec2LexLess {
            bool operator()(vertex_type const& lhs, vertex_type const& rhs)
            {
                return lhs(0)<rhs(0) || (lhs(0)==rhs(0) && lhs(1)<rhs(1));
            }
        };
        std::set<vertex_type,Vec2LexLess> nubpts(
            std::begin(BaseType::m_pts),
            std::end(BaseType::m_pts) );

        // Return immediately if there are only at most one essential vertices.
        if(nubpts.size() <= 1)
            return true;

        // Rotation matrix for 90 degree in clockwise.
        Eigen::Matrix2d rotR;
        rotR << 0.0, 1.0, -1.0, 0.0;

        // The iterator pointing to a vertex from which a face begins.
        auto itr = nubpts.begin();
        // The tangent vector to a face so that every control point lie on the left.
        Eigen::Vector2d vec(0.0, -1.0);

        do {
            // Find the next vertex
            auto itr_next = std::max_element(
                nubpts.begin(),
                nubpts.end(),
                [&itr,&vec](vertex_type const& lhs, vertex_type const& rhs) {
                    if (lhs == *itr)
                        return rhs != *itr;
                    else if (rhs == *itr)
                        return false;
                    else
                        return vec.dot(lhs-*itr)/(lhs-*itr).norm()
                            < vec.dot(rhs-*itr)/(rhs-*itr).norm();
                } );

            if(itr_next == itr) {
                // Error
                return false;
            }

            // Compute the tangent vector to the current face.
            vec = *itr_next - *itr;
            vec.normalize();

            // Execute the operation
            f(*itr, rotR*vec);

            // Go to next vertex.
            itr = itr_next;
        } while(itr != nubpts.begin());

        return true;
    }

    //! Check if the convex hull of control points overlaps with that of another Bezier curve.
    //! Here, so-called *The Separating Axis Theorem* is used.
    template <size_t N>
    bool hasHullIntersection(Bezier2D<N> const& other) const noexcept
    {
        bool is_separated = false;

        forEachFace(
            [&is_separated,&other](vertex_type const& pt, vertex_type const& nv) {
                is_separated |= std::accumulate(
                    std::begin(other.m_pts),
                    std::end(other.m_pts),
                    true,
                    [&pt, &nv](bool p, vertex_type const& o_pt) {
                        return p && nv.dot(o_pt) > 0.0;
                    } );
            } );
        other.forEachFace(
            [&is_separated,this](vertex_type const& pt, vertex_type const& nv) {
                is_separated |= std::accumulate(
                    std::begin(this->m_pts),
                    std::end(this->m_pts),
                    true,
                    [&pt, &nv](bool p, vertex_type const& this_pt) {
                        return p && nv.dot(this_pt) > 0.0;
                    } );
            } );

        return !is_separated;
    }
};

//! Compute the intersections with another 2d bezier curve.
//! \return The list of parameters where two bezier curves intersect. As for each element of the vector, the first component is the parameter for *this* class while the second is the one for *other*.
template <size_t M, size_t N>
auto intersect(Bezier2D<M> const& lhs, Bezier2D<N> const& rhs) noexcept
    -> std::vector<std::array<double,2>>
{
    using LHSBezier = Bezier2D<M>;
    using RHSBezier = Bezier2D<N>;
    struct Interval{
        double beg, end;
    };

    std::vector<std::pair<LHSBezier,RHSBezier>> bezier_stack{ std::make_pair(lhs,rhs) };
    decltype(bezier_stack) bezier_aux;
    std::vector<std::array<Interval,2>> interval_stack{ std::make_pair(Interval{0.0, 1.0}, Interval{0.0, 1.0} ) };
    decltype(interval_stack) interval_aux;

    constexpr size_t max_steps = 64;
    for(size_t i = 0; i < max_steps; ++i) {
        bezier_aux.clear();
        interval_aux.clear();
        std::transform(
            bezier_stack.begin(),
            bezier_stack.end(),
            interval_stack,
            bord2::NullOutIterator{},
            [&bezier_aux,&interval_aux](std::pair<LHSBezier,RHSBezier> const& bez, std::array<Interval,2> const& interval) -> void {
                if(!bez.first.hasHullIntersection(bez.second))
                    return;
                LHSBezier lhsdiv{bez.first.divide()};
                RHSBezier rhsdiv{bez.second.divide()};
                bezier_aux.emplace_back(lhsdiv.first, rhsdiv.first);
                bezier_aux.emplace_back(lhsdiv.second, rhsdiv.second);
                interval_aux.push_back({
                        {interval[0].beg, interval[0].beg/2 + interval[0].end/2},
                        {interval[1].beg, interval[1].beg/2 + interval[1].end/2}
                    } );
                interval_aux.push_back({
                        {interval[0].beg/2 + interval[0].end/2, interval[0].end},
                        {interval[1].beg/2 + interval[1].end/2, interval[1].end}
                    });
            } );
        std::swap(bezier_stack, bezier_aux);
        std::swap(interval_stack, interval_aux);
    }

    std::vector<std::array<double,2>> result{};
    std::transform(
        interval_stack.begin(),
        interval_stack.end(),
        std::back_inserter(result),
        [](std::array<Interval,2> const& interval) -> std::array<double,2> {
            return { interval[0].beg/2 + interval[0].end/2,
                     interval[1].beg/2 + interval[1].end/2 };
        } );
    return result;
}
