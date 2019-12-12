/*!
 * \file Bezier.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 12, 2019: created
 */

#pragma once

#include <utility>
#include <type_traits>
#include <array>

#include "../utils.hpp"

/*! The class for Bezier curves.
 * \tparam T The type of vertices; e.g. Eigen::Vector2d, Eigen::Vector3d. It must be possible to
 * - add two vertices;
 * - multiply vertices by floating points, i.e. double.
 * \tparam n The degree of the Bezier curve. Hence, the number of control points would be n+1.
 */
template <
    class T,
    size_t n,
    class = std::pair<
        decltype(std::declval<T>()+std::declval<T>()),
        decltype(std::declval<double>()*std::declval<T>())
        >
    >
class Bezier
{
public:
    using vertex_type = T;
    static constexpr size_t degree = n;
    static constexpr size_t num_pts = n+1;

protected:
    std::array<vertex_type, num_pts> m_pts;

public:
    template <class... Ts>
    Bezier(Ts &&... pts)
        : m_pts({std::forward<Ts>(pts)...})
    {}

    ~Bezier() = default;

    //! Get a control point.
    template<size_t i>
    auto get() const
        -> std::enable_if_t<(i<num_pts), vertex_type const&>
    {
        return m_pts[i];
    }

    //! Divide the Bezier curve using De Casteljau's algorithm.
    auto divide() -> std::pair< Bezier<T,n>,Bezier<T,n> >
    {
        return {
            divide_forth_impl(std::make_index_sequence<num_pts>()),
            divide_latter_impl(bord2::make_reversed_index_seq<num_pts>())
        };
    }

protected:
    vertex_type weightedSum(std::array<double,num_pts> const &weights) {
        if (num_pts <= 0)
            return vertex_type{};

        vertex_type result = weights[0]*m_pts[0];

        for(size_t i = 1; i < num_pts; ++i)
            result = result + weights[i]*m_pts[i];

        return result;
    }

    template<size_t... is>
    auto divide_forth_impl(std::index_sequence<is...>)
        -> std::array<vertex_type, num_pts>
    {
        return {weightedSum(bord2::binom<>::getArray<double,is,num_pts>())...};
    }

    template<size_t... is>
    auto divide_latter_impl(std::index_sequence<is...>)
        -> std::array<vertex_type, num_pts>
    {
        return {weightedSum(bord2::binom<>::getArrayRev<double,is,num_pts>())...};
    }
};