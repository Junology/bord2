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

//#include <iostream> // Debug

/*! The class for Bezier curves.
 * \tparam T The type of vertices; e.g. Eigen::Vector2d, Eigen::Vector3d. It must be possible to
 * - add two vertices;
 * - multiply vertices by floating points, i.e. double.
 * \tparam n The degree of the Bezier curve. Hence, the number of control points would be n+1.
 */
template <class T, size_t n>
class Bezier
{
public:
    //! The type of control points.
    using vertex_type = T;
    enum : size_t {
        //! The degree of the Bezier curve.
        degree = n,
        //! The number of control points.
        num_pts = n+1
    };

protected:
    std::array<vertex_type, num_pts> m_pts;

public:
    template <
      class U,
      class... Ts,
      std::enable_if_t<
          std::is_constructible<vertex_type, U>::value
          && !std::is_same<Bezier<T,n>,std::decay_t<U> >::value,
          bool
          > = true
    >
    constexpr Bezier(U && pt0, Ts &&... pts)
      : m_pts{std::forward<U>(pt0), std::forward<Ts>(pts)...}
    {}

    constexpr Bezier(Bezier<vertex_type,n> const &) = default;
    constexpr Bezier(Bezier<vertex_type,n> &&) = default;

    virtual ~Bezier() = default;

    //! Get a control point.
    template<size_t i>
    constexpr auto get() const
        -> std::enable_if_t<(i<num_pts), vertex_type const&>
    {
        return m_pts[i];
    }

    //! Evaluate the point at parameter t.
    constexpr vertex_type eval(double t) const {
        constexpr auto binomArr = bord2::binom<>::getArray<double,degree>();
        vertex_type result = bord2::cipow(1-t,degree) * m_pts[0];

        for(size_t i = 1; i < num_pts; ++i)
            result = result + bord2::cipow(1.0-t,degree-i)*bord2::cipow(t,i)*binomArr[i]*m_pts[i];
        return result;
    }

    //! Divide the Bezier curve using De Casteljau's algorithm.
    constexpr auto divide() const -> std::pair< Bezier<T,n>,Bezier<T,n> >
    {
        return {
            divide_forth_impl(std::make_index_sequence<num_pts>()),
            divide_latter_impl(std::make_index_sequence<num_pts>())
        };
    }

protected:
    constexpr vertex_type weightedSum(std::array<double,num_pts> const &weights) const {
        // Note that num_pts is always positive.
        vertex_type result = weights[0]*m_pts[0];

        for(size_t i = 1; i < num_pts; ++i)
            result = result + weights[i]*m_pts[i];

        return result;
    }

    template<size_t... is>
    constexpr auto divide_forth_impl(std::index_sequence<is...>) const
        -> std::array<vertex_type, num_pts>
    {
        constexpr std::array<double,sizeof...(is)>
            coeff{bord2::cipow(0.5,is)...};
        constexpr std::array<std::array<double,num_pts>,sizeof...(is)>
            weights{bord2::binom<>::getArray<double,is,num_pts>()...};

        return {coeff[is]*weightedSum(weights[is])...};
    }

    template<size_t... is>
    constexpr auto divide_latter_impl(std::index_sequence<is...>) const
        -> std::array<vertex_type, num_pts>
    {
        constexpr std::array<double,sizeof...(is)>
            coeff{bord2::cipow(0.5,is)...};
        constexpr std::array<std::array<double,num_pts>,sizeof...(is)>
            weights{bord2::binom<>::getArrayRev<double,is,num_pts>()...};

        return {coeff[num_pts-is-1]*weightedSum(weights[num_pts-is-1])...};
    }
};

