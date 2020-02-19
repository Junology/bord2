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
          std::is_constructible<vertex_type, U&&>::value
          && !std::is_same<Bezier<T,n>,std::decay_t<U> >::value,
          bool
          > = true
    >
    constexpr Bezier(U && pt0, Ts &&... pts) noexcept(std::is_nothrow_constructible<vertex_type, U&&>::value && bord2::allTrue({std::is_nothrow_constructible<vertex_type, Ts&&>::value...}))
      : m_pts{std::forward<U>(pt0), std::forward<Ts>(pts)...}
    {}

    constexpr Bezier(Bezier const &) noexcept(std::is_nothrow_copy_constructible<vertex_type>::value) = default;
    constexpr Bezier(Bezier &&) noexcept(std::is_nothrow_move_constructible<vertex_type>::value) = default;
    constexpr Bezier& operator=(Bezier const&) noexcept(std::is_nothrow_copy_assignable<vertex_type>::value) = default;
    constexpr Bezier& operator=(Bezier &&) noexcept(std::is_nothrow_move_assignable<vertex_type>::value) = default;

    //! Get a control point.
    template<size_t i>
    constexpr auto get() const noexcept
        -> std::enable_if_t<(i<num_pts), vertex_type const&>
    {
        return m_pts[i];
    }

    //! Evaluate the point at parameter t.
    constexpr vertex_type eval(double t) const noexcept {
        return eval_part<0,degree>(t);
    }

    //! Check if all the control points satisfy a given condition
    template <class F>
    bool allSatisfy(F const& f) const noexcept {
        static_assert(
            std::is_convertible<
                decltype(f(std::declval<vertex_type const&>())),
                bool
            >::value, "The condition has an illegal signature.");
        for(size_t i = 0; i < num_pts; ++i) {
            if (!f(m_pts[i])) return false;
        }
        return true;
    }

    //! Divide the Bezier curve using De Casteljau's algorithm.
    constexpr auto divide(double t = 0.5, double eps = 0.0) const noexcept
        -> std::pair< Bezier<vertex_type,degree>,Bezier<vertex_type,degree> >
    {
        return divide_impl(std::make_index_sequence<num_pts>(), t, eps);
    }

    //! Clip the Bezier curve to the one with the parameter on a given interval.
    //! \warning May cause 0 division when t0 = t1.
    constexpr auto clip(double t0, double t1) const noexcept
        -> Bezier<vertex_type, degree>
    {
        return clip_impl(std::make_index_sequence<num_pts>(), t0, t1);
    }

    //! Convert into Bezier curve of the same degree while with another vertex_type.
    template <class F>
    auto convert(F&& f) const
        -> Bezier<decltype(std::declval<F&&>()(std::declval<vertex_type&&>())),degree>
    {
        return convert_impl(std::make_index_sequence<num_pts>(), std::forward<F>(f));
    }

protected:
    template <size_t MIN, size_t MAX, size_t DEG=MAX-MIN>
    constexpr vertex_type eval_part(double t) const noexcept {
        static_assert(MAX<=degree && MIN <= MAX, "Index out of range");

        constexpr auto binomArr = bord2::binom<>::getArray<double,DEG>();
        vertex_type result = bord2::cipow(1.0-t,DEG)* m_pts[MIN];
        for(size_t i = 1; i <= DEG; ++i)
            result = std::move(result)
                + bord2::cipow(1.0-t,DEG-i)*bord2::cipow(t,i)*binomArr[i]*m_pts[MIN+i];
        return result;
    }

    template<size_t... Is>
    constexpr auto divide_impl(std::index_sequence<Is...>, double t, double eps) const noexcept
        -> std::pair< Bezier<vertex_type,degree>,Bezier<vertex_type,degree> >
    {
        static_assert(sizeof...(Is)==num_pts, "Wrong number of arguments");

        return std::make_pair(
            Bezier<vertex_type,degree>(eval_part<0,Is>(t+eps)...),
            Bezier<vertex_type,degree>(eval_part<Is,degree>(t-eps)...) );
    }

    template<size_t... Is>
    constexpr auto clip_impl(std::index_sequence<Is...>, double t0, double t1) const noexcept {
        auto aux = Bezier<vertex_type, degree>{eval_part<0,Is>(t1)...};
        double t = t0/t1;
        return Bezier<vertex_type, degree>(
            aux.template eval_part<Is,degree>(t)...
            );
    }

    template <class F, size_t... Is>
    auto convert_impl(std::index_sequence<Is...>, F&& f) const
        -> Bezier<decltype(std::declval<F&&>()(std::declval<vertex_type&&>())),degree>
    {
        return Bezier<decltype(std::declval<F&&>()(std::declval<vertex_type&&>())),degree>{
            f(m_pts[Is])...
        };
    }
};
