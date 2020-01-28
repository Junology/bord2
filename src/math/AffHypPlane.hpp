/*!
 * \file AffHypPlane.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 9, 2019: created
 */

#pragma once

// #include <iostream> // only for Debug
#include <utility>
#include <tuple>
#include <array>
#include <Eigen/Dense>

#include "../utils.hpp"

template<size_t n>
class AffHypPlane
{
public:
    /*static constexpr size_t*/ enum : size_t { dim = n };
    static constexpr double threshold = 1e-14;
    using VecT = typename Eigen::Matrix<double,dim,1>;

private:
    //! The defining function of the line is
    //!   <m_normal|x> + m_c = 0;
    VecT m_normal;
    double m_c;

    template<size_t... is>
    constexpr AffHypPlane(std::array<double,dim> const & normal, double c, std::index_sequence<is...>)
        : m_normal(normal[is]...), m_c(c)
    {}

public:
    constexpr AffHypPlane(std::array<double,dim> const & normal, double c)
    : AffHypPlane<n>(normal, c, std::make_index_sequence<n>())
    {}

    constexpr AffHypPlane(VecT const &normal, VecT const &refpt)
        : m_normal(normal), m_c(-normal.adjoint()*refpt)
    {}

    AffHypPlane(AffHypPlane<n> const &) = default;
    AffHypPlane(AffHypPlane<n> &&) = default;
    ~AffHypPlane() = default;

    //! Compute an intersection with another hyper plane.
    //! The function is specialized in case of dimension 2.
    template <size_t m= dim>
    auto intersect(AffHypPlane<m> const &another) const
        -> std::pair<bool,VecT>
    {
        Eigen::Matrix<double,2,dim> A;
        A.row(0) = m_normal;
        A.row(1) = another.m_normal;
        Eigen::Vector2d b{-m_c, -another.m_c};

        Eigen::Matrix<double,dim,1> x = A.colPivHouseholderQr().solve(b);

        return {(A*x-b).norm()/(b.norm()+m_normal.norm()) < threshold, x};
    }

    //! Compute the height of a given point from the hyperplane.
    //! \warning: The value is not normalized; or with respect to the normal vector of the hyperplane.
    constexpr double height(VecT const &v) const
    {
        return static_cast<double>(m_normal.adjoint()*v) + m_c;
    }

    template <class... Ts>
    constexpr auto height(Ts&&... xs) const
        -> std::enable_if_t<sizeof...(Ts)==dim && bord2::all_of<std::is_convertible<Ts,double>::value...>::value, double>
    {
        return height(VecT(std::forward<double>(xs)...));
    }
};

template<size_t n>
constexpr double AffHypPlane<n>::threshold;
