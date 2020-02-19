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

public:
    AffHypPlane(std::array<double,dim> const & normal, double c)
        : m_normal(Eigen::Map<const VecT>(&normal[0], dim)), m_c(c)
    {}

    AffHypPlane(VecT const &normal, VecT const &refpt)
        : m_normal(normal), m_c(-normal.dot(refpt))
    {}

    AffHypPlane(AffHypPlane<n> const &) = default;
    AffHypPlane(AffHypPlane<n> &&) = default;

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
    double height(VecT const &v) const
    {
        return m_normal.dot(v) + m_c;
    }

    template <class... Ts>
    double height(double x, Ts... xs) const
    {
        static_assert(1+sizeof...(Ts)==dim, "Wrong number of arguments.");
        static_assert(
            bord2::allTrue({std::is_convertible<Ts,double>::value...}),
            "The arguments must be convertible to double." );
        return height(VecT(x, xs...));
    }
};

template<size_t n>
constexpr double AffHypPlane<n>::threshold;
