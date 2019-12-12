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

#include "utils.hpp"

template<size_t n>
class AffHypPlane
{
public:
    static constexpr size_t dim = n;
    static constexpr double threshold = 1e-14;
    using VecT = typename Eigen::Matrix<double,dim,1>;

private:
    //! The defining function of the line is
    //!   <m_normal|x> + m_c = 0;
    VecT m_normal;
    double m_c;

public:
    AffHypPlane(std::array<double,dim> const & normal, double c)
        : m_normal(), m_c(c)
    {
        // We can use initializer_list instead of array since Eigen 3.3.10.
        for(size_t i=0; i<dim; ++i)
            m_normal(i) = normal[i];
    }

    AffHypPlane(VecT const &normal, VecT const &refpt)
        : m_normal(normal), m_c(-normal.adjoint()*refpt)
    {}

    ~AffHypPlane() = default;

    //! Compute an intersection with another hyper plane.
    template <size_t m= dim>
    std::pair<bool,VecT> intersect(AffHypPlane<m> const &another) const
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
        return m_normal.adjoint()*v+m_c;
    }

    template <class... Ts>
    auto height(Ts&&... xs)
        -> std::enable_if_t<sizeof...(Ts)==dim && bord2::all_of<std::is_convertible<Ts,double>::value...>::value, double>
    {
        return height(VecT(std::forward<double>(xs)...));
    }
};

template<size_t n>
constexpr double AffHypPlane<n>::threshold;

//! Compute an intersection with another hyper plane in the special case of dimension 2.
template <> template<>
std::pair<bool,typename AffHypPlane<2>::VecT> AffHypPlane<2>::intersect<2>(AffHypPlane<2> const &another) const
{
    // If two lines pass through the origin, then return the obvious intersection.
    if (std::abs(m_c) < threshold && std::abs(another.m_c) < threshold)
        return {true, VecT(0.0, 0.0)};

    double det = m_normal(0)*another.m_normal(1) - m_normal(1)*another.m_normal(0);

    if (std::abs(det) < threshold) {
        double detx = -m_c*another.m_normal(1)+another.m_c*m_normal(1);
        double dety = -m_c*another.m_normal(0)+another.m_c*m_normal(0);

        /* Debug
        std::cout << __FILE__ << " (" << __LINE__<< "):" << std::endl;
        std::cout << "(" << m_normal(0) << "," << m_normal(1) << "," << m_c << ")" << std::endl;
        std::cout << "(" << another.m_normal(0) << "," << another.m_normal(1) << "," << another.m_c << ")" << std::endl;
        std::cout << "detx=" << detx << std::endl;
        std::cout << "dety=" << dety << std::endl;
        // */

        if (std::abs(detx) > threshold || std::abs(dety) > threshold)
            return {false, VecT()};
        else if (m_normal(0) > threshold)
            return {true, VecT(-m_c/m_normal(0), 0.0)};
        else if (m_normal(1) > threshold)
            return {true, VecT(0.0, -m_c/m_normal(1))};
        else if (another.m_normal(0) > threshold)
            return {true, VecT(-another.m_c/another.m_normal(0), 0.0)};
        else if (another.m_normal(1) > threshold)
            return {true, VecT(0.0, -another.m_c/another.m_normal(1))};
        else
            return {false, VecT(0,0)};
    }
    else {
        double x = (-m_c*another.m_normal(1)+another.m_c*m_normal(1))/det;
        double y = (m_c*another.m_normal(0)-another.m_c*m_normal(0))/det;
        return {true, VecT(x,y)};
    }
}
