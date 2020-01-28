/*!
 * \file AffHypPlane.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 13, 2019: created
 */

#include "AffHypPlane.hpp"

// #include <iostream> // Debug

//! Compute an intersection with another hyper plane in the special case of dimension 2.
template <> template<>
auto AffHypPlane<2>::intersect<2>(AffHypPlane<2> const &another) const
    -> std::pair<bool,typename AffHypPlane<2>::VecT>
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
