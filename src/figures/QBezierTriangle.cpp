/*!
 * \file QBezierTriangle.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 21, 2019: created
 */

#include "QBezierTriangle.hpp"
#include "../math/misc.hpp"
#include "../math/QuadraticCurve.hpp"

//#include <iostream> // Debug

void QBezierTriangle::draw(SchemeType &scheme) const
{
    scheme.moveTo(m_vert[0]);

    scheme.setPen(0.5, bord2::Gray);
    // Draw the edge lines.
    for(size_t i = 1; i <= 3; ++i) {
        scheme.qbezierTo(m_edge[(i+1)%3], m_vert[i%3]);
    }
    scheme.stroke();

    // Draw the ridgelines.
    if(!m_reliable)
        scheme.setPen(2.0, bord2::Red);
    else
        scheme.setPen(1.0, bord2::Black);

    for(auto ridge : m_ridges) {
        scheme.moveTo(ridge.get<0>());
        scheme.bezierTo(
            ridge.get<1>(),
            ridge.get<2>(),
            ridge.get<3>() );
    }
    scheme.stroke();
}

void QBezierTriangle::updateProjector(Eigen::Matrix<double,2,3> const &proj)
{
    // Empty the ridgeline buffer
    m_ridges.clear();

    // Tangent vector factors;
    // i.e. the tangent space at p(s[0],s[1]) is spanned by
    // 2.0 * m1 * affWrap(s) and 2.0 * m2 * affWrap(s)
    Eigen::Matrix3d m1, m2;

    m1 <<
        m_vert[0] + m_vert[1] - 2.0*m_edge[2],
        m_vert[0] + m_edge[0] - m_edge[1] - m_edge[2],
        m_edge[2] - m_vert[0];
    m2 <<
        m_vert[0] + m_edge[0] - m_edge[1] - m_edge[2],
        m_vert[0] + m_vert[2] - 2.0*m_edge[1],
        m_edge[1] - m_vert[0];

    /* Debug
    std::cout << "==" << __FILE__ << __LINE__ << std::endl;
    std::cout << m_vert[0] + m_vert[1] - 2.0*m_edge[2] << std::endl;
    std::cout << "--" << std::endl;
    std::cout << m_vert[0] + m_edge[0] - m_edge[1] - m_edge[2] << std::endl;
    std::cout << "--" << std::endl;
    std::cout << m_edge[2] - m_vert[0] << std::endl;
    std::cout << "--" << std::endl;
    std::cout << m1 << std::endl;
    // */

    Eigen::Matrix<double,2,3> m12d = proj*m1;
    Eigen::Matrix<double,2,3> m22d = proj*m2;

    // The singular locus as a quadratic curve.
    QuadraticCurve singlocus{
        cross2D(m12d.col(0), m22d.col(0)),
        cross2D(m12d.col(1), m22d.col(1)),
        cross2D(m12d.col(2), m22d.col(2)),
        cross2D(m12d.col(0), m22d.col(1))
            + cross2D(m12d.col(1), m22d.col(0)),
        cross2D(m12d.col(0), m22d.col(2))
            + cross2D(m12d.col(2), m22d.col(0)),
        cross2D(m12d.col(1), m22d.col(2))
            + cross2D(m12d.col(2), m22d.col(1)) };

    auto parambezs = singlocus.onTriangle(
        { Eigen::Vector2d(1.0, 0.0),
                Eigen::Vector2d(0.0, 0.0),
                Eigen::Vector2d(0.0, 1.0) });

    m_reliable = parambezs.second;
    for(auto bez : parambezs.first) {
        Eigen::Vector2d pdv1 = bez.get<1>() - bez.get<0>();
        Eigen::Vector2d pdv2 = bez.get<2>() - bez.get<3>();
        Eigen::Vector3d from = eval(bez.get<0>()(0), bez.get<0>()(1));
        Eigen::Vector3d to = eval(bez.get<3>()(0), bez.get<3>()(1));
        Eigen::Vector3d c1
            = from
            + pdv1(0) * 2.0*m1 * affWrap<double,2>(bez.get<0>())
            + pdv1(1) * 2.0*m2 * affWrap<double,2>(bez.get<0>());
        Eigen::Vector3d c2
            = to
            + pdv2(0) * 2.0*m1 * affWrap<double,2>(bez.get<3>())
            + pdv2(1) * 2.0*m2 * affWrap<double,2>(bez.get<3>());
        m_ridges.emplace_back(from,c1,c2,to);
    }
}
