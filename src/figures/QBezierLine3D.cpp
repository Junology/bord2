/*!
 * \file QBezierLine3D.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#define _USE_MATH_DEFINES

#include "QBezierLine3D.hpp"

#include <iostream>
#include <cmath>

QBezierLine3D::QBezierLine3D(vec3d_type const &init, vec3d_type const &ctrl, vec3d_type const &term)
    : m_pts{ Eigen::Vector3d(init[0], init[1], init[2]),
             Eigen::Vector3d(ctrl[0], ctrl[1], ctrl[2]),
             Eigen::Vector3d(term[0], term[1], term[2])
        },
      m_focus(0.0, 0.0, 0.0),
      m_mat_proj()
{
    QBezierLine3D::setAngles(0.0, 0.0);
}

void QBezierLine3D::draw(PathScheme &scheme)
{
    auto orig = scheme.getCenter();
    Eigen::Vector2d orig2d(orig.first, orig.second);
    Eigen::Vector2d init2d = m_mat_proj * (m_pts[0] - m_focus) + orig2d;
    Eigen::Vector2d ctrl2d = m_mat_proj * (m_pts[1] - m_focus) + orig2d;
    Eigen::Vector2d term2d = m_mat_proj * (m_pts[2] - m_focus) + orig2d;

    /* Debug
    std::cout << "Debug (" << __FILE__ << ":" << __LINE__ << ")" << std::endl;
    std::cout << init2d << std::endl;
    std::cout << ctrl2d << std::endl;
    std::cout << term2d << std::endl;
    // */

    scheme.moveTo(init2d(0), init2d(1));
    scheme.qbezierTo(ctrl2d(0), ctrl2d(1), term2d(0), term2d(1));
    scheme.stroke();
}

void QBezierLine3D::setAngles(double elev, double azim)
{
    double t = M_PI*elev/180.0;
    double u = M_PI*azim/180.0;

    m_mat_proj <<
               cos(u),       -sin(u), 0,
     // cos(t)*sin(u), cos(t)*cos(u), -sin(t),
        sin(t)*sin(u), sin(t)*cos(u), cos(t);
}

void QBezierLine3D::setFocus(double x, double y, double z)
{
    m_focus(0) = x;
    m_focus(1) = y;
    m_focus(2) = z;
}
