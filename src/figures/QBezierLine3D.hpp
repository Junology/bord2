/*!
 * \file QBezierLine3D.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#pragma once

#include <array>
#include <Eigen/Dense>

#include "PathFigure3D.hpp"

class QBezierLine3D : public PathFigure3D
{
protected:
    //! Three control points
    Eigen::Vector3d m_init, m_ctrl, m_term;

public:
    using vec3d_type = typename std::array<double, 3>;
    QBezierLine3D(vec3d_type const &init, vec3d_type const &ctrl, vec3d_type const &term)
        : PathFigure3D(),
          m_init(init[0], init[1], init[2]),
          m_ctrl(ctrl[0], ctrl[1], ctrl[2]),
          m_term(term[0], term[1], term[2])
    {}

    virtual ~QBezierLine3D() = default;

    //! Methods inherited from PathFigure
    void draw(PathScheme &scheme) const override {
        auto orig = scheme.getCenter();
        Eigen::Vector2d orig2d(orig[0], orig[1]);

        auto mat_proj = PathFigure3D::getProjector();
        auto focus = PathFigure3D::getFocus();
        Eigen::Vector2d init2d = mat_proj * (m_init - focus) + orig2d;
        Eigen::Vector2d ctrl2d = mat_proj * (m_ctrl - focus) + orig2d;
        Eigen::Vector2d term2d = mat_proj * (m_term - focus) + orig2d;

        scheme.moveTo(init2d(0), init2d(1));
        scheme.qbezierTo(ctrl2d(0), ctrl2d(1), term2d(0), term2d(1));
        scheme.stroke();
    }
};
