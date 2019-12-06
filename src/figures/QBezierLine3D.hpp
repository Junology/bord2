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

#include "../PathFigure.hpp"

class QBezierLine3D : public PathFigure3D
{
protected:
    //! Three control points
    Eigen::Vector3d m_pts[3];
    //! View Origin
    Eigen::Vector3d m_focus;
    //! Projection matrix
    Eigen::Matrix<double,2,3> m_mat_proj;

public:
    using vec3d_type = typename std::array<double, 3>;
    QBezierLine3D(vec3d_type const &init, vec3d_type const &ctrl, vec3d_type const &term);
    virtual ~QBezierLine3D() = default;

    //! Methods inherited from PathFigure
    void draw(PathScheme&) override;

    //! Methods inherited from PathFigure3D
    void setAngles(double elevation, double azimuth) override;

    //! The method to set the center of the view.
    void setFocus(double x, double y, double z) override;
};
