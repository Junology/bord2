/*!
 * \file PathFigure.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#pragma once

#include <array>
#include <Eigen/Dense>

#include "../PathScheme.hpp"

//! Base class for figures in 3-dimensional spaces.
//! They are supposed to be drawn linearly projected to a plane.
class PathFigure3D : public PathFigure
{
private:
    Eigen::Matrix<double,2,3> m_mat_proj;
    Eigen::Vector3d m_focus;

public:
    using Vec3dT = typename std::array<double, 3>;

    PathFigure3D(Vec3dT const &focus = {0.0, 0.0, 0.0}, double elev = 0.0, double azim = 0.0 )
        : m_focus(focus[0], focus[1], focus[2]), m_mat_proj()
    {
        PathFigure3D::setAngles(elev, azim);
    }

    virtual ~PathFigure3D() = default;

    //! The method to set angles of the viewpoint in degrees.
    virtual void setAngles(double elevation, double azimuth) {
        double t = M_PI*elevation/180.0;
        double u = M_PI*azimuth/180.0;

        m_mat_proj <<
            cos(u),       -sin(u), 0,
         // cos(t)*sin(u), cos(t)*cos(u), -sin(t),
            sin(t)*sin(u), sin(t)*cos(u), cos(t);
    }

    //! The method to set the center of the view.
    virtual void setFocus(double x, double y, double z) {
        m_focus(0) = x;
        m_focus(1) = y;
        m_focus(2) = z;
    }

public:
    Eigen::Matrix<double,2,3> const & getProjector() const {
        return m_mat_proj;
    }

    Eigen::Vector3d const & getFocus() const {
        return m_focus;
    }
};
