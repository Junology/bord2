/*!
 * \file PathFigure.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#pragma once

#include "PathScheme.hpp"

//! Base class for figures which draw themselves using PathScheme.
class PathFigure {
public:
    PathFigure() = default;
    virtual ~PathFigure() = default;

    virtual void draw(PathScheme&) = 0;
};

//! Base class for figures in 3-dimensional spaces.
//! They are supposed to be drawn linearly projected to a plane.
class PathFigure3D : public PathFigure
{
public:
    PathFigure3D() = default;
    virtual ~PathFigure3D() = default;

    //! The method to set angles of the viewpoint in degrees.
    virtual void setAngles(double elevation, double azimuth) = 0;

    //! The method to set the center of the view.
    virtual void setFocus(double x, double y, double z) = 0;
};
