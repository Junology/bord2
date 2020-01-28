/*!
 * \file PathFigure3D.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 12, 2019: created
 */

#pragma once

#include <Eigen/Dense>

#include "../PathScheme.hpp"

class PathFigure3D : public PathFigure<Eigen::Vector3d>
{
public:
    PathFigure3D() = default;
    virtual ~PathFigure3D() = default;

    virtual void updateProjector(Eigen::Matrix<double,2,3> const &proj) {}
};
