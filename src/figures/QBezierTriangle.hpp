/*!
 * \file QBezierTriangle.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#pragma once

#include <array>
#include <Eigen/Dense>

#include "PathFigure3D.hpp"

class QBezierTriangle : public PathFigure3D
{
private:
    Eigen::Vector3d m_vert[3], m_edge[3];

public:
    QBezierTriangle(std::array<Vec3dT,3> const &vert, std::array<Vec3dT,3> const &edge)
        : m_vert{ Eigen::Vector3d(vert[0][0], vert[0][1], vert[0][2]),
                  Eigen::Vector3d(vert[1][0], vert[1][1], vert[1][2]),
                  Eigen::Vector3d(vert[2][0], vert[2][1], vert[2][2]) },
          m_edge{ Eigen::Vector3d(edge[0][0], edge[0][1], edge[0][2]),
                  Eigen::Vector3d(edge[1][0], edge[1][1], edge[1][2]),
                  Eigen::Vector3d(edge[2][0], edge[2][1], edge[2][2]) }
    {}

    virtual ~QBezierTriangle() = default;

    void draw(PathScheme &scheme) const override;
};

