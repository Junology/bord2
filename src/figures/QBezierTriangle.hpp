/*!
 * \file QBezierTriangle.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>

#include "../math/Bezier.hpp"
#include "PathFigure3D.hpp"

class QBezierTriangle : public PathFigure3D
{
private:
    std::array<Eigen::Vector3d,3> m_vert, m_edge;
    std::vector<Bezier<Eigen::Vector3d,3> > m_ridges;
    bool m_reliable;

public:
    QBezierTriangle(std::array<vertex_type,3> const &vert, std::array<vertex_type,3> const &edge)
        : m_vert{ Eigen::Vector3d(vert[0][0], vert[0][1], vert[0][2]),
                  Eigen::Vector3d(vert[1][0], vert[1][1], vert[1][2]),
                  Eigen::Vector3d(vert[2][0], vert[2][1], vert[2][2]) },
          m_edge{ Eigen::Vector3d(edge[0][0], edge[0][1], edge[0][2]),
                  Eigen::Vector3d(edge[1][0], edge[1][1], edge[1][2]),
                  Eigen::Vector3d(edge[2][0], edge[2][1], edge[2][2]) },
          m_ridges(),
          m_reliable(true)
    {}

    QBezierTriangle(std::array<double,3> const &v0, std::array<double,3> const &v1, std::array<double,3> const &v2, std::array<double,3> const &e0, std::array<double,3> const &e1, std::array<double,3> const &e2)
        : m_vert{ Eigen::Vector3d(v0[0], v0[1], v0[2]),
                  Eigen::Vector3d(v1[0], v1[1], v1[2]),
                  Eigen::Vector3d(v2[0], v2[1], v2[2]) },
          m_edge{ Eigen::Vector3d(e0[0], e0[1], e0[2]),
                  Eigen::Vector3d(e1[0], e1[1], e1[2]),
                  Eigen::Vector3d(e2[0], e2[1], e2[2]) },
          m_ridges(),
          m_reliable(true)
    {}
    virtual ~QBezierTriangle() = default;

    Eigen::Vector3d eval(double t0, double t1, double t2) {
        return t0*t0*m_vert[0]
            + t1*t1*m_vert[1]
            + t2*t2*m_vert[2]
            + 2.0*t0*t1*m_edge[2]
            + 2.0*t1*t2*m_edge[0]
            + 2.0*t2*t0*m_edge[1];
    }

    Eigen::Vector3d eval(double s1, double s2) {
        return eval(1.0-s1-s2, s1, s2);
    }

    /**!
     * \section Member functions derived from PathFigure
     **/
    virtual void draw(SchemeType &scheme) const override;

    /**!
     * \section Member functions derived from PathFigure3d
     **/
    virtual void updateProjector(Eigen::Matrix<double,2,3> const &proj) override;
};

