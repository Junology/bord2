/*!
 * \file QBezierLine3D.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#pragma once

#include <Eigen/Dense>

#include "../math/Bezier.hpp"
#include "../PathScheme.hpp"
#include "PathFigure3D.hpp"

class QBezierLine3D : public PathFigure3D, public Bezier<Eigen::Vector3d,2>
{
public:
    using BezierT = Bezier<Eigen::Vector3d,2>;

private:
    //! Three control points
    Eigen::Vector3d m_init, m_ctrl, m_term;

public:
    QBezierLine3D(std::array<double,3> const &init, std::array<double,3> const &ctrl, std::array<double,3> const &term)
        : BezierT(
            Eigen::Vector3d(init[0], init[1], init[2]),
            Eigen::Vector3d(ctrl[0], ctrl[1], ctrl[2]),
            Eigen::Vector3d(term[0], term[1], term[2])
            )
    {}

    virtual ~QBezierLine3D() = default;

    //! Methods inherited from PathFigure
    void draw(SchemeType &scheme) const override {
        scheme.moveTo(BezierT::get<0>());
        scheme.qbezierTo(BezierT::get<1>(), BezierT::get<2>());
        scheme.stroke();
    }
};
