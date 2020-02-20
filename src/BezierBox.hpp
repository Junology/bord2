/*!
 * \file BezierBox.hpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Januray 16, 2020: created
 */

#pragma once

#include <vector>

#include <Eigen/Dense>

#include "math/Bezier.hpp"
#include "math/Bezier2D.hpp"
#include "PathScheme.hpp"
#include "ProjSpatialScheme.hpp"

template <class T>
class BezierBox
    : public ProjSpatialScheme<T>
{
    static_assert(std::is_base_of<T, PathScheme<Eigen::Vector2d>>::value, "The base scheme must be derived from PathScheme<Eigen::Vector2d>.");

public:
    using typename PathScheme<Eigen::Vector3d>::PathElement;

    struct BezierElement {
        bord2::PathElemType type;
        union {
            Bezier<Eigen::Vector3d,1> line;
            Bezier<Eigen::Vector3d,2> qbez;
            Bezier<Eigen::Vector2d,3> cbez;
        };
    };
private:
    std::vector<PathElement> m_bezv;

public:
    void append(BezierT && bez) {
        m_bezv.push_back(std::forward<BezierT>(bez));
    }

    virtual void draw(typename PathFigure<T>::SchemeType &scheme) override {
        for (auto bez : m_bezv) {
            scheme.moveTo(bez.get<0>());
            scheme.bezierTo(bez.get<1>(), bez.get<2>(), bez.get<3>());
            scheme.stroke();
        }
    }
};
