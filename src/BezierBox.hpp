/*!
 * \file BezierBox.hpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Januray 16, 2020: created
 */

#pragma once

#include <vector>

#include "math/Bezier.hpp"
#include "PathScheme.hpp"

template <class T>
class BezierBox : public PathFigure<T>
{
public:
    using BezierT = Bezier<T,3>;

private:
    std::vector<BezierT> m_bezv;

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
