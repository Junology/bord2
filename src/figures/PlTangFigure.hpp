/*!
 * \file PlTangFigure.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 29, 2020: created
 */

#pragma once

#include <Eigen/Dense>

#include "../PlTang.hpp"
#include "../PathScheme.hpp"

template <size_t R, size_t C>
class PlTangFigure : public PathFigure<Eigen::Vector2d>
{
private:
    PlTang<R,C> m_pltang;
    Eigen::Vector2d m_baseX, m_baseY;

public:
    PlTangFigure(PlTang<R,C> const& pltang, Eigen::Vector2d const& baseX, Eigen::Vector2d const& baseY) noexcept
        : m_pltang(pltang), m_baseX(baseX), m_baseY(baseY)
    {}

    ~PlTangFigure() noexcept = default;

    virtual void draw(SchemeType &scheme) const override
    {
        for(size_t j=0; j < m_pltang.vlength(); ++j) {
            Eigen::Vector2d const& baseX = m_baseX;
            Eigen::Vector2d const& baseY = m_baseY;
            m_pltang.forElTang(
                j,
                [&scheme, &j, baseX, baseY](size_t i, char c) {
                    auto orig = i*baseX + j*baseY;
                    switch(c) {
                    case '-':
                        scheme.lineTo(orig + baseX + baseY/2);
                        break;

                    case '|':
                        scheme.moveTo(orig + baseX/2);
                        scheme.lineTo(orig + baseX/2 + baseY);
                        break;

                    case '7':
                        scheme.qbezierTo(
                            orig + baseX/2+ baseY/2,
                            orig + baseX/2+ baseY);
                        break;

                    case 'r':
                        scheme.moveTo(orig + baseX/2 + baseY);
                        scheme.qbezierTo(
                            orig + baseX/2 + baseY/2,
                            orig + baseX + baseY/2);
                        break;

                    case 'L':
                        scheme.moveTo(orig + baseX/2);
                        scheme.qbezierTo(
                            orig + baseX/2 + baseY/2,
                            orig + baseX + baseY/2);
                        break;

                    case 'J':
                        scheme.qbezierTo(
                            orig + baseX/2 + baseY/2,
                            orig + baseX/2);
                        break;
                    }
                });
            scheme.stroke();
        }
    }
};
