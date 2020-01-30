/*!
 * \file TangleMoveFigure.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 29, 2020: created
 */

#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>

#include "../PlTang.hpp"
#include "../PlTangMove.hpp"
#include "PlTangFigure.hpp"
#include "PathFigure3D.hpp"

template <size_t R, size_t C>
class TangleMoveFigure : public PathFigure3D
{
private:
    struct TangleMove {
        using MoveType = PlTang<2,2>;
        MoveType move{};
        size_t x{}, y{};

        constexpr TangleMove() noexcept = default;
        constexpr TangleMove(MoveType const& move0, size_t x0, size_t y0) noexcept
        : move(move0), x(x0), y(y0)
        {}

        constexpr TangleMove(TangleMove const&) noexcept= default;
        ~TangleMove() noexcept = default;

        TangleMove& operator=(TangleMove const&) noexcept = default;
        TangleMove& operator=(TangleMove &&) noexcept = default;
    };

    PlTang<R,C> m_tangInit;
    std::vector<TangleMove> m_moves{};
    Eigen::Vector3d m_baseX, m_baseY;
    // Direction in which ridgelines are critical
    Eigen::Vector3d m_critV;

public:
    TangleMoveFigure(PlTang<R,C> const& tang, Eigen::Vector2d const& baseX, Eigen::Vector2d const& baseY)
        : m_tangInit(tang),
          m_baseX(baseX(0), baseX(1), 0.0),
          m_baseY(baseY(0), baseY(1), 0.0)
    {}

    virtual ~TangleMoveFigure() = default;

    bool push(typename TangleMove::MoveType const& move, size_t x, size_t y) noexcept
    {
        typename TangleMove::MoveType dom
            = m_moves.empty()
            ? m_tangInit.template slice<2,2>(x, y, move.getBefore().hlength(), move.getBefore().vlength())
            : m_moves.back().move.getAfter();

        if(dom == move.getBefore()) {
            m_moves.emplace(move, x, y);
            return true;
        }
        else {
            return false;
        }
    }

    void pop() noexcept
    {
        m_moves.pop_back();
    }

    /**!
     * \section Member functions derived from PathFigure
     **/
    virtual void draw(SchemeType &scheme) const override
    {
        Eigen::Vector3d orig =
            - (static_cast<double>(m_tangInit.hlength())/2.0*m_baseX)
            - (static_cast<double>(m_tangInit.vlength())/2.0*m_baseY)
            - m_moves.size()/(2.0*m_baseX.norm()*m_baseY.norm())*(m_baseX.cross(m_baseY));

        AdapterScheme<Eigen::Vector3d, Eigen::Vector2d> adpscheme{
            &scheme,
                [this, &orig](Eigen::Vector2d const&p) {
                return p(0)*m_baseX + p(1)*m_baseY + orig;
            }
        };
        PlTangFigure<R,C> pltangfig{m_tangInit, Eigen::Vector2d(1,0), Eigen::Vector2d(0,1)};

        adpscheme.setPen(2, bord2::Red);
        pltangfig.draw(adpscheme);

        adpscheme.setBrush(bord2::Blue);
        adpscheme.moveTo({1.0, 0.0});
        adpscheme.qbezierTo({1.0, 1.0}, {0.0, 1.0});
        adpscheme.qbezierTo({-1.0, 1.0}, {-1.0, 0.0});
        adpscheme.qbezierTo({-1.0, -1.0}, {0.0, -1.0});
        adpscheme.qbezierTo({1.0, -1.0}, {1.0, 0.0});
        adpscheme.closePath();
        adpscheme.fill();

        // Don't forget to release; otherwise, the object scheme would be broken.
        adpscheme.release();
    }

    /**!
     * \section Member functions derived from PathFigure3d
     **/
    virtual void updateProjector(Eigen::Matrix<double,2,3> const &proj) override
    {
        m_critV = proj.row(0);
    }
};

