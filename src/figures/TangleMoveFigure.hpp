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
#include <functional>
#include <Eigen/Dense>

#include "../PlTang.hpp"
#include "../PlTangMove.hpp"
#include "PlTangFigure.hpp"
#include "PathFigure3D.hpp"

#include <iostream> // Debug

template <class TangleType>
class TangleMoveFigure;

template <size_t R, size_t C>
class TangleMoveFigure<PlTang<R,C>> : public PathFigure3D
{
public:
    struct MoveDrawData {
        //using GetCritFunc = Eigen::Vector3d (TangleMoveFigure<PlTang<R,C>>::*)(char);
        Eigen::Vector3d const &baseX, &baseY, &baseZ;
        std::vector<Eigen::Vector3d> const& critvBefore;
        std::vector<Eigen::Vector3d> const& critvAfter;
        /*GetCritFunc*/ // std::function<Eigen::Vector3d(char)> getCrit;
    };

private:
    struct TangleMove {
        using MoveType = PlTangMove<2,2>;

        MoveType move{};
        size_t x{}, y{};
        std::function<void(SchemeType&, MoveDrawData)> draw;

        constexpr TangleMove() noexcept = default;
        template <class F>
        constexpr TangleMove(MoveType const& move0, size_t x0, size_t y0, F&& f) noexcept
        : move(move0), x(x0), y(y0), draw(f)
        {}

        constexpr TangleMove(TangleMove const&) = default;
        ~TangleMove() noexcept = default;

        TangleMove& operator=(TangleMove const&) noexcept = default;
        TangleMove& operator=(TangleMove &&) noexcept = default;
    };

    PlTang<R,C> m_tangDom, m_tangCod;
    std::vector<TangleMove> m_moves{};
    Eigen::Vector3d m_baseX, m_baseY, m_baseZ;
    // Direction in which ridgelines are critical
    Eigen::Vector3d m_critV;

public:
    TangleMoveFigure(PlTang<R,C> const& tang, Eigen::Vector2d const& baseX, Eigen::Vector2d const& baseY)
        : m_tangDom(tang), m_tangCod(tang),
          m_baseX(baseX(0), baseX(1), 0.0),
          m_baseY(baseY(0), baseY(1), 0.0),
          // Be careful for the declaration order of the member m_baseZ.
          m_baseZ(m_baseX.cross(m_baseY) / std::sqrt(m_baseX.norm()*m_baseY.norm()))
    {}

    virtual ~TangleMoveFigure() = default;

    template <class F>
    bool push(typename TangleMove::MoveType const& move, size_t x, size_t y, F&& f) noexcept
    {
        PlTang<2,2> dom = m_tangCod.template slice<2,2>(
            x, y,
            move.getBefore().hlength(),
            move.getBefore().vlength());

        if(dom == move.getBefore()) {
            m_moves.emplace_back(move, x, y, std::forward<F>(f));
            m_tangCod.replace(x, y, move.getAfter());
            return true;
        }
        else {
            return false;
        }
    }

    void pop() noexcept {
        m_moves.pop_back();
    }

    /**!
     * \section Member functions derived from PathFigure
     **/
    virtual void draw(SchemeType &scheme) const override
    {
        Eigen::Vector3d orig =
            - (static_cast<double>(m_tangDom.hlength())/2.0*m_baseX)
            - (static_cast<double>(m_tangDom.vlength())/2.0*m_baseY)
            - (static_cast<double>(m_moves.size())/2.0*m_baseZ);

        AdapterScheme<PathScheme<Eigen::Vector3d>, Eigen::Vector2d> adpscheme{
            &scheme,
                [this, &orig](Eigen::Vector2d const&p) {
                return p(0)*m_baseX + p(1)*m_baseY + orig;
            }
        };
        PlTangFigure pltangfig{m_tangDom, Eigen::Vector2d(1,0), Eigen::Vector2d(0,1)};

        adpscheme.setPen(2, bord2::Red);
        pltangfig.draw(adpscheme);

        // Don't forget to release; otherwise, the object scheme would be broken by std::unique_ptr.
        adpscheme.release();

        // Draw cobordisms constructed from moves.
        PlTang<R,C> tang_cur = m_tangDom;
        size_t r = 0;
        for(size_t s = 0; r < m_moves.size(); ++s) {
            std::vector<unsigned char> mvtable(tang_cur.hlength()*tang_cur.vlength(), 0);
            std::vector<TangleMove const*> oncemvs{};

            // Determine moves simultaneously applied in the step.
            while(r < m_moves.size()) {
                auto &mv = m_moves[r];

                if (!mvtable[mv.x + mv.y*tang_cur.hlength()]
                    && !mvtable[(mv.x+1) + mv.y*tang_cur.hlength()]
                    && !mvtable[mv.x + (mv.y+1)*tang_cur.hlength()]
                    && !mvtable[(mv.x+1) + (mv.y+1)*tang_cur.hlength()])
                {
                    mvtable[mv.x + mv.y*tang_cur.hlength()] = 1;
                    mvtable[(mv.x+1) + mv.y*tang_cur.hlength()] = 1;
                    mvtable[mv.x + (mv.y+1)*tang_cur.hlength()] = 1;
                    mvtable[(mv.x+1) + (mv.y+1)*tang_cur.hlength()] = 1;

                    oncemvs.push_back(&mv);
                    ++r;
                }
                else {
                    break;
                }
            }

            // Draw identities.
            for(size_t j = 0; j < tang_cur.vlength(); ++j) {
                tang_cur.forElTang(j, [&, this](size_t i, char c){
                        if (mvtable[i + j*m_tangDom.hlength()] != 0)
                            return;

                        auto crit = getCrit(c);
                        // Since getCrit returns exactly Eigen::Vector3d(0,0,0) when it fails, we can detect it by this inequality (or even equality ==0).
                        if (crit.norm() <= 0.0)
                            return;

                        scheme.setPen(1, bord2::Black);
                        scheme.moveTo(
                            crit + i*m_baseX + j*m_baseY + orig + s*m_baseZ);
                        scheme.lineTo(
                            crit + i*m_baseX + j*m_baseY + orig + (s+1)*m_baseZ);
                        scheme.stroke();
                    });
            }

            // Draw cobordisms associated to moves in this step and apply them.
            for(auto mv : oncemvs) {
                // Move to the upper-left corner of the move.
                scheme.save();
                scheme.translate(
                    orig + (mv->x)*m_baseX + (mv->y)*m_baseY + s*m_baseZ);
                
                // Compute critical values in the move.
                std::vector<Eigen::Vector3d> critvsB{}, critvsA{};
                for(size_t j = 0; j < mv->move.getBefore().vlength(); ++j) {
                    mv->move.getBefore().forElTang(
                        j,
                        [&](size_t i, char c){
                            auto crv = getCrit(c);
                            if(crv.norm() > 0)
                                critvsB.push_back(crv + i*m_baseX + j*m_baseY);
                        });
                    mv->move.getAfter().forElTang(
                        j,
                        [&](size_t i, char c){
                            auto crv = getCrit(c);
                            if(crv.norm() > 0)
                                critvsA.push_back(crv + i*m_baseX + j*m_baseY);
                        });
                }

                mv->draw(
                    scheme,
                    MoveDrawData{m_baseX, m_baseY, m_baseZ, critvsB, critvsA});

                scheme.restore();

                //* Debug
                /*
                if (std::strcmp(mv->move.getName(), "ExtendR") == 0) {
                    std::cout << __FILE__":" << __LINE__ << std::endl;
                    std::cout << "critvsB:" << std::endl;
                    for(size_t k = 0; k < critvsB.size(); ++k)
                        std::cout << critvsB[k].adjoint() << std::endl;
                    std::cout << "critvsA:" << std::endl;
                    for(size_t k = 0; k < critvsA.size(); ++k)
                        std::cout << critvsA[k].adjoint() << std::endl;
                }
                // *//*

                if(critvsB.empty() && critvsA.size() == 1) {
                    auto innB = mv->move.getBefore().takeInnPt();
                    if(innB[0] > 0.0) {
                        auto v = origloc + innB[0]*m_baseX + innB[1]*m_baseY;

                        scheme.moveTo(v);
                        scheme.bezierTo(
                            v + baseZ,
                            origloc + critvsA[0],
                            origloc + critvsA[0] + baseZ);

                        //* Debug
                        std::cout << __FILE__":" << __LINE__ << std::endl;
                        std::cout << v.adjoint() << std::endl;
                        std::cout << (v+baseZ).adjoint() << std::endl;
                        std::cout << (origloc+critvsA[0]).adjoint() << std::endl;
                        std::cout << (origloc+critvsA[0]+baseZ).adjoint() << std::endl;
                        // *//*
                    }
                }
                else if(critvsB.size() == 1 && critvsA.empty()) {
                    auto innA = mv->move.getAfter().takeInnPt();
                    if(innA[0] > 0.0) {
                        auto v = origloc + innA[0]*m_baseX + innA[1]*m_baseY;

                        scheme.moveTo(v + baseZ);
                        scheme.bezierTo(
                            v,
                            origloc + critvsB[0] + baseZ,
                            origloc + critvsB[0]);
                    }
                }
                else if(critvsB.size() == 1 && critvsA.size() == 1) {
                    scheme.moveTo(origloc + critvsB[0]);
                    scheme.bezierTo(
                        origloc + critvsB[0] + baseZ,
                        origloc + critvsA[0],
                        origloc + critvsA[0] + baseZ);
                }
                else if(!critvsB.empty() || !critvsA.empty()) {
                    if(critvsB.size() >= 2) {
                        scheme.moveTo(origloc + critvsB[0]);
                        scheme.bezierTo(
                            origloc + critvsB[0] + baseZ,
                            origloc + critvsB[1] + baseZ,
                            origloc + critvsB[1]);
                    }
                    if(critvsA.size() >= 2) {
                        scheme.moveTo(origloc + critvsA[0] + baseZ);
                        scheme.bezierTo(
                            origloc + critvsA[0],
                            origloc + critvsA[1],
                            origloc + critvsA[1] + baseZ);
                    }
                }
                scheme.stroke();

                tang_cur.replace(
                    mv->x,
                    mv->y,
                    mv->move.getAfter());
                             */
            }

            /*
            // Draw the tangle in this step.
            AdapterScheme<Eigen::Vector3d, Eigen::Vector2d> mvscheme{
                &scheme,
                    [this, &orig, &baseZ, &s](Eigen::Vector2d const&p) {
                    return p(0)*m_baseX + p(1)*m_baseY + orig + (s+1)*baseZ;
                }
            };
            PlTangFigure<R,C> tangfig{
                tang_cur, Eigen::Vector2d(1,0), Eigen::Vector2d(0,1) };
            mvscheme.setPen(2, bord2::Red);
            tangfig.draw(mvscheme);

            mvscheme.release();
            */
        }
    }

    /**!
     * \section Member functions derived from PathFigure3d
     **/
    virtual void updateProjector(Eigen::Matrix<double,2,3> const &proj) override
    {
        m_critV = proj.row(0).adjoint();
    }

protected:
    Eigen::Vector3d getCrit(char c) const {
        Eigen::Vector3d v0{}, v1{0.5*m_baseX + 0.5*m_baseY}, v2{};

        switch(c) {
        case 'L':
            v0 = 0.5*m_baseX;
            v2 = m_baseX + 0.5*m_baseY;
            break;

        case 'J':
            v0 = 0.5*m_baseY;
            v2 = 0.5*m_baseX;
            break;

        case 'r':
            v0 = 0.5*m_baseX + m_baseY;
            v2 = m_baseX + 0.5*m_baseY;
            break;

        case '7':
            v0 = 0.5*m_baseY;
            v2 = 0.5*m_baseX + m_baseY;
            break;

        default:
            return Eigen::Vector3d{0.0, 0.0, 0.0};
        }

        double numer = m_critV.adjoint() * (v2 - v1);
        double denom = m_critV.adjoint() * (v0 - 2.0*v1 + v2);
        double t = numer / denom;

        return (0.0 < t && t < 1.0)
                      ? (t*t*v0 + 2*t*(1-t)*v1 + (1-t)*(1-t)*v2)
                      : Eigen::Vector3d{0.0, 0.0, 0.0};
    }
};

