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
#include "../Graph.hpp"
#include "../GenericGraph.hpp"
#include "PlTangFigure.hpp"
#include "PathFigure3D.hpp"

#include <iostream> // Debug

template <class T>
class TangleMoveFigure;

template <size_t R, size_t C>
class TangleMoveFigure<PlTang<R,C>> : public PathFigure3D
{
public:
    struct MoveDrawData {
        size_t x, y, z;
        Eigen::Vector2d vproj;
    };

private:
    enum ConnectionVertex : size_t {
        BeforeUL = 0,
        BeforeUR = 1,
        BeforeDL = 2,
        BeforeDR = 3,
        AfterUL  = 4,
        AfterUR  = 5,
        AfterDL  = 6,
        AfterDR  = 7
    };

    //! Tangles in the domain and the codomain of the bordism.
    PlTang<R,C> m_tangDom{}, m_tangCod{};

    //! Sequence of grouped moves.
    std::vector<PlTangMove<2,2>::MoveSeq> m_mvseqs{};

    //! Basis matrix for the 3d-space in rendering.
    Eigen::Matrix3d m_base;

    //! Vector normal to ridge-lines.
    Eigen::Vector2d m_critV;

    //! Paths in drawing.
    std::vector<std::vector<std::pair<Eigen::Vector3d,bool>>> m_paths{};

public:
    TangleMoveFigure(PlTang<R,C> const& tang,
                     std::vector<PlTangMove<2,2>::MoveSeq> const& mvseqs,
                     Eigen::Matrix3d const& base)
        : m_tangDom(tang), m_tangCod(tang),
          m_mvseqs(mvseqs),
          m_base(base)
    {
        m_critV = Eigen::Vector2d(1.0, 0.0);

        for(auto& seq : mvseqs) {
            for(auto& mv : seq) {
                m_tangCod.replace(mv.x, mv.y, mv.move.getAfter());
            }
        }

        makePath();
    }

    void setTangleMove(PlTang<R,C> const& tang,
                       std::vector<PlTangMove<2,2>::MoveSeq> const& mvseqs,
                       Eigen::Matrix3d const& base)
    {
        m_tangDom = tang;
        m_tangCod = tang;
        m_base = base;
        m_mvseqs = mvseqs;

        for(auto& seq : mvseqs) {
            for(auto& mv : seq) {
                m_tangCod.replace(mv.x, mv.y, mv.move.getAfter());
            }
        }

        makePath();
    }

    virtual ~TangleMoveFigure() = default;

    /**!
     * \section Member functions derived from PathFigure
     **/
    virtual void draw(SchemeType &scheme) const override
    {
        Eigen::Vector3d orig
            = -m_base * Eigen::Vector3d(
                static_cast<double>(m_tangDom.hlength())/2.0,
                static_cast<double>(m_tangDom.vlength())/2.0,
                static_cast<double>(m_mvseqs.size())/2.0 );

        scheme.save();
        scheme.translate(orig);

        /* Draw the domain tangle */{
            AdapterScheme<PathScheme<Eigen::Vector3d>, Eigen::Vector2d> adpscheme(
                &scheme,
                [this, &orig](Eigen::Vector2d const&p) {
                    return m_base.block<3,2>(0,0)*p;
                } );

            PlTangFigure pltangfig{ m_tangDom,
                    Eigen::Vector2d(1.0, 0.0),
                    Eigen::Vector2d(0.0, 1.0) };

            adpscheme.setPen(2, bord2::Red);
            pltangfig.draw(adpscheme);
            adpscheme.release();
        }

        for(auto& path : m_paths) {
            if (path.size() <= 1)
                continue;

            scheme.moveTo(m_base * path[0].first);

            for(size_t i = 1; i < path.size(); ++i) {
                Eigen::Vector3d c1 = m_base*path[i-1].first
                    + path[i-1].second*m_base.col(2);
                Eigen::Vector3d c2 = m_base*path[i].first
                    + path[i].second*m_base.col(2);
                scheme.bezierTo(c1, c2, m_base*path[i].first);
            }
        }

        /* Draw the codomain tangle */{
            AdapterScheme<PathScheme<Eigen::Vector3d>, Eigen::Vector2d> adpscheme(
                &scheme,
                [this, &orig](Eigen::Vector2d const&p) {
                    return m_base.block<3,2>(0,0)*p
                        + m_mvseqs.size()*m_base.col(2);
                } );

            PlTangFigure pltangfig{ m_tangCod,
                    Eigen::Vector2d(1.0, 0.0),
                    Eigen::Vector2d(0.0, 1.0) };

            adpscheme.setPen(2, bord2::Red);
            pltangfig.draw(adpscheme);
            adpscheme.release();
        }
        scheme.restore();
    }

    /**!
     * \section Member functions derived from PathFigure3d
     **/
    virtual void updateProjector(Eigen::Matrix<double,2,3> const &proj) override
    {
        m_critV = (proj*m_base.block<3,2>(0,0)).row(0).adjoint();
        makePath();
    }

protected:
    Eigen::Vector2d getCrit(char c) const {
        Eigen::Vector2d v0, v1{Eigen::Vector2d(0.5,0.5)}, v2;

        switch(c) {
        case 'L':
            v0 = Eigen::Vector2d(0.5, 0.0);
            v2 = Eigen::Vector2d(1.0, 0.5);
            break;

        case 'J':
            v0 = Eigen::Vector2d(0.0, 0.5);
            v2 = Eigen::Vector2d(0.5, 0.0);
            break;

        case 'r':
            v0 = Eigen::Vector2d(0.5, 1.0);
            v2 = Eigen::Vector2d(1.0, 0.5);
            break;

        case '7':
            v0 = Eigen::Vector2d(0.0, 0.5);
            v2 = Eigen::Vector2d(0.5, 1.0);
            break;

        default:
            return Eigen::Vector2d{0.0, 0.0};
        }

        double numer = m_critV.adjoint() * (v2 - v1);
        double denom = m_critV.adjoint() * (v0 - 2.0*v1 + v2);
        double t = numer / denom;

        return (0.0 < t && t < 1.0)
                      ? (t*t*v0 + 2*t*(1-t)*v1 + (1-t)*(1-t)*v2)
                      : Eigen::Vector2d{0.0, 0.0};
    }

    void makePath() {
        // Clear the old paths
        m_paths.clear();

        PlTang<R,C> tang_cur = m_tangDom;
        GenericGraph<std::map<size_t,Eigen::Vector3d>> graph;
        std::vector<size_t> levels(m_mvseqs.size(), 0);

        // Counter for the layers.
        size_t q = 0;

        // Traverse all the grouped moves
        for (size_t r = 0; r < m_mvseqs.size(); ++r) {
            std::vector<unsigned char> mvtbl(tang_cur.hlength()*tang_cur.vlength(), 0);
            size_t s = 0;
            for(auto& mv : m_mvseqs[r]) {
                // If the next move commutes with olds, draw it in the same level.
                if(!mvtbl[mv.x + mv.y*tang_cur.hlength()]
                    && !mvtbl[(mv.x+1) + mv.y*tang_cur.hlength()]
                    && !mvtbl[mv.x + (mv.y+1)*tang_cur.hlength()]
                    && !mvtbl[(mv.x+1) + (mv.y+1)*tang_cur.hlength()])
                {
                    mvtbl[mv.x + mv.y*tang_cur.hlength()] = 1;
                    mvtbl[(mv.x+1) + mv.y*tang_cur.hlength()] = 1;
                    mvtbl[mv.x + (mv.y+1)*tang_cur.hlength()] = 1;
                    mvtbl[(mv.x+1) + (mv.y+1)*tang_cur.hlength()] = 1;
                }
                // If it doesn't commute, go to the next level.
                else {
                    ++s;
                    std::fill(mvtbl.begin(), mvtbl.end(), 0);
                }

                std::map<size_t, Eigen::Vector2d> critmap;
                size_t wid = mv.move.getBefore().hlength();
                size_t hei = mv.move.getBefore().vlength();

                // Compute critical point in ridgeline-normal direction.
                for(size_t j = 0; j < mv.move.getBefore().vlength(); ++j) {
                    mv.move.getBefore().forElTang(
                        j,
                        [&mv, &j, &critmap, this, wid](size_t i, char c) {
                            auto crv = getCrit(c);
                            if(crv.norm() > 0)
                                // Make sure the row-major ordering.
                                critmap.emplace(
                                    i+j*wid,
                                    crv + Eigen::Vector2d(mv.x+i, mv.y+j));
                        } );
                    mv.move.getAfter().forElTang(
                        j,
                        [&mv, &j, &critmap, this, wid, hei](size_t i, char c) {
                            auto crv = getCrit(c);
                            if(crv.norm() > 0)
                                // Make sure the row-major ordering.
                                critmap.emplace(
                                    i+j*wid + wid*hei,
                                    crv + Eigen::Vector2d(mv.x+i, mv.y+j));
                        } );
                }

                // Connect critical points following the connection graph.
                mv.move.getGraph().forEachEdge(
                    [&](size_t m, size_t n) {
                        auto itrm = critmap.find(m);
                        auto itrn = critmap.find(n);

                        if (itrm == critmap.end() && itrn == critmap.end())
                            return;

                        auto vm = itrm == critmap.end()
                            ? Eigen::Vector3d(
                                mv.x + m%2 + 0.5,
                                mv.y + m/2 + 0.5,
                                m >= 4 ? q+s+1 : q+s )
                            : Eigen::Vector3d(
                                itrm->second(0),
                                itrm->second(1),
                                m >= 4 ? q+s+1 : q+s );
                        auto vn = itrn == critmap.end()
                            ? Eigen::Vector3d(
                                mv.x + n%2 + 0.5,
                                mv.y + n/2 + 0.5,
                                n >= 4 ? q+s+1 : q+s )
                            : Eigen::Vector3d(
                                itrn->second(0),
                                itrn->second(1),
                                n >= 4 ? q+s+1 : q+s );

                        auto keym = findAppend(graph, vm);
                        auto keyn = findAppend(graph, vn);

                        // If vm and vn are at the same height, append the middle point in the edge to indicate whether the path should be over-convex or under-convex.
                        if ( (m >= 4) == (n >= 4) ) {
                            auto keymid = graph.append(
                                Eigen::Vector3d(
                                    mv.x+1.0, mv.y+1.0, q+s+0.5 ));
                            graph.connect(keym, keymid);
                            graph.connect(keyn, keymid);
                        }
                        // Otherwise, just connect two critical points directly.
                        else {
                            graph.connect(keym, keyn);
                        }
                    } );
            } // end of traverse in m_mvseqs[r].

            // Now the variable s equals the number of levels of the bordism constructed from the grouped moves.
            levels[r] = s;

            // Advance the counter of layers.
            q += s;
        } // end of traverse of the array m_mvseqs.

        // Hight modificator.
        auto modif = [&levels] (Eigen::Vector3d v) -> Eigen::Vector3d {
            size_t r = 0;
            for(size_t q = 0; v(2) > q+levels[r] && r < levels.size(); q+=levels[r])
                ++r;
            v(2) = (v(2)-r)/static_cast<double>(levels[r]) + r;
            return v;
        };
        // Integral detector
        auto isInt = [](double x) -> bool {
            constexpr double threshold = 10e-10;
            return std::abs(x-std::round(x)) < threshold;
        };

        while(graph.inhabited()) {
            auto comp = graph.trimComponent(graph.minkey());
            auto mayend = comp.findEnd();
            m_paths.emplace_back();

            double prevlevel = -1.0;
            comp.trackPath(
                mayend.first ? mayend.second : graph.minkey(),
                [&](std::pair<size_t, Eigen::Vector3d> const& v) {
                    if (isInt(v.second(2))) {
                        m_paths.back().emplace_back(
                            modif(v.second),
                            v.second(2) > prevlevel ? 1 : -1);
                    }
                    prevlevel = v.second(2);
                });
            /*** BOOKMERK **/
        }
    }
};

